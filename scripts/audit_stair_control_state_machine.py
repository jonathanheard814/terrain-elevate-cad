#!/usr/bin/env python3
"""Exhaustive safety verification of the stair-climb control state machine.

Truth boundary: this proves properties of the control LOGIC as written in
src/terrain_elevate/stair_control.py. It is not a real-time timing analysis,
not a hardware FMEA, and not functional-safety certification. What it does
give is completeness: because `step` is a pure function of (state, predicates),
every state can be enumerated against every combination of predicates, so these
properties are checked over the whole transition space rather than over a
handful of hand-written scenarios.

Unlike the sizing screens, a failure here is a genuine logic defect rather than
an honest engineering finding, so this audit hard-fails the build.
"""
from __future__ import annotations

import itertools
import json
import sys
from dataclasses import fields
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.stair_control import (  # noqa: E402
    ON_STAIR_STATES,
    Predicates,
    SensorFrame,
    State,
    Thresholds,
    derive_predicates,
    step,
)

#: The independent conditions that must ALL hold to commit to a staircase.
#: Each maps to a different physical source, which is what makes the ablation
#: test below meaningful.
INTENT_PREDICATES = (
    "deadman",
    "force_pattern_ok",
    "dwell_satisfied",
    "committal_requested",
)

DRIVING_STATES = (State.CLIMBING, State.DESCENDING)


def _all_predicate_combinations():
    names = [f.name for f in fields(Predicates)]
    for values in itertools.product((False, True), repeat=len(names)):
        yield Predicates(**dict(zip(names, values)))


def main() -> None:
    violations: list[dict] = []
    transitions = 0
    reached_states: set[State] = set()

    def fail(prop: str, state: State, p: Predicates, detail: str) -> None:
        violations.append({
            "property": prop,
            "from_state": state.value,
            "predicates": {f.name: getattr(p, f.name) for f in fields(Predicates)},
            "detail": detail,
        })

    for state in State:
        for p in _all_predicate_combinations():
            transitions += 1
            nxt, out = step(state, p)
            reached_states.add(nxt)

            # 1. No free-fall: committed to a staircase without drive authority
            #    means the mechanical holds must be applied.
            if out.stair_mode_active and not out.wheel_drive_enabled:
                if not (out.holding_brakes_engaged and out.anti_drop_pawls_engaged):
                    fail("no_free_fall_on_stairs", state, p,
                         "stair mode without drive authority did not engage both "
                         "holding brakes and anti-drop pawls")

            # 2. Deadman is absolute: releasing it always removes drive.
            if not p.deadman and out.wheel_drive_enabled:
                fail("deadman_removes_drive", state, p,
                     "wheel drive enabled with deadman released")

            # 3. Faults never leave the vehicle driving.
            if p.fault and (out.wheel_drive_enabled or out.corner_actuators_enabled):
                fail("fault_disables_actuation", state, p,
                     "actuation still enabled while a fault is present")
            if p.fault and not (out.holding_brakes_engaged and out.anti_drop_pawls_engaged):
                fail("fault_engages_holds", state, p,
                     "fault did not engage both holding brakes and anti-drop pawls")

            # 4. Reverse requires an explicit, dedicated request. This is the
            #    requirement that reversing must never be INFERRED from the
            #    caregiver merely easing off the handle.
            if out.wheel_drive_direction == -1 and not p.reverse_requested:
                fail("reverse_requires_explicit_intent", state, p,
                     "reverse commanded without the explicit reverse request")

            # 5. A curb or threshold must not become a stair climb.
            if nxt is State.SINGLE_CORNER_EVENT and out.stair_mode_active:
                fail("single_corner_is_not_stair_mode", state, p,
                     "single-corner event entered stair mode")

            # 6. Drive authority implies the deadman is held.
            if out.wheel_drive_enabled and not p.deadman:
                fail("drive_implies_deadman", state, p, "drive without deadman")

            # 7. Brakes and drive are mutually exclusive.
            if out.holding_brakes_engaged and out.wheel_drive_enabled:
                fail("brakes_and_drive_exclusive", state, p,
                     "holding brakes engaged while wheel drive enabled")

    # 8. No single point of failure on stair committal. For each independent
    #    intent condition, no state and no combination of the others may reach
    #    a driving-on-stairs state while that one condition is false. This is
    #    the ablation that makes "no single sensor failing high can commit the
    #    vehicle to a staircase" a verified claim rather than an assertion.
    #    The preview cross-check is ablated as a pair: climbing keys off the
    #    front sensor and descending off the rear, so neither alone gates both
    #    directions. Suppressing both is what tests the requirement that the
    #    preview sensor must actually confirm a stair before committal --
    #    without it, a held button plus a shove would be enough.
    ablation_groups: tuple[tuple[str, tuple[str, ...]], ...] = tuple(
        (name, (name,)) for name in INTENT_PREDICATES
    ) + (("preview_confirms_stair", ("stair_confirmed_ahead", "stair_confirmed_behind")),)

    ablation: dict[str, dict] = {}
    for label, missing_names in ablation_groups:
        breaches = 0
        for state in State:
            for p in _all_predicate_combinations():
                if any(getattr(p, n) for n in missing_names):
                    continue
                nxt, _ = step(state, p)
                if nxt in DRIVING_STATES and state not in DRIVING_STATES:
                    breaches += 1
                    fail("committal_no_single_point_of_failure", state, p,
                         f"entered {nxt.value} with '{label}' false")
        ablation[label] = {
            "blocks_entry_to_stair_driving": breaches == 0,
            "breaches": breaches,
        }

    # 9. No unreachable states (dead logic) and no undefined targets.
    unreachable = sorted(s.value for s in State if s not in reached_states)

    # 10. Threshold consistency in the sensor -> predicate layer. `step` is
    #     verified above over predicates directly, so nothing there catches a
    #     bad threshold CHOICE. The hazard is concrete: in GROUND_ROLLING the
    #     single-corner branch is evaluated before the stair branch, so if the
    #     single-corner rise ceiling ever exceeded the stair-detection floor, a
    #     real staircase in that overlap would be answered as a curb -- one
    #     corner twitching instead of a stair committal. Assert the bands
    #     cannot overlap, and that a landing cannot simultaneously read as a
    #     stair.
    t = Thresholds()
    threshold_checks = {
        "single_corner_band_below_stair_band": (
            t.max_single_corner_rise_mm <= t.min_stair_rise_mm
        ),
        "landing_clear_below_stair_floor": (
            t.landing_clear_rise_mm < t.min_stair_rise_mm
        ),
        "committal_dwell_positive": t.min_committal_dwell_s > 0.0,
        "handle_force_positive": t.min_handle_force_N > 0.0,
    }
    for name, ok in threshold_checks.items():
        if not ok:
            violations.append({
                "property": "threshold_consistency",
                "from_state": None,
                "predicates": None,
                "detail": f"threshold check '{name}' failed",
            })

    # 11. The derived-predicate layer must honour the same separation on real
    #     sensor values, not just on the threshold numbers.
    def _frame(front_mm: float, spread_mm: float) -> SensorFrame:
        return SensorFrame(
            deadman_engaged=True, handle_force_N=40.0, handle_force_sustained_s=2.0,
            stair_committal_button=True, reverse_committal_button=False,
            front_preview_rise_mm=front_mm, rear_preview_rise_mm=0.0,
            chassis_pitch_deg=0.0, corner_extension_spread_mm=spread_mm,
            wheel_speed_mps=0.0, fault_present=False,
        )

    for front_mm, spread_mm, label in (
        (203.2, 90.0, "reference stair rise with a large corner spread"),
        (150.0, 90.0, "intermediate rise with a large corner spread"),
        (60.0, 90.0, "curb-height rise with a large corner spread"),
    ):
        dp = derive_predicates(_frame(front_mm, spread_mm))
        if dp.stair_confirmed_ahead and dp.single_corner_event:
            violations.append({
                "property": "stair_and_single_corner_mutually_exclusive",
                "from_state": None,
                "predicates": {"front_preview_rise_mm": front_mm,
                               "corner_extension_spread_mm": spread_mm},
                "detail": f"{label} classified as BOTH a stair and a single-corner event",
            })

    result = {
        "truth_boundary": (
            "Exhaustive verification of the control logic's transition table. Not a "
            "real-time timing analysis, not a hardware FMEA, not functional-safety "
            "certification."
        ),
        "method": (
            "step() is a pure function of (state, predicates), so every State was "
            "enumerated against every combination of the boolean predicates and each "
            "safety property checked on the resulting transition. The committal "
            "property is additionally verified by ablation: each independent intent "
            "condition is forced false in turn and the whole space re-enumerated to "
            "confirm entry to stair driving becomes unreachable."
        ),
        "states": len(State),
        "predicates": len(fields(Predicates)),
        "transitions_checked": transitions,
        "properties_verified": [
            "no_free_fall_on_stairs",
            "deadman_removes_drive",
            "fault_disables_actuation",
            "fault_engages_holds",
            "reverse_requires_explicit_intent",
            "single_corner_is_not_stair_mode",
            "drive_implies_deadman",
            "brakes_and_drive_exclusive",
            "committal_no_single_point_of_failure",
            "threshold_consistency",
            "stair_and_single_corner_mutually_exclusive",
        ],
        "threshold_checks": threshold_checks,
        "committal_ablation": ablation,
        "unreachable_states": unreachable,
        "violations": violations[:50],
        "violation_count": len(violations),
        "open_items": [
            "All thresholds in stair_control.Thresholds are ASSUMED engineering "
            "placeholders, not tuned or validated values; the logic is verified "
            "against them symbolically, but the numbers themselves close only with a "
            "real HMI study (handle force and committal dwell) and the dynamic model "
            "(pitch and corner-spread limits).",
            "Timing is not modelled here: this verifies the transition table, not "
            "that it executes within the sub-millisecond local control loop. Closes "
            "with a real-time implementation and a scheduling analysis.",
            "Sensor faults are modelled only as a single aggregate fault_present "
            "latch. Per-sensor failure modes, disagreement between the redundant "
            "IMUs, and stuck-value detection are not yet represented; closes with a "
            "hardware FMEA.",
        ],
        "result": "PASS" if not violations and not unreachable else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "Terrain_Elevate_P1_V0_59_stair_control_audit.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps(result, indent=2))
    if result["result"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
