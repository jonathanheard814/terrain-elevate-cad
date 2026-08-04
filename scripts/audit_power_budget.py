#!/usr/bin/env python3
"""Electrical power budget: real load sum vs. sourced battery pack.

Sums actual electrical draw from every already-sourced powered subsystem
(corner actuators, wheel drives, pod pitch actuators, control/sensor
electronics) under two scenarios -- continuous climbing and worst-case
simultaneous peak -- and checks both against the sourced battery pack's
continuous discharge rating and against a target continuous-climb runtime.

Truth boundary: engineering sizing screen. Not a certified electrical design,
not a UL/IEC compliance review, not a thermal analysis.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _ride_quality_actuator_power_crosscheck(
    root: Path,
    system_voltage: float,
    corner_duty: float,
    corner_peak_count: int,
    pack_continuous_limit_A: float,
    usable_energy_Wh: float,
    target_runtime_minutes: float,
) -> dict:
    """Can the sourced pack power the actuator ride_quality actually requires?

    Deliberately computed at a physically impossible 100% electrical-to-
    mechanical efficiency, and counting the corner actuators ONLY (no wheel
    drives, pod actuators, or electronics baseline). Every real motor, drive,
    gearbox and screw makes these numbers strictly worse, so a FAIL here is a
    hard lower-bound result that no efficiency assumption can argue away.
    """
    audit_path = root / "analysis_out" / "Terrain_Elevate_P1_V0_59_smooth_climb_audit.json"
    if not audit_path.exists():
        return {
            "applicable": False,
            "reason": (
                "smooth_climb_audit.json not found -- run "
                "scripts/simulate_smooth_stair_climb.py first. Cross-check skipped, "
                "not passed."
            ),
            "result": "NOT_RUN",
        }

    climb = json.loads(audit_path.read_text())
    ride_quality = climb.get("ride_quality", {})
    spec = climb.get("actuator_reselection_spec", {})

    if ride_quality.get("result") == "PASS":
        return {
            "applicable": False,
            "reason": (
                "ride_quality passes with the currently sourced corner actuator, so "
                "the power budget above already describes the real machine."
            ),
            "result": "NOT_APPLICABLE",
        }

    required_mech_W = spec.get("required_mechanical_power_at_screw_W")
    if not required_mech_W:
        return {
            "applicable": False,
            "reason": (
                "ride_quality fails but no achievable window was found, so no "
                "required actuator power exists to cross-check against. See "
                "smooth_climb_audit.json open_items."
            ),
            "result": "NOT_APPLICABLE",
        }

    ideal_continuous_W = required_mech_W * 4 * corner_duty
    ideal_peak_W = required_mech_W * corner_peak_count
    ideal_continuous_A = ideal_continuous_W / system_voltage
    ideal_peak_A = ideal_peak_W / system_voltage
    per_corner_A = required_mech_W / system_voltage

    # Duty-assumption-free framing: how many corners can extend at the
    # required rate simultaneously before the pack's own rating is exceeded.
    corners_within_pack = int(pack_continuous_limit_A // per_corner_A)
    ideal_runtime_minutes = (usable_energy_Wh / ideal_continuous_W) * 60.0

    continuous_ok = ideal_continuous_A <= pack_continuous_limit_A
    runtime_ok = ideal_runtime_minutes >= target_runtime_minutes

    return {
        "applicable": True,
        "method": (
            "Substitutes the corner-actuator mechanical power that ride_quality "
            "requires (smooth_climb_audit.json actuator_reselection_spec) into the "
            "same duty/simultaneity assumptions used above, at an idealised 100% "
            "electrical-to-mechanical efficiency and counting corner actuators only."
        ),
        "required_mechanical_power_per_corner_W": required_mech_W,
        "idealised_continuous_A": ideal_continuous_A,
        "idealised_peak_A": ideal_peak_A,
        "per_corner_A_at_100pct_efficiency": per_corner_A,
        "pack_max_continuous_discharge_A": pack_continuous_limit_A,
        "corners_simultaneously_extendable_within_pack_rating": corners_within_pack,
        "corner_count": 4,
        "idealised_runtime_minutes_corners_only": ideal_runtime_minutes,
        "target_runtime_minutes": target_runtime_minutes,
        "continuous_current_result": "PASS" if continuous_ok else "FAIL",
        "runtime_result": "PASS" if runtime_ok else "FAIL",
        "result": "PASS" if (continuous_ok and runtime_ok) else "FAIL",
        "interpretation": (
            f"At 100% efficiency and ignoring every other load, {corners_within_pack} "
            f"of 4 corner actuators can extend at the ride-quality-required rate "
            f"within the pack's {pack_continuous_limit_A:.0f} A continuous rating. "
            "The sourced battery cannot power the actuator that ride_quality "
            "requires; this is a lower bound, so no efficiency or duty-cycle "
            "argument recovers it."
        ) if not continuous_ok else (
            "The sourced pack can supply the ride-quality-required actuator even at "
            "these idealised figures; re-check once real drive efficiencies are applied."
        ),
    }


def main() -> None:
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    load_cases = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    physics = json.loads((ROOT / "data" / "te_v059_physics_elements.json").read_text())
    electrical = json.loads((ROOT / "data" / "te_v059_electrical_system.json").read_text())

    g = params["geometry"]
    loads = electrical["load_assumptions"]
    battery = electrical["battery_pack"]
    system_voltage = electrical["system"]["nominal_voltage_V"]

    # Corner actuators.
    corner_current_A = loads["corner_actuator_current_A_SRC"]
    corner_peak_count = loads["corner_actuators_simultaneous_peak_ASSUMED"]
    corner_duty = loads["corner_actuator_average_duty_fraction_ASSUMED"]
    corner_peak_A = corner_current_A * corner_peak_count
    corner_continuous_A = corner_current_A * 4 * corner_duty

    # Wheel drives: real mechanical power from the already-screened climb
    # torque and forward speed, not the motor's "up to" peak rating.
    wheel_radius_m = g["wheel_diameter_mm"] / 2000.0
    forward_speed_mps = load_cases["combined_climb_assumptions"]["stair_climb_forward_speed_mps_ASSUMED"]
    wheel_angular_speed_rad_s = forward_speed_mps / wheel_radius_m
    per_wheel_torque_Nm = physics["actuator_models"]["wheel_drive"]["screened_per_wheel_torque_Nm"]
    per_wheel_mech_W = per_wheel_torque_Nm * wheel_angular_speed_rad_s
    wheel_efficiency = loads["wheel_drive_mechanical_efficiency_ASSUMED"]
    per_wheel_electrical_W = per_wheel_mech_W / wheel_efficiency
    wheel_continuous_A = (per_wheel_electrical_W * 4) / system_voltage
    # Peak: same four wheels drawing continuously is already the sustained
    # case; there is no separate "peak" wheel scenario beyond this in the
    # combined ramp-like climb (wheel-only stair lifting is not the
    # mechanism -- see requirements_screen.json wheel_propulsion_screen).
    wheel_peak_A = wheel_continuous_A

    # Pod pitch actuators.
    pod_power_W = loads["pod_pitch_actuator_power_W_ASSUMED"]
    pod_active_fraction = loads["pod_pitch_active_fraction_ASSUMED"]
    pod_peak_A = (pod_power_W * 2) / system_voltage
    pod_continuous_A = pod_peak_A * pod_active_fraction

    # Control/sensor electronics baseline.
    electronics_A = loads["electronics_baseline_continuous_W_ASSUMED"] / system_voltage

    continuous_A = corner_continuous_A + wheel_continuous_A + pod_continuous_A + electronics_A
    peak_A = corner_peak_A + wheel_peak_A + pod_peak_A + electronics_A

    # Cross-check against the actuator ride_quality actually demands.
    #
    # Everything above sizes the pack against the CURRENTLY SOURCED corner
    # motor (maxon EC-i 52, corner_actuator_current_A_SRC). But
    # simulate_smooth_stair_climb.py's ride_quality gate reports that this
    # exact motor is several times too small, and publishes the mechanical
    # power an actuator that DOES satisfy ride_quality would need. Without
    # this cross-check the two audits silently describe different machines:
    # the power budget passes for a vehicle the ride-quality screen has
    # already rejected. Sizing the pack against a motor known to be
    # insufficient is not a PASS worth reporting on its own.
    crosscheck = _ride_quality_actuator_power_crosscheck(
        ROOT, system_voltage, corner_duty, corner_peak_count,
        pack_continuous_limit_A=battery["pack_max_continuous_discharge_A_CALC"],
        usable_energy_Wh=battery["usable_energy_Wh_CALC"],
        target_runtime_minutes=loads["target_continuous_climb_runtime_minutes_ASSUMED"],
    )

    continuous_W = continuous_A * system_voltage
    peak_W = peak_A * system_voltage

    pack_continuous_limit_A = battery["pack_max_continuous_discharge_A_CALC"]
    continuous_current_margin_A = pack_continuous_limit_A - peak_A
    battery_current_result = "PASS" if peak_A <= pack_continuous_limit_A else "FAIL"

    usable_energy_Wh = battery["usable_energy_Wh_CALC"]
    runtime_at_continuous_climb_minutes = (usable_energy_Wh / continuous_W) * 60.0 if continuous_W > 0 else math.inf
    target_runtime_minutes = loads["target_continuous_climb_runtime_minutes_ASSUMED"]
    runtime_result = "PASS" if runtime_at_continuous_climb_minutes >= target_runtime_minutes else "FAIL"

    as_sourced_pass = battery_current_result == "PASS" and runtime_result == "PASS"
    # The as-sourced screen only answers "can the pack run the parts already on
    # the BOM". It is reported separately and kept intact, but it must not be
    # the headline verdict while the cross-check above shows those parts cannot
    # deliver the required ride.
    overall_pass = as_sourced_pass and crosscheck.get("result") != "FAIL"

    result = {
        "truth_boundary": electrical["truth_boundary"],
        "method": (
            "Sums real electrical draw from every already-sourced powered subsystem "
            "under two scenarios: sustained continuous climbing (wheel drives at their "
            "screened climb torque/speed, corner actuators at an averaged duty cycle, "
            "pod actuators at an averaged active fraction) and worst-case simultaneous "
            "peak (multiple corner/pod actuators assumed active at once), then checks "
            "both against the sourced 13S2P battery pack's continuous discharge rating "
            "and a target continuous-climb runtime."
        ),
        "system_nominal_voltage_V": system_voltage,
        "loads": {
            "corner_actuators": {
                "continuous_A": corner_continuous_A,
                "peak_A": corner_peak_A,
                "basis": "maxon EC-i 52 nominal current (SRC) x simultaneity/duty assumptions",
            },
            "wheel_drives": {
                "continuous_A": wheel_continuous_A,
                "peak_A": wheel_peak_A,
                "per_wheel_mechanical_W_CALC": per_wheel_mech_W,
                "basis": "screened climb torque (requirements_screen.json) x forward speed, not motor peak rating",
            },
            "pod_pitch_actuators": {
                "continuous_A": pod_continuous_A,
                "peak_A": pod_peak_A,
                "basis": "ASSUMED Electrak MD-class power x active-fraction duty",
            },
            "electronics_baseline": {
                "continuous_A": electronics_A,
                "peak_A": electronics_A,
                "basis": "ASSUMED summed ECU/sensor/connector baseline",
            },
        },
        "totals": {
            "continuous_A": continuous_A,
            "continuous_W": continuous_W,
            "peak_A": peak_A,
            "peak_W": peak_W,
        },
        "battery_check": {
            "pack_configuration": battery["configuration_CALC"],
            "pack_nominal_voltage_V": battery["pack_nominal_voltage_V_CALC"],
            "pack_max_continuous_discharge_A": pack_continuous_limit_A,
            "screened_peak_draw_A": peak_A,
            "continuous_current_margin_A": continuous_current_margin_A,
            "result": battery_current_result,
        },
        "runtime_check": {
            "usable_energy_Wh": usable_energy_Wh,
            "continuous_climb_draw_W": continuous_W,
            "runtime_at_continuous_climb_minutes": runtime_at_continuous_climb_minutes,
            "target_runtime_minutes": target_runtime_minutes,
            "result": runtime_result,
        },
        "as_sourced_result": "PASS" if as_sourced_pass else "FAIL",
        "as_sourced_result_meaning": (
            "Whether the sourced pack can run the parts currently on the BOM. This "
            "is NOT the same question as whether it can run the machine the design "
            "actually requires -- see ride_quality_actuator_power_crosscheck."
        ),
        "ride_quality_actuator_power_crosscheck": crosscheck,
        "open_items": [
            "pod_pitch_actuator_power_W_ASSUMED and electronics_baseline_continuous_W_ASSUMED "
            "are engineering placeholders, not datasheet values; closes when the Thomson "
            "Electrak MD is configured to order and a per-PCB current budget exists for "
            "the control electronics.",
            "Precharge/contactor sequencing is assumed covered by the Orion Jr BMS's "
            "integrated contactor control (data/te_v059_electrical_system.json); closes "
            "when the actual harness/contactor wiring is drawn and verified against the "
            "BMS's rated switching current.",
            "No thermal analysis of the battery pack, motor drives, or wiring harness "
            "has been performed; closes with a thermal model once enclosure airflow is "
            "designed.",
        ] + ([
            "ride_quality_actuator_power_crosscheck FAILs: the corner actuator that "
            "simulate_smooth_stair_climb.py's ride_quality gate requires cannot be "
            "powered by the sourced 13S2P pack, even at an impossible 100% efficiency "
            "with every other load ignored. This is an architecture-level conflict, "
            "not a component-selection gap -- raising actuator speed raises corner "
            "power roughly in proportion, so the same battery cannot serve both the "
            "ride budget and the runtime target. Closes by changing one of the three "
            "constraints that collide here (ramp-error budget, energy storage, or the "
            "corner mechanism itself), not by re-selecting a motor.",
        ] if crosscheck.get("result") == "FAIL" else []),
        "result": "PASS" if overall_pass else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "Terrain_Elevate_P1_V0_59_power_budget_audit.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
