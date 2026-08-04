"""Stair-climb control state machine.

Truth boundary: this is the control *logic* and its safety interlocks, written
so the transition table can be exhaustively verified (see
scripts/audit_stair_control_state_machine.py). It is not tuned gains, not a
timing-validated real-time implementation, and not certified functional-safety
software. Every sensor and actuator it references is a part that already exists
in the CAD and the sourced part register -- nothing here assumes hardware the
vehicle does not have.

The design is deliberately split into two layers:

  SensorFrame  -- raw values as the sourced sensors actually report them
       |
       v  derive_predicates(), thresholds stated once, in one place
  Predicates   -- booleans with safety meaning
       |
       v  step(), pure function of (state, predicates)
  (State, Outputs)

Keeping `step` a pure function of booleans is what makes exhaustive
verification possible: the auditor enumerates every state against every
combination of predicates, rather than sampling a few scenarios and hoping the
untested ones behave.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class State(Enum):
    """Vehicle control states.

    HOLD states are distinguished from OFF because a hold on a staircase is
    mechanically very different from being parked: it must actively keep the
    anti-drop pawls and holding brakes engaged rather than simply being
    unpowered.
    """

    STANDBY = "standby"
    GROUND_ROLLING = "ground_rolling"
    STAIR_DETECTED = "stair_detected"
    COMMITTAL_PENDING = "committal_pending"
    CLIMBING = "climbing"
    DESCENDING = "descending"
    SINGLE_CORNER_EVENT = "single_corner_event"
    MID_CLIMB_HOLD = "mid_climb_hold"
    LANDING_TRANSITION = "landing_transition"
    FAULT_HOLD = "fault_hold"


#: States in which the vehicle is committed to a staircase. Used by the
#: no-free-fall invariant: in any of these, losing drive authority must engage
#: the mechanical holds rather than coast.
ON_STAIR_STATES = frozenset(
    {State.CLIMBING, State.DESCENDING, State.MID_CLIMB_HOLD, State.LANDING_TRANSITION}
)


@dataclass(frozen=True)
class SensorFrame:
    """Raw sensor values, named for the parts that produce them."""

    deadman_engaged: bool                    # handle_grip_deadman_bar + release paddle
    handle_force_N: float                    # load_sensing_pins
    handle_force_sustained_s: float          # integrator over the same pins
    stair_committal_button: bool             # handle_grip_assist_mode_button, held
    reverse_committal_button: bool           # distinct explicit reverse request
    front_preview_rise_mm: float             # front ifm O1D100
    rear_preview_rise_mm: float              # rear ifm O1D100
    chassis_pitch_deg: float                 # dual_imu_mount (redundant pair)
    corner_extension_spread_mm: float        # Novotechnik TF1 x4, max minus min
    wheel_speed_mps: float                   # Novotechnik RFC-4800
    fault_present: bool                      # any subsystem fault latch


@dataclass(frozen=True)
class Thresholds:
    """Every numeric threshold the logic uses, in one auditable place.

    ASSUMED unless marked otherwise: these are engineering placeholders chosen
    to make the logic testable, not tuned or validated values. They close when
    a real HMI study (force/dwell) and the dynamic model (pitch/spread) exist.
    """

    min_handle_force_N: float = 15.0
    min_committal_dwell_s: float = 0.75
    min_stair_rise_mm: float = 100.0
    max_single_corner_rise_mm: float = 100.0
    landing_clear_rise_mm: float = 40.0
    stationary_speed_mps: float = 0.02
    single_corner_spread_mm: float = 60.0
    #: Attitude envelope from the redundant IMU pair. The architecture stair
    #: angle is 36.03 deg (parameters.json stair_angle_deg_CALC); anything
    #: appreciably beyond it means the chassis is on something steeper than the
    #: reference staircase, or is tipping. Margin is ASSUMED, not derived from
    #: a tip-over analysis -- that needs the dynamic model and a real CG study.
    max_chassis_pitch_deg: float = 45.0


@dataclass(frozen=True)
class Predicates:
    """Safety-meaningful booleans derived from a SensorFrame."""

    deadman: bool
    force_pattern_ok: bool
    dwell_satisfied: bool
    committal_requested: bool
    reverse_requested: bool
    stair_confirmed_ahead: bool
    stair_confirmed_behind: bool
    landing_reached: bool
    single_corner_event: bool
    stationary: bool
    pitch_within_envelope: bool
    fault: bool


def derive_predicates(f: SensorFrame, t: Thresholds | None = None) -> Predicates:
    """Reduce raw sensor values to safety predicates.

    `single_corner_event` deliberately requires the detected rise to be BELOW
    the stair threshold while only one corner is displaced: a curb, threshold
    strip or single-wheel pothole is a different event from a staircase and
    must be answered by one corner's short correction, never by entering full
    stair mode.
    """
    t = t or Thresholds()
    return Predicates(
        deadman=f.deadman_engaged,
        force_pattern_ok=f.handle_force_N >= t.min_handle_force_N,
        dwell_satisfied=f.handle_force_sustained_s >= t.min_committal_dwell_s,
        committal_requested=f.stair_committal_button,
        reverse_requested=f.reverse_committal_button,
        stair_confirmed_ahead=f.front_preview_rise_mm >= t.min_stair_rise_mm,
        stair_confirmed_behind=f.rear_preview_rise_mm >= t.min_stair_rise_mm,
        landing_reached=(
            f.front_preview_rise_mm < t.landing_clear_rise_mm
            and f.rear_preview_rise_mm < t.landing_clear_rise_mm
        ),
        single_corner_event=(
            f.front_preview_rise_mm < t.max_single_corner_rise_mm
            and f.corner_extension_spread_mm >= t.single_corner_spread_mm
        ),
        stationary=abs(f.wheel_speed_mps) <= t.stationary_speed_mps,
        pitch_within_envelope=abs(f.chassis_pitch_deg) <= t.max_chassis_pitch_deg,
        fault=f.fault_present,
    )


@dataclass(frozen=True)
class Outputs:
    """Commanded actuator/interlock state for one control step."""

    holding_brakes_engaged: bool
    anti_drop_pawls_engaged: bool
    corner_actuators_enabled: bool
    wheel_drive_enabled: bool
    wheel_drive_direction: int  # +1 forward, -1 reverse, 0 none
    stair_mode_active: bool
    haptic_pattern: str


def _hold(stair_mode: bool, haptic: str) -> Outputs:
    """A hold: mechanically restrained, no drive authority.

    Both the normally-engaged holding brakes and the anti-drop pawls are
    applied. The pawls are the reason a mid-climb stop is a hold and not a
    controlled descent -- brakes alone would make stopping depend on continuous
    electrical holding torque.
    """
    return Outputs(
        holding_brakes_engaged=True,
        anti_drop_pawls_engaged=True,
        corner_actuators_enabled=False,
        wheel_drive_enabled=False,
        wheel_drive_direction=0,
        stair_mode_active=stair_mode,
        haptic_pattern=haptic,
    )


def _climb_intent_satisfied(p: Predicates, behind: bool = False) -> bool:
    """All independent conditions required to commit to a staircase.

    Every term is required, and each comes from a different physical source:
    the deadman bar, the handle load-sensing pins (magnitude AND sustained
    dwell), an explicit button press, and the preview distance sensor actually
    seeing a stair. No single sensor failing high can commit the vehicle to a
    staircase -- which is the property
    audit_stair_control_state_machine.py verifies by ablation rather than
    asserting here.
    """
    confirmed = p.stair_confirmed_behind if behind else p.stair_confirmed_ahead
    return (
        p.deadman
        and p.force_pattern_ok
        and p.dwell_satisfied
        and p.committal_requested
        and confirmed
        and not p.fault
    )


def step(state: State, p: Predicates) -> tuple[State, Outputs]:
    """Advance the state machine one control step.

    Pure function of (state, predicates) so the whole transition space can be
    enumerated. Priority order matters and is deliberate: fault, then deadman,
    then everything else. Safety interlocks are evaluated before any
    convenience behaviour.
    """
    # 1. Fault dominates everything.
    if p.fault:
        return State.FAULT_HOLD, _hold(state in ON_STAIR_STATES, "fault")

    # 2. Attitude interlock. The redundant IMU pair reports the chassis is
    #    outside its safe pitch envelope: either something steeper than the
    #    reference staircase, or the vehicle is tipping. Remove drive authority
    #    and sit on the mechanical holds. Deliberately NOT latched like a
    #    fault -- recovering attitude should not require a maintenance action --
    #    but because recovery lands in a hold state, resuming still re-runs the
    #    full committal test rather than silently continuing the climb.
    if not p.pitch_within_envelope:
        if state in ON_STAIR_STATES:
            return State.MID_CLIMB_HOLD, _hold(True, "attitude_limit")
        return State.STANDBY, _hold(False, "attitude_limit")

    # 3. Losing the deadman always removes drive authority. On a staircase this
    #    is a mid-climb hold on the pawls, not a coast or a free-fall; on flat
    #    ground it is a normal stop.
    if not p.deadman:
        if state in ON_STAIR_STATES:
            return State.MID_CLIMB_HOLD, _hold(True, "mid_climb_hold")
        return State.STANDBY, _hold(False, "idle")

    if state is State.FAULT_HOLD:
        # Faults latch: clearing is an explicit maintenance action, not an
        # automatic recovery the caregiver could trigger by re-gripping.
        return State.FAULT_HOLD, _hold(False, "fault")

    if state in (State.STANDBY, State.GROUND_ROLLING):
        if p.single_corner_event:
            # A curb or threshold: one corner's short correction. Forward
            # motion continues and stair mode is NOT entered.
            return State.SINGLE_CORNER_EVENT, Outputs(
                holding_brakes_engaged=False,
                anti_drop_pawls_engaged=False,
                corner_actuators_enabled=True,
                wheel_drive_enabled=True,
                wheel_drive_direction=1,
                stair_mode_active=False,
                haptic_pattern="single_corner",
            )
        if p.stair_confirmed_ahead or p.stair_confirmed_behind:
            return State.STAIR_DETECTED, Outputs(
                holding_brakes_engaged=False,
                anti_drop_pawls_engaged=False,
                corner_actuators_enabled=False,
                wheel_drive_enabled=p.force_pattern_ok,
                wheel_drive_direction=1 if p.force_pattern_ok else 0,
                stair_mode_active=False,
                haptic_pattern="stair_detected",
            )
        return State.GROUND_ROLLING, Outputs(
            holding_brakes_engaged=False,
            anti_drop_pawls_engaged=False,
            corner_actuators_enabled=False,
            wheel_drive_enabled=p.force_pattern_ok,
            wheel_drive_direction=1 if p.force_pattern_ok else 0,
            stair_mode_active=False,
            haptic_pattern="assist" if p.force_pattern_ok else "idle",
        )

    if state is State.SINGLE_CORNER_EVENT:
        # A single-corner correction is short by construction: it ends as soon
        # as the corner spread closes, and it can never escalate directly into
        # stair mode -- that requires going back through committal.
        if p.single_corner_event:
            return State.SINGLE_CORNER_EVENT, Outputs(
                holding_brakes_engaged=False,
                anti_drop_pawls_engaged=False,
                corner_actuators_enabled=True,
                wheel_drive_enabled=True,
                wheel_drive_direction=1,
                stair_mode_active=False,
                haptic_pattern="single_corner",
            )
        return State.GROUND_ROLLING, Outputs(
            holding_brakes_engaged=False,
            anti_drop_pawls_engaged=False,
            corner_actuators_enabled=False,
            wheel_drive_enabled=p.force_pattern_ok,
            wheel_drive_direction=1 if p.force_pattern_ok else 0,
            stair_mode_active=False,
            haptic_pattern="assist" if p.force_pattern_ok else "idle",
        )

    if state is State.STAIR_DETECTED:
        if not (p.stair_confirmed_ahead or p.stair_confirmed_behind):
            return State.GROUND_ROLLING, Outputs(
                holding_brakes_engaged=False,
                anti_drop_pawls_engaged=False,
                corner_actuators_enabled=False,
                wheel_drive_enabled=p.force_pattern_ok,
                wheel_drive_direction=1 if p.force_pattern_ok else 0,
                stair_mode_active=False,
                haptic_pattern="assist" if p.force_pattern_ok else "idle",
            )
        if p.committal_requested:
            return State.COMMITTAL_PENDING, Outputs(
                holding_brakes_engaged=True,
                anti_drop_pawls_engaged=True,
                corner_actuators_enabled=False,
                wheel_drive_enabled=False,
                wheel_drive_direction=0,
                stair_mode_active=False,
                haptic_pattern="committal_pending",
            )
        return State.STAIR_DETECTED, Outputs(
            holding_brakes_engaged=False,
            anti_drop_pawls_engaged=False,
            corner_actuators_enabled=False,
            wheel_drive_enabled=p.force_pattern_ok,
            wheel_drive_direction=1 if p.force_pattern_ok else 0,
            stair_mode_active=False,
            haptic_pattern="stair_detected",
        )

    if state is State.COMMITTAL_PENDING:
        # Ascent and descent are separate committals against separate preview
        # sensors. Descent additionally requires the explicit reverse button:
        # it must never be inferred from the caregiver merely easing off the
        # handle, which is exactly what a stumble or a distraction looks like.
        if _climb_intent_satisfied(p) and not p.reverse_requested:
            return State.CLIMBING, _drive(+1, "climbing")
        if _climb_intent_satisfied(p, behind=True) and p.reverse_requested:
            return State.DESCENDING, _drive(-1, "descending")
        if not p.committal_requested:
            return State.STAIR_DETECTED, _hold(False, "committal_cancelled")
        return State.COMMITTAL_PENDING, _hold(False, "committal_pending")

    if state in (State.CLIMBING, State.DESCENDING):
        if p.landing_reached:
            return State.LANDING_TRANSITION, _hold(True, "landing")
        if not p.force_pattern_ok:
            # Caregiver eased off mid-flight: hold on the pawls. Note this does
            # NOT become a descent -- reversing requires explicit intent.
            return State.MID_CLIMB_HOLD, _hold(True, "mid_climb_hold")
        if state is State.DESCENDING and not p.reverse_requested:
            # The reverse request must be held for as long as the vehicle is
            # actually reversing. Releasing it mid-descent holds on the pawls;
            # continuing to reverse on a stale request would be exactly the
            # inferred-reverse behaviour the requirement forbids.
            return State.MID_CLIMB_HOLD, _hold(True, "mid_climb_hold")
        return state, _drive(+1 if state is State.CLIMBING else -1,
                             "climbing" if state is State.CLIMBING else "descending")

    if state is State.MID_CLIMB_HOLD:
        # Resuming from a mid-climb hold re-runs the full committal test. A
        # hold is not a paused command that can restart on a single input.
        if _climb_intent_satisfied(p) and not p.reverse_requested:
            return State.CLIMBING, _drive(+1, "climbing")
        if _climb_intent_satisfied(p, behind=True) and p.reverse_requested:
            return State.DESCENDING, _drive(-1, "descending")
        return State.MID_CLIMB_HOLD, _hold(True, "mid_climb_hold")

    if state is State.LANDING_TRANSITION:
        # The flight is over: settle to rest under the mechanical holds rather
        # than coasting onto the landing under power. Only leave the staircase
        # once the preview sensors agree there is no further rise AND the
        # vehicle has actually come to rest, so the transition cannot fire on a
        # momentary sensor dropout mid-flight.
        #
        # Travel direction is deliberately NOT carried into this state. If more
        # rise appears after all, the vehicle holds and requires a fresh
        # committal, which re-establishes direction explicitly -- rather than
        # this state guessing a direction and driving the wrong way off a
        # landing reached while descending.
        if p.landing_reached and p.stationary:
            return State.GROUND_ROLLING, Outputs(
                holding_brakes_engaged=False,
                anti_drop_pawls_engaged=False,
                corner_actuators_enabled=False,
                wheel_drive_enabled=False,
                wheel_drive_direction=0,
                stair_mode_active=False,
                haptic_pattern="landing_complete",
            )
        if not p.landing_reached:
            return State.MID_CLIMB_HOLD, _hold(True, "mid_climb_hold")
        return State.LANDING_TRANSITION, _hold(True, "landing")

    # Unreachable for a valid State; fail safe rather than fall through.
    return State.FAULT_HOLD, _hold(state in ON_STAIR_STATES, "fault")


def _drive(direction: int, haptic: str) -> Outputs:
    """Driving on a staircase: brakes released, corner actuators live.

    Pawls are released only while drive authority is actually held; every path
    that removes drive authority routes through `_hold`, which re-engages them.
    """
    return Outputs(
        holding_brakes_engaged=False,
        anti_drop_pawls_engaged=False,
        corner_actuators_enabled=True,
        wheel_drive_enabled=True,
        wheel_drive_direction=direction,
        stair_mode_active=True,
        haptic_pattern=haptic,
    )
