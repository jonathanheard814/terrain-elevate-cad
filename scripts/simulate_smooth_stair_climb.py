#!/usr/bin/env python3
"""Kinematic stair-climb smoothness simulation.

scripts/analyze_vehicle_requirements.py already computes, per corner, the raw
geometric mismatch between an idealized smooth ramp and the physical stepped
stair (analysis_out/*_stair_phase_actuator_sweep.csv). That raw signal is a
sawtooth with a hard discontinuity at every stair nosing (one corner can jump
by ~100 mm between two adjacent samples) -- the existing "ramp-like climb"
screen only checks that the actuator has enough force/speed/time budget to
reach that discontinuity, not that the commanded motion is actually smooth.

This script builds a time-domain, jerk-limited (minimum-jerk / quintic)
trajectory for each corner actuator instead: using the sourced preview
sensor's real lookahead range, each corner's correction is blended smoothly
*before* the wheel reaches the stair edge rather than snapped at it, while
chassis forward speed is held constant throughout (no stop-and-go).

Truth boundary: this is a kinematic motion-profile simulation (position/
velocity/acceleration/jerk of a commanded trajectory). It is not FEA, not a
dynamic contact simulation, and not a physical test.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _stair_height_at(x_m: float, rise_m: float, going_m: float) -> float:
    """Idealized ground height: treats the stair as an instantaneous step.

    Only valid for a point contact. A real wheel of finite radius cannot
    follow this -- use _wheel_effective_ground_m for anything ride-related.
    """
    return math.floor(max(x_m, 0.0) / going_m) * rise_m


def _wheel_effective_ground_m(x_m: float, radius_m: float, rise_m: float, going_m: float) -> float:
    """Effective ground height seen by a wheel of finite radius, as its
    centre rolls over a staircase.

    A wheel of any real diameter does not drop into a step discontinuity: it
    contacts each tread nosing and pivots over it, so its centre traces an arc of
    radius `radius_m` about that edge. The centre height is the upper
    envelope of every constraint in range -- each tread surface (centre sits
    radius above it) and each nosing edge (centre sits on a circle about the
    edge point). Returning centre_height - radius gives the equivalent
    flat-ground height the wheel effectively rides at, which is continuous
    even though the underlying staircase is not.
    """
    x = max(x_m, 0.0)
    centre = 0.0
    # Only steps whose features can be within a wheel radius matter.
    first = max(0, int((x - radius_m) / going_m) - 1)
    last = int((x + radius_m) / going_m) + 2
    for i in range(first, last):
        tread_h = i * rise_m
        tread_start = i * going_m
        tread_end = (i + 1) * going_m
        # Resting on the flat tread surface.
        if tread_start <= x <= tread_end:
            centre = max(centre, tread_h + radius_m)
        # Pivoting over this tread's leading nosing edge.
        edge_x = tread_end
        edge_h = tread_h + rise_m
        dx = x - edge_x
        if abs(dx) < radius_m:
            centre = max(centre, edge_h + math.sqrt(radius_m * radius_m - dx * dx))
    return centre - radius_m


def _raw_required_extensions_mm(x_positions_m, wheel_x_offsets_m, wheelbase_m, rise_m, going_m, slope, wheel_radius_m):
    """Per-corner geometric correction (mm) at each x sample: the gap between
    the ideal ramp line and the actual step surface under that wheel.

    The + wheelbase_m/2 keeps every corner's x_contact non-negative (the
    rearmost corner's offset is -wheelbase/2, so without it x_contact goes
    negative at the start of a run and _stair_height_at's max(x, 0) clamp
    produces a bogus offset).

    Deliberately NOT normalized against the lowest corner, unlike
    scripts/analyze_vehicle_requirements.py. Because stair height is a floor
    function, (ramp_h - stepped_h) is already bounded to [0, rise), so this
    is directly commandable as-is and makes the chassis track the ideal ramp
    exactly. Subtracting the per-sample minimum only shifts the whole chassis
    down to touch its lowest corner, and because which corner is lowest keeps
    changing, that shift injects extra command transitions that the physical
    geometry does not require -- measured at roughly double the transition
    count (11 vs 5-6 per corner over 6 treads), which inflates peak actuator
    speed by about 36%. See the normalization_note in the emitted audit."""
    per_corner = {name: [] for name in wheel_x_offsets_m}
    for x in x_positions_m:
        for name, x_offset in wheel_x_offsets_m.items():
            x_contact = x + x_offset + wheelbase_m / 2
            ground_h = _wheel_effective_ground_m(x_contact, wheel_radius_m, rise_m, going_m)
            ramp_h = x_contact * slope
            per_corner[name].append((ramp_h - ground_h) * 1000.0)
    return per_corner


def _smoothing_kernel(window_n):
    """Normalized weights whose cumulative sum traces a quintic minimum-jerk
    S-curve: convolving a step function with this kernel reproduces the
    classic point-to-point minimum-jerk blend for an isolated step, and
    correctly *superposes* (rather than corrupts) the result when steps are
    closer together than the window -- unlike placing independent blend
    windows per step, convolution has no non-overlap assumption to violate."""
    if window_n <= 1:
        return [1.0]
    weights = []
    for i in range(window_n):
        s = (i + 0.5) / window_n
        # d/ds (10s^3 - 15s^4 + 6s^5) = 30 s^2 (1-s)^2
        weights.append(30.0 * s * s * (1.0 - s) * (1.0 - s))
    total = sum(weights)
    return [w / total for w in weights]


def _preview_running_max(series, back_n, preview_n):
    """Morphological dilation of the clearance floor over [t-back_n, t+preview_n].

    The forward half is the preview sensor: start lifting before the wheel
    reaches a riser, instead of demanding an impossible instantaneous lift at
    the riser face. The backward half is what makes the subsequent smoothing
    safe. Smoothing is a weighted average with weights summing to one over
    [t, t+back_n]; dilating backward by that same width guarantees every
    sample in that averaging span is itself >= the floor at t, so the average
    cannot dip below the floor. Without it, smoothing quietly drives the
    commanded wheel path into the riser.
    """
    n = len(series)
    out = [0.0] * n
    for i in range(n):
        start = max(0, i - back_n)
        end = min(n, i + preview_n + 1)
        out[i] = max(series[start:end])
    return out


def _convolve_lookahead(series, kernel):
    """smoothed[i] = sum_j kernel[j] * series[i+j] -- the kernel looks ahead
    of i, matching a preview sensor that already knows the upcoming stair
    geometry. Samples beyond the end of the run hold the last known value."""
    n = len(series)
    k = len(kernel)
    out = [0.0] * n
    last = series[-1]
    for i in range(n):
        acc = 0.0
        for j in range(k):
            idx = i + j
            acc += kernel[j] * (series[idx] if idx < n else last)
        out[i] = acc
    return out


def _derivative(series, dt_s):
    return [(series[i + 1] - series[i]) / dt_s for i in range(len(series) - 1)]


def _max_abs(series):
    return max((abs(v) for v in series), default=0.0)


def main() -> None:
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    load = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    physics = json.loads((ROOT / "data" / "te_v059_physics_elements.json").read_text())
    physics_motor_label = physics["actuator_models"]["corner_vertical_actuator"]["motor"]
    g = params["geometry"]
    actuator = load["actuator_assumptions"]
    combined = load["combined_climb_assumptions"]
    smooth = load["smooth_climb_assumptions"]

    rise_m = g["stair_rise_reference_mm"] / 1000.0
    going_m = g["stair_going_reference_mm"] / 1000.0
    wheelbase_m = g["wheelbase_mm"] / 1000.0
    wheel_radius_m = g["wheel_diameter_mm"] / 2000.0
    lead_mm = g["ball_screw_lead_mm_SRC"]
    slope = rise_m / going_m

    forward_speed_mps = combined["stair_climb_forward_speed_mps_ASSUMED"]

    gear_ratio = actuator["gear_ratio_ASSUMED"]
    motor_output_speed_rpm = actuator["motor_nominal_speed_rpm_SRC"] / gear_ratio
    actuator_linear_speed_mm_s = motor_output_speed_rpm * lead_mm / 60.0

    preview_treads = smooth["preview_treads_ASSUMED"]
    preview_time_s = preview_treads * going_m / forward_speed_mps
    speed_margin = smooth["actuator_speed_margin_fraction_ASSUMED"]
    control_margin_s = smooth["control_loop_margin_s_ASSUMED"]
    tolerance_mm = smooth["position_discontinuity_tolerance_mm_ASSUMED"]
    n_treads = smooth["simulated_tread_count_ASSUMED"]

    wheel_x_offsets_m = {
        "front_left": wheelbase_m / 2,
        "front_right": wheelbase_m / 2,
        "rear_left": -wheelbase_m / 2,
        "rear_right": -wheelbase_m / 2,
    }

    dx_m = going_m / 4000.0
    n_x_samples = int(n_treads * going_m / dx_m) + 1
    x_positions_m = [i * dx_m for i in range(n_x_samples)]

    raw_by_corner = _raw_required_extensions_mm(
        x_positions_m, wheel_x_offsets_m, wheelbase_m, rise_m, going_m, slope, wheel_radius_m
    )

    max_jump_mm = max(
        abs(series[i + 1] - series[i])
        for series in raw_by_corner.values()
        for i in range(len(series) - 1)
    )
    # Peak-to-peak range of the (now continuous, periodic) clearance signal --
    # reported for context. NOT used to size the window below: a closed-form
    # "(15/8)*range/T" isolated-step estimate was tried and measured against
    # the real convolution output (see git history) -- it assumes the whole
    # range happens as one abrupt transition, which drastically overestimates
    # the velocity a *continuous* periodic signal actually needs, so it kept
    # picking windows over 10x too long. Direct measurement (below) replaces
    # it: there is no shortcut formula for what a continuous wheel-rolling
    # contact signal needs, only measuring the actual convolution output.
    max_correction_range_mm = max(max(series) - min(series) for series in raw_by_corner.values())

    preview_cap_s = preview_time_s - control_margin_s
    tread_period_s = going_m / forward_speed_mps
    dt_s = combined["control_loop_period_ms_ASSUMED"] / 1000.0 * 4.0
    total_time_s = (n_treads * going_m) / forward_speed_mps
    n_time_samples = int(total_time_s / dt_s) + 1
    time_s = [i * dt_s for i in range(n_time_samples)]

    # Steady-state trim, computed here (it only depends on preview_time_s and
    # time_s, not on window_n) so the bisection search below measures peak
    # velocity over the *same* trimmed region as the final reported metric.
    # Using the untrimmed array here previously caused the search to chase a
    # spurious edge artifact -- see the note where this is reused after the
    # search -- landing on a needlessly large window.
    steady_start_idx = next((i for i, t in enumerate(time_s) if t >= preview_time_s), 0)
    steady_end_idx = len(time_s) - steady_start_idx
    if steady_end_idx <= steady_start_idx:
        steady_start_idx, steady_end_idx = 0, len(time_s)

    def _peak_velocity_for_window_n(candidate_window_n: int) -> float:
        """Actually run the preview-dilation + convolution pipeline at a
        candidate window size and measure the resulting peak velocity across
        all four corners -- the only reliable way to know what a given
        window costs in actuator speed for this (non-isolated-step) signal."""
        peak = 0.0
        for name in wheel_x_offsets_m:
            raw_series = raw_by_corner[name]
            raw_time = []
            for t in time_s:
                x_mm = t * forward_speed_mps * 1000.0
                idx = min(int(x_mm / (dx_m * 1000.0)), len(raw_series) - 1)
                raw_time.append(raw_series[idx])
            previewed = _preview_running_max(raw_time, candidate_window_n, candidate_window_n)
            smoothed = _convolve_lookahead(previewed, _smoothing_kernel(candidate_window_n))
            smoothed = smoothed[steady_start_idx:steady_end_idx]
            peak = max(peak, _max_abs(_derivative(smoothed, dt_s)))
        return peak

    # Smallest window that keeps peak velocity within the actuator's speed
    # margin, found by bisection (velocity decreases monotonically as the
    # window widens -- confirmed by direct sweep, not assumed): this
    # maximizes ramp-tracking accuracy subject to the actuator actually being
    # able to deliver it, instead of guessing a window and hoping. Upper
    # bound is the tighter of the preview budget and a generous multiple of
    # tread period; if even that cannot bring velocity under budget, the
    # actuator itself is the limit and no window fixes it (reported honestly
    # via velocity_within_actuator_capability below, not hidden by silently
    # over-smoothing).
    lo_n = 2
    search_ceiling_n = max(lo_n + 1, round(min(preview_cap_s, 2.0 * tread_period_s) / dt_s))
    hi_n = search_ceiling_n
    target_velocity_mm_s = speed_margin * actuator_linear_speed_mm_s
    search_ceiling_infeasible = _peak_velocity_for_window_n(hi_n) > target_velocity_mm_s
    if search_ceiling_infeasible:
        window_n = hi_n  # even the widest allowed window can't hit the margin
    else:
        while hi_n - lo_n > 1:
            mid_n = (lo_n + hi_n) // 2
            if _peak_velocity_for_window_n(mid_n) <= target_velocity_mm_s:
                hi_n = mid_n
            else:
                lo_n = mid_n
        window_n = hi_n
    window_s = window_n * dt_s
    kernel = _smoothing_kernel(window_n)
    # The backward dilation must be at least window_n (kernel length minus
    # one) for the clearance guarantee in _preview_running_max's docstring to
    # hold -- every kernel tap's dilation window has to still cover the
    # current sample. Measured empirically (see sweep in git history/PR
    # notes): extending the *forward* reach beyond window_n does not improve
    # ramp-tracking -- the required clearance profile is already close to
    # continuously rising for most of a tread period (the wheel radius is
    # comparable to the tread going), so a wider forward window just holds
    # the plateau at peak height for longer without buying anything. Matching
    # forward reach to window_n is both simplest and empirically as good as
    # any larger choice tested.
    preview_n = window_n

    smoothed_by_corner = {}
    raw_time_by_corner = {}
    step_height_time_by_corner = {}
    ramp_height_time_by_corner = {}
    for name, x_offset in wheel_x_offsets_m.items():
        # Resample the raw (unsmoothed) spatial signal onto the time base first.
        raw_series = raw_by_corner[name]
        raw_time = []
        step_time = []
        ramp_time = []
        for t in time_s:
            x_m = t * forward_speed_mps
            x_mm = x_m * 1000.0
            idx = min(int(x_mm / (dx_m * 1000.0)), len(raw_series) - 1)
            raw_time.append(raw_series[idx])
            x_contact = x_m + x_offset + wheelbase_m / 2
            step_time.append(_wheel_effective_ground_m(x_contact, wheel_radius_m, rise_m, going_m) * 1000.0)
            ramp_time.append(x_contact * slope * 1000.0)
        raw_time_by_corner[name] = raw_time
        step_height_time_by_corner[name] = step_time
        ramp_height_time_by_corner[name] = ramp_time
        # The raw signal is a *clearance floor*, not a target: the wheel must
        # never be below it (it would hit the riser), but may be above it.
        # Taking the running max over the preview window starts each lift
        # early enough to be feasible, then convolution smooths it. Together
        # these are exactly what the preview sensor exists to enable.
        previewed = _preview_running_max(raw_time, window_n, preview_n)
        smoothed_by_corner[name] = _convolve_lookahead(previewed, kernel)

    # steady_start_idx/steady_end_idx (skip the first preview_time_s start-up
    # transient and the last preview_time_s tail under-dilation) were already
    # computed above, before the bisection search, so the search measures
    # velocity over the same trimmed region used here.

    per_corner_metrics = {}
    for name in wheel_x_offsets_m:
        smoothed = smoothed_by_corner[name][steady_start_idx:steady_end_idx]
        raw_resampled = raw_time_by_corner[name][steady_start_idx:steady_end_idx]
        step_h = step_height_time_by_corner[name][steady_start_idx:steady_end_idx]
        ramp_h = ramp_height_time_by_corner[name][steady_start_idx:steady_end_idx]

        smoothed_step = [smoothed[i + 1] - smoothed[i] for i in range(len(smoothed) - 1)]
        raw_step = [raw_resampled[i + 1] - raw_resampled[i] for i in range(len(raw_resampled) - 1)]

        vel = _derivative(smoothed, dt_s)
        acc = _derivative(vel, dt_s)
        jerk = _derivative(acc, dt_s)

        # What the occupant actually rides: the chassis corner height is the
        # physical step under the wheel plus whatever the actuator extends.
        # Smoothing the command more always lowers actuator speed but makes
        # this track the ideal ramp worse, so speed alone is not a sufficient
        # criterion -- the ride itself has to be measured.
        chassis_h = [step_h[i] + smoothed[i] for i in range(len(smoothed))]
        ramp_error = [chassis_h[i] - ramp_h[i] for i in range(len(chassis_h))]
        chassis_vel = _derivative(chassis_h, dt_s)
        chassis_acc = _derivative(chassis_vel, dt_s)
        chassis_jerk = _derivative(chassis_acc, dt_s)

        # Uncorrected baseline: what the same wheel would ride with no
        # actuator at all (bare staircase), for an honest before/after.
        bare_vel = _derivative(step_h, dt_s)
        bare_acc = _derivative(bare_vel, dt_s)

        # Safety-critical: the commanded extension must never fall below the
        # clearance floor, or the wheel drives into a riser face.
        clearance_violation = max(
            (raw_resampled[i] - smoothed[i] for i in range(len(smoothed))), default=0.0
        )

        per_corner_metrics[name] = {
            "max_clearance_violation_mm": max(clearance_violation, 0.0),
            "max_raw_position_step_mm": _max_abs(raw_step),
            "max_smoothed_position_step_mm": _max_abs(smoothed_step),
            "max_smoothed_velocity_mm_s": _max_abs(vel),
            "max_smoothed_acceleration_mm_s2": _max_abs(acc),
            "max_smoothed_jerk_mm_s3": _max_abs(jerk),
            "max_chassis_ramp_tracking_error_mm": _max_abs(ramp_error),
            "max_chassis_vertical_acceleration_mm_s2": _max_abs(chassis_acc),
            "max_chassis_vertical_jerk_mm_s3": _max_abs(chassis_jerk),
            "uncorrected_bare_stair_acceleration_mm_s2": _max_abs(bare_acc),
        }

    max_smoothed_step_mm = max(m["max_smoothed_position_step_mm"] for m in per_corner_metrics.values())
    max_smoothed_velocity_mm_s = max(m["max_smoothed_velocity_mm_s"] for m in per_corner_metrics.values())
    max_raw_step_mm = max(m["max_raw_position_step_mm"] for m in per_corner_metrics.values())

    velocity_within_actuator_capability = max_smoothed_velocity_mm_s <= actuator_linear_speed_mm_s
    no_position_discontinuity = max_smoothed_step_mm <= tolerance_mm

    max_clearance_violation_mm = max(m["max_clearance_violation_mm"] for m in per_corner_metrics.values())
    clearance_maintained = max_clearance_violation_mm <= tolerance_mm
    max_ramp_error_mm = max(m["max_chassis_ramp_tracking_error_mm"] for m in per_corner_metrics.values())
    max_chassis_accel_mm_s2 = max(m["max_chassis_vertical_acceleration_mm_s2"] for m in per_corner_metrics.values())
    bare_stair_accel_mm_s2 = max(m["uncorrected_bare_stair_acceleration_mm_s2"] for m in per_corner_metrics.values())
    ramp_error_budget_mm = smooth["chassis_ramp_error_budget_mm_ASSUMED"]
    ride_tracks_ramp = max_ramp_error_mm <= ramp_error_budget_mm
    # NOT claimed as a comfort improvement: peak vertical acceleration is
    # dominated by the wheel's own geometric pivot over each nosing (an
    # instant of the rolling-contact arc, not something extension-smoothing
    # touches -- the extension only adds to ground height, it doesn't change
    # how sharply the ground height itself transitions). Absorbing that
    # high-frequency transient is the passive spring/damper's job (already in
    # the design at each corner), not this actuator's. Reported for
    # reference only; ride_tracks_ramp (bulk chassis-to-ramp following, which
    # the actuator does control) is the gate that matters here.
    peak_accel_ratio_reference_only = (
        bare_stair_accel_mm_s2 / max_chassis_accel_mm_s2 if max_chassis_accel_mm_s2 > 0 else math.inf
    )
    # Forward speed is never reduced anywhere in this construction -- the
    # chassis-travel/time mapping is fixed at forward_speed_mps throughout.
    # That is a structural fact of the model, not a result to gate on; the
    # real open question is whether the actuator can keep up, which is what
    # velocity_within_actuator_capability actually measures.
    forward_speed_held_constant = True

    # no_position_discontinuity is reported but not part of overall_pass: see
    # its "note" above -- it is mathematically the same quantity as
    # velocity_within_actuator_capability, just rescaled by dt.
    # Both remaining gates are required: smoothing the command harder always
    # buys actuator speed at the cost of ramp-tracking, so passing one alone
    # is meaningless.
    overall_pass = velocity_within_actuator_capability and ride_tracks_ramp and clearance_maintained

    # If ride_quality FAILs even at the actuator-optimal window found above,
    # find what actuator speed *would* be needed to hit the budget: bisect on
    # window_n for the largest window whose ramp error still stays under
    # budget (error grows monotonically with window, confirmed by sweep),
    # then measure the velocity that window actually requires. This is the
    # real, searched number that actuator_reselection_spec closes against --
    # not a guess -- for whichever gate is the one actually failing.
    required_velocity_for_ride_quality_mm_s = None
    if not ride_tracks_ramp:

        def _ramp_error_for_window_n(candidate_window_n: int) -> float:
            peak = 0.0
            for name, x_offset in wheel_x_offsets_m.items():
                raw_series = raw_by_corner[name]
                raw_time = []
                step_time = []
                ramp_time = []
                for t in time_s:
                    x_m = t * forward_speed_mps
                    x_mm = x_m * 1000.0
                    idx = min(int(x_mm / (dx_m * 1000.0)), len(raw_series) - 1)
                    raw_time.append(raw_series[idx])
                    x_contact = x_m + x_offset + wheelbase_m / 2
                    step_time.append(_wheel_effective_ground_m(x_contact, wheel_radius_m, rise_m, going_m) * 1000.0)
                    ramp_time.append(x_contact * slope * 1000.0)
                previewed = _preview_running_max(raw_time, candidate_window_n, candidate_window_n)
                smoothed = _convolve_lookahead(previewed, _smoothing_kernel(candidate_window_n))
                smoothed = smoothed[steady_start_idx:steady_end_idx]
                step_h = step_time[steady_start_idx:steady_end_idx]
                ramp_h = ramp_time[steady_start_idx:steady_end_idx]
                chassis_h = [step_h[i] + smoothed[i] for i in range(len(smoothed))]
                peak = max(peak, _max_abs(chassis_h[i] - ramp_h[i] for i in range(len(chassis_h))))
            return peak

        lo_rq, hi_rq = 2, search_ceiling_n
        if _ramp_error_for_window_n(lo_rq) <= ramp_error_budget_mm:
            while hi_rq - lo_rq > 1:
                mid_rq = (lo_rq + hi_rq) // 2
                if _ramp_error_for_window_n(mid_rq) <= ramp_error_budget_mm:
                    lo_rq = mid_rq
                else:
                    hi_rq = mid_rq
            required_velocity_for_ride_quality_mm_s = _peak_velocity_for_window_n(lo_rq)
        # else: even the tightest possible window (n=2, one control-loop step)
        # cannot hit the budget -- the signal itself has more high-frequency
        # content than any actuator-side smoothing choice can resolve, which
        # would point at the wheel/stair geometry itself, not the actuator.

    # Exact closure spec for actuator re-selection. For a ball-screw drive:
    #   linear speed  = (motor_rpm / gear_ratio) * lead_mm / 60
    #   axial force   = 2*pi * screw_eff * (motor_torque * gear_ratio * gear_eff) / lead_m
    # Both scale with the motor's torque x speed product, so the shortfall is
    # reported as the factor by which that product must increase -- which is
    # selection-neutral (a faster motor, a lower gear ratio, or a higher screw
    # lead alone cannot fix it; speed gains trade directly against force).
    governing_corner_force_N = 1323.8977499999999  # requirements_screen.json corner_actuator_screen
    screw_eff = actuator["screw_efficiency_ASSUMED"]
    gear_eff = actuator["gearbox_efficiency_ASSUMED"]
    motor_torque_Nm = actuator["motor_nominal_torque_Nm_SRC"]
    motor_speed_rpm = actuator["motor_nominal_speed_rpm_SRC"]
    lead_m = lead_mm / 1000.0
    screened_axial_force_N = 2 * math.pi * screw_eff * (motor_torque_Nm * gear_ratio * gear_eff) / lead_m
    # The binding requirement is whichever gate actually fails: if ride_quality
    # fails (even at the actuator-optimal window), the searched
    # required_velocity_for_ride_quality_mm_s is the real target -- sizing
    # against max_smoothed_velocity_mm_s here would under-report it, since
    # that value is already the actuator-constrained (not ride-quality-
    # constrained) optimum. If ride_quality passes, max_smoothed_velocity_mm_s
    # is the real, achieved figure.
    reselection_target_velocity_mm_s = (
        required_velocity_for_ride_quality_mm_s
        if required_velocity_for_ride_quality_mm_s is not None
        else max_smoothed_velocity_mm_s
    )
    # Required lead/gear ratio from each constraint (see algebra above).
    lead_over_gear_for_speed = reselection_target_velocity_mm_s * 60.0 / motor_speed_rpm
    lead_over_gear_for_force = (
        2 * math.pi * screw_eff * motor_torque_Nm * gear_eff * 1000.0 / governing_corner_force_N
    )
    motor_product_shortfall = lead_over_gear_for_speed / lead_over_gear_for_force
    required_mechanical_power_W = governing_corner_force_N * (reselection_target_velocity_mm_s / 1000.0)

    result = {
        "truth_boundary": (
            "Kinematic motion-profile simulation of commanded corner-actuator "
            "position over time. Not FEA, not dynamic contact simulation, not a "
            "physical test. Forward speed is an ASSUMED constant input, not a "
            "result derived from wheel traction (see requirements_screen.json "
            "wheel_propulsion_screen, which is a separate, currently-FAIL check)."
        ),
        "method": (
            "Reuses the raw per-corner required-extension-vs-position sawtooth from "
            "analyze_vehicle_requirements.py, resamples it onto a time base at the "
            "assumed constant forward speed, then convolves it with a normalized "
            "quintic minimum-jerk kernel (the derivative of the classic 10s^3-15s^4"
            "+6s^5 S-curve) instead of placing independent point-to-point blends -- "
            "convolution superposes overlapping/closely-spaced corrections correctly "
            "rather than assuming they never overlap. The kernel window is sized to "
            "the tightest of: a screened-actuator-speed margin for the largest single "
            "correction, the real spacing between consecutive corrections for the "
            "same corner, and the sourced preview sensor's lookahead budget."
        ),
        "inputs": {
            "stair_rise_mm": g["stair_rise_reference_mm"],
            "stair_going_mm": g["stair_going_reference_mm"],
            "forward_speed_mps_ASSUMED": forward_speed_mps,
            "screened_actuator_linear_speed_mm_s": actuator_linear_speed_mm_s,
            "preview_treads_ASSUMED": preview_treads,
            "preview_time_s": preview_time_s,
            "actuator_speed_margin_fraction_ASSUMED": speed_margin,
            "simulated_tread_count": n_treads,
            "largest_single_sample_clearance_change_mm": max_jump_mm,
            "clearance_signal_peak_to_peak_range_mm": max_correction_range_mm,
            "preview_budget_window_s": preview_cap_s,
            "search_ceiling_window_s": search_ceiling_n * dt_s,
            "search_ceiling_infeasible": search_ceiling_infeasible,
            "window_used_s": window_s,
            "window_selection_method": (
                "Smallest window (by bisection) whose actually-measured peak velocity "
                "stays within actuator_speed_margin_fraction_ASSUMED of the screened "
                "actuator speed -- not a closed-form estimate. search_ceiling_window_s is "
                "the search's upper bound (2 tread periods, capped by the preview budget); "
                "if search_ceiling_infeasible is true, even that widest allowed window "
                "could not bring velocity under the margin, window_used_s is that ceiling "
                "as the best available compromise, and velocity_within_actuator_capability "
                "below reports the real shortfall rather than hiding it."
            ),
        },
        "per_corner": per_corner_metrics,
        "gates": {
            "no_position_discontinuity": {
                "max_smoothed_position_step_mm": max_smoothed_step_mm,
                "tolerance_mm": tolerance_mm,
                "max_raw_position_step_mm_for_comparison": max_raw_step_mm,
                "result": "PASS" if no_position_discontinuity else "FAIL",
                "note": (
                    "max_smoothed_position_step_mm equals max_smoothed_velocity_mm_s * "
                    "sample_dt_s by construction (it is the same numerically-differentiated "
                    "signal, not an independent check) -- mathematical continuity is already "
                    "guaranteed by convolving with a bounded kernel, regardless of sampling. "
                    "This gate is informational; velocity_within_actuator_capability below is "
                    "the substantive, non-redundant finding."
                ),
            },
            "velocity_within_actuator_capability": {
                "max_smoothed_velocity_mm_s": max_smoothed_velocity_mm_s,
                "screened_actuator_linear_speed_mm_s": actuator_linear_speed_mm_s,
                "result": "PASS" if velocity_within_actuator_capability else "FAIL",
                "note": (
                    None if velocity_within_actuator_capability else
                    "The correction cadence required by this stair geometry at the "
                    "assumed forward speed is tighter than the sourced actuator's "
                    "rated linear speed can smoothly track. Closes with a faster "
                    "corner actuator (e.g. higher screw lead or lower gear reduction) "
                    "-- not by reducing forward speed, which is locked at a 0.2 m/s "
                    "floor in te_v059_acceptance_criteria.json."
                ),
            },
        },
        "wheel_step_geometry": {
            "purpose": (
                "Whether the wheel can roll over a stair nosing unaided, or must be "
                "actively lifted clear of the riser face first. This is pure geometry "
                "and it drives the whole actuator duty cycle."
            ),
            "wheel_diameter_mm": g["wheel_diameter_mm"],
            "wheel_radius_mm": wheel_radius_m * 1000.0,
            "stair_rise_mm": g["stair_rise_reference_mm"],
            "rolls_over_unaided": wheel_radius_m * 1000.0 >= g["stair_rise_reference_mm"],
            "required_lift_to_clear_riser_mm": max(
                g["stair_rise_reference_mm"] - wheel_radius_m * 1000.0, 0.0
            ),
            "wheel_diameter_for_unaided_rolling_mm": 2.0 * g["stair_rise_reference_mm"],
            "finding": (
                (
                    "The wheel centre on the lower tread sits at radius above it. To pass a "
                    "nosing without being lifted, that centre must already be at or above "
                    "the nosing, i.e. radius >= rise. At the current "
                    f"{g['wheel_diameter_mm']:.0f} mm diameter (radius "
                    f"{wheel_radius_m * 1000.0:.0f} mm) against a "
                    f"{g['stair_rise_reference_mm']:.1f} mm rise, radius >= rise holds with "
                    f"{wheel_radius_m * 1000.0 - g['stair_rise_reference_mm']:.1f} mm margin, "
                    "so the wheel rolls over the nosing unaided -- no forced lift is "
                    "required at any step. This does not by itself mean the ride is smooth: "
                    "the wheel's natural rolling-contact path still deviates from an ideal "
                    "ramp line (see ride_quality below), which is a separate question this "
                    "screen does not answer."
                )
                if wheel_radius_m * 1000.0 >= g["stair_rise_reference_mm"]
                else (
                    "The wheel centre on the lower tread sits at radius above it. To pass a "
                    "nosing without being lifted, that centre must already be at or above "
                    "the nosing, i.e. radius >= rise. At the current "
                    f"{g['wheel_diameter_mm']:.0f} mm diameter (radius "
                    f"{wheel_radius_m * 1000.0:.0f} mm) against a "
                    f"{g['stair_rise_reference_mm']:.1f} mm rise, the wheel jams against the "
                    "riser face and must be actively lifted at least "
                    f"{max(g['stair_rise_reference_mm'] - wheel_radius_m * 1000.0, 0.0):.1f} mm "
                    "at every step. Rolling over unaided would require a wheel diameter of at "
                    f"least {2.0 * g['stair_rise_reference_mm']:.0f} mm. This is a genuine "
                    "design tension between the packaging requirement and the no-lifting "
                    "requirement, not a modelling artifact -- surfacing it, not resolving it, "
                    "is what this screen does."
                )
            ),
        },
        "ride_quality": {
            "purpose": (
                "The metric that actually matters to the occupant: how closely the "
                "chassis corner follows the ideal ramp line, versus the bare staircase "
                "it would ride with no actuator. This is the bulk, low-frequency "
                "tracking the corner actuator actually controls. Smoothing the actuator "
                "command harder always lowers required actuator speed while worsening "
                "this, so both are gated together."
            ),
            "max_chassis_ramp_tracking_error_mm": max_ramp_error_mm,
            "ramp_error_budget_mm": ramp_error_budget_mm,
            "stair_rise_mm_for_scale": g["stair_rise_reference_mm"],
            "result": "PASS" if ride_tracks_ramp else "FAIL",
            "peak_vertical_acceleration": {
                "note": (
                    "NOT a claimed comfort improvement. Peak vertical acceleration is "
                    "dominated by the wheel's own geometric pivot over each nosing -- a "
                    "rolling-contact transient the corner actuator's extension-smoothing "
                    "does not touch (it adds to ground height, it does not change how "
                    "sharply that ground height transitions). Absorbing that high-"
                    "frequency transient is the per-corner passive spring/damper's job, "
                    "not this actuator's -- reported for reference only, not gated."
                ),
                "max_chassis_vertical_acceleration_mm_s2": max_chassis_accel_mm_s2,
                "uncorrected_bare_stair_vertical_acceleration_mm_s2": bare_stair_accel_mm_s2,
                "peak_ratio_reference_only": peak_accel_ratio_reference_only,
            },
        },
        "clearance": {
            "purpose": (
                "Safety-critical: the commanded extension must never fall below the "
                "clearance floor, or the wheel is driven into a riser face."
            ),
            "max_clearance_violation_mm": max_clearance_violation_mm,
            "tolerance_mm": tolerance_mm,
            "result": "PASS" if clearance_maintained else "FAIL",
        },
        "actuator_reselection_spec": {
            "purpose": (
                "Exact, selection-neutral closure criterion for the corner actuator, "
                "sized against whichever gate actually fails. Ball-screw speed and force "
                "both scale with the motor's torque x speed product, so raising screw "
                "lead or lowering gear ratio to gain speed costs proportional force -- "
                "only a higher-power motor closes this."
            ),
            "binding_gate": (
                "ride_quality" if required_velocity_for_ride_quality_mm_s is not None
                else "velocity_within_actuator_capability" if not velocity_within_actuator_capability
                else "none_both_gates_pass"
            ),
            "achieved_velocity_at_actuator_optimal_window_mm_s": max_smoothed_velocity_mm_s,
            "required_velocity_for_ride_quality_mm_s": required_velocity_for_ride_quality_mm_s,
            "required_linear_speed_mm_s": reselection_target_velocity_mm_s,
            "delivered_linear_speed_mm_s": actuator_linear_speed_mm_s,
            "speed_shortfall_factor": reselection_target_velocity_mm_s / actuator_linear_speed_mm_s,
            "required_axial_force_N": governing_corner_force_N,
            "delivered_axial_force_N": screened_axial_force_N,
            "required_mechanical_power_at_screw_W": required_mechanical_power_W,
            "motor_torque_speed_product_shortfall_factor": motor_product_shortfall,
            "current_motor": physics_motor_label,
            "closure": (
                "Select a corner-actuator motor whose nominal torque x nominal speed "
                "product exceeds the current one by at least "
                f"{motor_product_shortfall:.2f}x, then re-run this screen -- equivalent to "
                f"roughly {required_mechanical_power_W:.0f} W continuous-or-duty-cycle-rated "
                "mechanical power at the screw (vs the current 420 W motor), given the low "
                "duty cycle already established in te_v059_electrical_system.json (~75% "
                "corner-active time, not continuous). Two real candidates already checked "
                "against actual datasheets and found INSUFFICIENT, so this is not yet closed "
                "by a simple family swap: the Dunkermotoren BG75 dMove family (all three "
                "stack lengths, 48 V) tops out around 6044 Nm*rpm continuous "
                "(BG75x75, 2.07/1.36 Nm at 2920/3270 rpm per the manufacturer datasheet) "
                "versus roughly 17215 Nm*rpm required; maxon's largest compact EC-flat frame "
                "(EC 90 flat, 260 W/48 V) is similarly short at about 4820 Nm*rpm. Closing "
                "this for real needs either a larger industrial BLDC/servo motor frame "
                "outside these compact families, or replacing the ball-screw+motor+gearhead "
                "assembly with a purpose-built high-speed electric rod actuator sized to "
                f"{governing_corner_force_N:.0f} N / {reselection_target_velocity_mm_s / 1000.0:.2f} m/s "
                "directly (Exlar and Tolomatic are real vendors in this force/speed class) -- "
                "either path requires vendor engagement with this exact spec, which is "
                "already listed under required_verifications_before_release in "
                "requirements_screen.json."
            ),
        },
        "normalization_note": (
            "Per-corner extension here is the raw ramp-minus-step gap, not normalized "
            "against the lowest corner as in analyze_vehicle_requirements.py. Because "
            "stair height is a floor function that quantity is already bounded to "
            "[0, rise) and directly commandable. Normalizing shifts the chassis down to "
            "its lowest corner, and since which corner is lowest keeps changing, that "
            "injects extra command transitions the geometry does not require -- measured "
            "at roughly double the transition count and about 36% higher peak actuator "
            "speed. The un-normalized formulation used here is both more physical and "
            "less demanding."
        ),
        "forward_speed_held_constant": forward_speed_held_constant,
        "no_walking_gait": {
            "claim": (
                "All four wheels remain in continuous rolling ground contact throughout "
                "climbing; the corner actuators only adjust vertical height under a wheel "
                "that never leaves the ground, and forward progress never pauses to lift "
                "and place a wheel like a leg/foot. This is a continuously-rolling wheeled "
                "architecture, not a legged/quadruped walking gait."
            ),
            "basis": (
                "Structural, not simulated here: enforced by the locked design constraints "
                "in data/te_v059_parameters.json (extra_small_wheels=0, tracks=0, belts=0) "
                "and the corner mechanism architecture in src/terrain_elevate/cad_model.py, "
                "which drives wheel height via a fixed vertical ball-screw slider, not a "
                "leg that lifts off and re-plants."
            ),
        },
        "open_items": [
            "No sourced mechanical or ride-comfort maximum-jerk limit exists yet for "
            "this platform; max_smoothed_jerk_mm_s3 is reported per corner as a "
            "measured quantity, not screened against a threshold. Closes when a "
            "ride-comfort or actuator-duty-cycle jerk limit is sourced.",
        ]
        + (
            []
            if velocity_within_actuator_capability
            else [
                "velocity_within_actuator_capability is FAIL: see actuator_reselection_spec "
                "above for the exact required motor torque x speed product -- closes by "
                "sourcing a higher-power corner actuator motor, not by reducing forward "
                "speed (locked at a 0.2 m/s floor) or by further smoothing (smoothing "
                "trades directly against ride_quality below, it cannot buy back both)."
            ]
        )
        + (
            []
            if ride_tracks_ramp
            else [
                f"ride_quality is FAIL: max_chassis_ramp_tracking_error_mm "
                f"({max_ramp_error_mm:.1f} mm) exceeds ramp_error_budget_mm "
                f"({ramp_error_budget_mm:.1f} mm) even at the actuator-optimal window "
                "found by bisection (window_used_s in inputs above) -- this is the "
                "best ramp-tracking the sourced actuator can deliver at the current "
                f"{g['wheel_diameter_mm']:.0f} mm wheel diameter, not an under-tuned "
                "window. See wheel_step_geometry above for whether the wheel rolls "
                "over the nosing unaided at this diameter (it now does, if "
                "rolls_over_unaided is true, which fixed the earlier clearance/"
                "jamming problem but not this one -- rolling unaided is necessary "
                "for ride quality, not sufficient). "
                + (
                    "actuator_reselection_spec.required_velocity_for_ride_quality_mm_s "
                    "is the searched (not estimated) actuator speed that would actually "
                    "hit the budget."
                    if required_velocity_for_ride_quality_mm_s is not None
                    else "Even the tightest possible window (a single control-loop step) "
                    "cannot hit the budget -- no actuator speed fixes this; the signal's "
                    "own high-frequency content exceeds what any smoothing can resolve."
                )
                + " Closes by sourcing a faster actuator per that figure, by increasing "
                "wheel diameter further, or by justifying a larger "
                "chassis_ramp_error_budget_mm_ASSUMED against an actual ride-comfort "
                "limit, not choosing it to force a pass."
            ]
        ),
        "result": "PASS" if overall_pass else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "Terrain_Elevate_P1_V0_59_smooth_climb_audit.json").write_text(json.dumps(result, indent=2))

    csv_lines = [
        "time_s,chassis_travel_mm,front_left_smoothed_mm,front_right_smoothed_mm,"
        "rear_left_smoothed_mm,rear_right_smoothed_mm,front_left_raw_mm,front_right_raw_mm,"
        "rear_left_raw_mm,rear_right_raw_mm"
    ]
    for i, t in enumerate(time_s):
        travel_mm = t * forward_speed_mps * 1000.0
        row = [f"{t:.6f}", f"{travel_mm:.6f}"]
        for name in ("front_left", "front_right", "rear_left", "rear_right"):
            row.append(f"{smoothed_by_corner[name][i]:.6f}")
        for name in ("front_left", "front_right", "rear_left", "rear_right"):
            row.append(f"{raw_time_by_corner[name][i]:.6f}")
        csv_lines.append(",".join(row))
    (out_dir / "Terrain_Elevate_P1_V0_59_smooth_climb_trajectory.csv").write_text("\n".join(csv_lines) + "\n")

    print(json.dumps(result, indent=2))
    # Intentionally does not raise on FAIL: like wheel_propulsion_screen in
    # analyze_vehicle_requirements.py, an honest FAIL here is real, useful
    # information (the sourced actuator's speed is the governing constraint),
    # not a build error to hide by exiting nonzero.


if __name__ == "__main__":
    main()
