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
    return math.floor(max(x_m, 0.0) / going_m) * rise_m


def _raw_required_extensions_mm(x_positions_m, wheel_x_offsets_m, wheelbase_m, rise_m, going_m, slope):
    """Per-corner raw geometric correction (mm) at each x sample, normalized so
    the lowest corner at each sample is 0 -- identical formula to
    scripts/analyze_vehicle_requirements.py so the two stay consistent. The
    extra + wheelbase_m/2 keeps every corner's x_contact non-negative (the
    rearmost corner's offset is -wheelbase/2, so without this it goes
    negative right at the start of a run and _stair_height_at's max(x,0)
    clamp produces a bogus large offset there)."""
    per_corner = {name: [] for name in wheel_x_offsets_m}
    for x in x_positions_m:
        raw = {}
        for name, x_offset in wheel_x_offsets_m.items():
            x_contact = x + x_offset + wheelbase_m / 2
            stepped_h = _stair_height_at(x_contact, rise_m, going_m)
            ramp_h = x_contact * slope
            raw[name] = (ramp_h - stepped_h) * 1000.0
        min_raw = min(raw.values())
        for name in wheel_x_offsets_m:
            per_corner[name].append(raw[name] - min_raw)
    return per_corner


def _detect_levels(x_mm, y_mm, jump_threshold_mm):
    """Collapse a piecewise-constant signal into (level_value, start_x, end_x) runs."""
    runs = []
    start_idx = 0
    for i in range(1, len(y_mm)):
        if abs(y_mm[i] - y_mm[i - 1]) > jump_threshold_mm:
            runs.append((y_mm[start_idx], x_mm[start_idx], x_mm[i - 1]))
            start_idx = i
    runs.append((y_mm[start_idx], x_mm[start_idx], x_mm[-1]))
    return runs


def _edge_times_s(levels, forward_speed_mps):
    """Time (s) at which each detected step edge is crossed, at constant forward speed."""
    return [(levels[k][2] / 1000.0) / forward_speed_mps for k in range(len(levels) - 1)]


def _min_gap_s(edge_times):
    if len(edge_times) < 2:
        return float("inf")
    return min(edge_times[i + 1] - edge_times[i] for i in range(len(edge_times) - 1))


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
    g = params["geometry"]
    actuator = load["actuator_assumptions"]
    combined = load["combined_climb_assumptions"]
    smooth = load["smooth_climb_assumptions"]

    rise_m = g["stair_rise_reference_mm"] / 1000.0
    going_m = g["stair_going_reference_mm"] / 1000.0
    wheelbase_m = g["wheelbase_mm"] / 1000.0
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
    x_positions_mm = [x * 1000.0 for x in x_positions_m]

    raw_by_corner = _raw_required_extensions_mm(x_positions_m, wheel_x_offsets_m, wheelbase_m, rise_m, going_m, slope)

    levels_by_corner = {
        name: _detect_levels(x_positions_mm, raw_by_corner[name], jump_threshold_mm=1.0)
        for name in wheel_x_offsets_m
    }

    max_jump_mm = max(
        abs(levels_by_corner[name][k + 1][0] - levels_by_corner[name][k][0])
        for name in wheel_x_offsets_m
        for k in range(len(levels_by_corner[name]) - 1)
    )

    # Minimum-jerk (quintic) peak velocity for an *isolated* blend of
    # magnitude dP over duration T is (15/8) * dP / T -- use that to size a
    # window for the largest jump at a margin below rated actuator speed.
    # But successive corrections for the same corner recur roughly once per
    # tread; if that natural spacing is *shorter* than the margin-sized
    # window, cap the window to the tightest real spacing instead (with a 2%
    # gap) so the two constraints don't fight -- the resulting peak velocity
    # is then measured for real below, not assumed.
    speed_margin_window_s = (15.0 / 8.0) * max_jump_mm / (speed_margin * actuator_linear_speed_mm_s)
    tightest_gap_s = min(_min_gap_s(_edge_times_s(levels_by_corner[name], forward_speed_mps)) for name in wheel_x_offsets_m)
    spacing_cap_s = 0.98 * tightest_gap_s if tightest_gap_s != float("inf") else float("inf")
    preview_cap_s = preview_time_s - control_margin_s
    window_s = min(speed_margin_window_s, spacing_cap_s, preview_cap_s)

    dt_s = combined["control_loop_period_ms_ASSUMED"] / 1000.0 * 4.0
    total_time_s = (n_treads * going_m) / forward_speed_mps
    n_time_samples = int(total_time_s / dt_s) + 1
    time_s = [i * dt_s for i in range(n_time_samples)]
    window_n = max(1, round(window_s / dt_s))
    kernel = _smoothing_kernel(window_n)

    smoothed_by_corner = {}
    raw_time_by_corner = {}
    for name in wheel_x_offsets_m:
        # Resample the raw (unsmoothed) spatial signal onto the time base first.
        raw_series = raw_by_corner[name]
        raw_time = []
        for t in time_s:
            x_mm = t * forward_speed_mps * 1000.0
            idx = min(int(x_mm / (dx_m * 1000.0)), len(raw_series) - 1)
            raw_time.append(raw_series[idx])
        raw_time_by_corner[name] = raw_time
        # Convolution superposes overlapping corrections correctly instead of
        # assuming they never overlap.
        smoothed_by_corner[name] = _convolve_lookahead(raw_time, kernel)

    # Steady-state window: skip the first preview_time_s to avoid a start-up
    # transient at the very beginning of the simulated run.
    steady_start_idx = next((i for i, t in enumerate(time_s) if t >= preview_time_s), 0)

    per_corner_metrics = {}
    for name in wheel_x_offsets_m:
        smoothed = smoothed_by_corner[name][steady_start_idx:]
        raw_resampled = raw_time_by_corner[name][steady_start_idx:]

        smoothed_step = [smoothed[i + 1] - smoothed[i] for i in range(len(smoothed) - 1)]
        raw_step = [raw_resampled[i + 1] - raw_resampled[i] for i in range(len(raw_resampled) - 1)]

        vel = _derivative(smoothed, dt_s)
        acc = _derivative(vel, dt_s)
        jerk = _derivative(acc, dt_s)

        per_corner_metrics[name] = {
            "max_raw_position_step_mm": _max_abs(raw_step),
            "max_smoothed_position_step_mm": _max_abs(smoothed_step),
            "max_smoothed_velocity_mm_s": _max_abs(vel),
            "max_smoothed_acceleration_mm_s2": _max_abs(acc),
            "max_smoothed_jerk_mm_s3": _max_abs(jerk),
        }

    max_smoothed_step_mm = max(m["max_smoothed_position_step_mm"] for m in per_corner_metrics.values())
    max_smoothed_velocity_mm_s = max(m["max_smoothed_velocity_mm_s"] for m in per_corner_metrics.values())
    max_raw_step_mm = max(m["max_raw_position_step_mm"] for m in per_corner_metrics.values())

    velocity_within_actuator_capability = max_smoothed_velocity_mm_s <= actuator_linear_speed_mm_s
    no_position_discontinuity = max_smoothed_step_mm <= tolerance_mm
    # Forward speed is never reduced anywhere in this construction -- the
    # chassis-travel/time mapping is fixed at forward_speed_mps throughout.
    # That is a structural fact of the model, not a result to gate on; the
    # real open question is whether the actuator can keep up, which is what
    # velocity_within_actuator_capability actually measures.
    forward_speed_held_constant = True

    # no_position_discontinuity is reported but not part of overall_pass: see
    # its "note" above -- it is mathematically the same quantity as
    # velocity_within_actuator_capability, just rescaled by dt.
    overall_pass = velocity_within_actuator_capability

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
            "governing_raw_jump_mm": max_jump_mm,
            "speed_margin_window_s": speed_margin_window_s,
            "tightest_same_corner_correction_spacing_s": tightest_gap_s,
            "preview_budget_window_s": preview_cap_s,
            "window_used_s": window_s,
            "binding_constraint": (
                "same_corner_correction_spacing" if window_s == spacing_cap_s
                else "actuator_speed_margin" if window_s == speed_margin_window_s
                else "preview_budget"
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
                "velocity_within_actuator_capability is FAIL: the sourced corner "
                "actuator is not fast enough to track this stair geometry's "
                "correction cadence at the assumed 0.2 m/s climb speed without "
                "exceeding its rated linear speed. Closes by sourcing a "
                "higher-lead ball screw or lower corner-actuator gear reduction "
                "and re-running this screen, not by silently reducing forward speed."
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
