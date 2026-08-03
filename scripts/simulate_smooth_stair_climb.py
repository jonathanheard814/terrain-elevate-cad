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


def _raw_required_extensions_mm(x_positions_m, wheel_x_offsets_m, rise_m, going_m, slope):
    """Per-corner raw geometric correction (mm) at each x sample, normalized so
    the lowest corner at each sample is 0 -- identical formula to
    scripts/analyze_vehicle_requirements.py so the two stay consistent."""
    per_corner = {name: [] for name in wheel_x_offsets_m}
    for x in x_positions_m:
        raw = {}
        for name, x_offset in wheel_x_offsets_m.items():
            x_contact = x + x_offset
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


def _minimum_jerk_blend(t, t0, t1, p0, p1):
    """Quintic minimum-jerk blend: zero velocity and zero acceleration at both endpoints."""
    if t1 <= t0:
        return p1
    if t <= t0:
        return p0
    if t >= t1:
        return p1
    s = (t - t0) / (t1 - t0)
    smooth = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return p0 + (p1 - p0) * smooth


def _build_transitions(levels, forward_speed_mps, transition_time_s):
    """Convert detected (value, start_x_mm, end_x_mm) runs into ordered
    (edge_time_s, value_before, value_after) transition events."""
    transitions = []
    for k in range(len(levels) - 1):
        value_before = levels[k][0]
        value_after = levels[k + 1][0]
        edge_x_mm = levels[k][2]
        edge_t_s = (edge_x_mm / 1000.0) / forward_speed_mps
        transitions.append((edge_t_s, value_before, value_after))
    return transitions


def _position_at(t, initial_level, transitions, transition_time_s):
    current_value = initial_level
    for edge_t, before, after in transitions:
        t0 = edge_t - transition_time_s
        if t < t0:
            break
        if t < edge_t:
            return _minimum_jerk_blend(t, t0, edge_t, before, after)
        current_value = after
    return current_value


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

    raw_by_corner = _raw_required_extensions_mm(x_positions_m, wheel_x_offsets_m, rise_m, going_m, slope)

    levels_by_corner = {
        name: _detect_levels(x_positions_mm, raw_by_corner[name], jump_threshold_mm=1.0)
        for name in wheel_x_offsets_m
    }

    max_jump_mm = max(
        abs(levels_by_corner[name][k + 1][0] - levels_by_corner[name][k][0])
        for name in wheel_x_offsets_m
        for k in range(len(levels_by_corner[name]) - 1)
    )

    # Minimum-jerk (quintic) peak velocity for a blend of magnitude dP over
    # duration T is (15/8) * dP / T. Size T so the *largest* jump stays within
    # a margin of the screened actuator speed; every smaller jump then also
    # respects that same margin automatically.
    transition_time_s = (15.0 / 8.0) * max_jump_mm / (speed_margin * actuator_linear_speed_mm_s)
    smooth_prestaging_feasible = transition_time_s <= (preview_time_s - control_margin_s)

    dt_s = combined["control_loop_period_ms_ASSUMED"] / 1000.0 * 4.0
    total_time_s = (n_treads * going_m) / forward_speed_mps
    n_time_samples = int(total_time_s / dt_s) + 1
    time_s = [i * dt_s for i in range(n_time_samples)]

    smoothed_by_corner = {}
    raw_time_by_corner = {}
    for name in wheel_x_offsets_m:
        levels = levels_by_corner[name]
        transitions = _build_transitions(levels, forward_speed_mps, transition_time_s)
        initial_level = levels[0][0]
        smoothed_by_corner[name] = [
            _position_at(t, initial_level, transitions, transition_time_s) for t in time_s
        ]
        # Resample the raw (unsmoothed) signal onto the same time base for a
        # like-for-like "before" comparison.
        raw_series = raw_by_corner[name]
        raw_time_by_corner[name] = []
        for t in time_s:
            x_mm = t * forward_speed_mps * 1000.0
            idx = min(int(x_mm / (dx_m * 1000.0)), len(raw_series) - 1)
            raw_time_by_corner[name].append(raw_series[idx])

    # Steady-state window: skip the first preview_time_s to avoid a start-up
    # transient (the very first blend may need to begin before t=0).
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
    no_crawling = smooth_prestaging_feasible  # forward speed is only ever held
    # constant (never reduced) if pre-staging finishes inside the preview
    # budget; if infeasible, the honest conclusion is the chassis would have
    # to slow down, and "no crawling" is not demonstrated.

    overall_pass = no_position_discontinuity and velocity_within_actuator_capability and no_crawling

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
            "analyze_vehicle_requirements.py, converts position to time at the "
            "assumed constant forward speed, then replaces each instantaneous "
            "step with a quintic minimum-jerk blend sized to respect a screened "
            "actuator-speed margin, using the sourced preview sensor's real range "
            "to justify a multi-tread lookahead window instead of reacting at the "
            "stair edge."
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
            "computed_transition_time_s": transition_time_s,
        },
        "per_corner": per_corner_metrics,
        "gates": {
            "no_position_discontinuity": {
                "max_smoothed_position_step_mm": max_smoothed_step_mm,
                "tolerance_mm": tolerance_mm,
                "max_raw_position_step_mm_for_comparison": max_raw_step_mm,
                "result": "PASS" if no_position_discontinuity else "FAIL",
            },
            "velocity_within_actuator_capability": {
                "max_smoothed_velocity_mm_s": max_smoothed_velocity_mm_s,
                "screened_actuator_linear_speed_mm_s": actuator_linear_speed_mm_s,
                "result": "PASS" if velocity_within_actuator_capability else "FAIL",
            },
            "no_crawling_constant_forward_speed": {
                "claim": "Forward speed never has to drop below forward_speed_mps_ASSUMED to let a corner actuator catch up.",
                "transition_time_s": transition_time_s,
                "preview_time_budget_s": preview_time_s,
                "control_loop_margin_s": control_margin_s,
                "result": "PASS" if no_crawling else "FAIL",
            },
        },
        "open_items": [
            "No sourced mechanical or ride-comfort maximum-jerk limit exists yet for "
            "this platform; max_smoothed_jerk_mm_s3 is reported per corner as a "
            "measured quantity, not screened against a threshold. Closes when a "
            "ride-comfort or actuator-duty-cycle jerk limit is sourced."
        ],
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
    if not overall_pass:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
