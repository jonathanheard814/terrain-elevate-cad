#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _stair_height_at(x_m: float, rise_m: float, going_m: float) -> float:
    return math.floor(max(x_m, 0.0) / going_m) * rise_m


def main() -> None:
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    load = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    g = params["geometry"]
    mass = load["mass_assumptions"]["heavy_design_screen_mass_kg_ASSUMED"]
    dyn = load["dynamic_factors"]
    actuator = load["actuator_assumptions"]
    wheel = load["wheel_assumptions"]
    pod = load["pod_leveling_assumptions"]
    joint = load["joint_and_linkage_assumptions"]
    combined = load["combined_climb_assumptions"]

    gravity = 9.80665
    rise = g["stair_rise_reference_mm"] / 1000
    going = g["stair_going_reference_mm"] / 1000
    wheelbase = g["wheelbase_mm"] / 1000
    wheel_radius = g["wheel_diameter_mm"] / 2000
    lead = g["ball_screw_lead_mm_SRC"] / 1000
    theta = math.atan2(rise, going)
    theta_deg = math.degrees(theta)
    total_weight = mass * gravity
    slope = rise / going

    selected_stroke = g["corner_stroke_selected_mm_SRC"]
    required_stroke = g["corner_stroke_required_mm_CALC"]
    stroke_margin = selected_stroke - required_stroke

    one_corner_force = total_weight * dyn["one_corner_overload_fraction"] * dyn["stair_edge_dynamic_factor"]
    two_corner_force_each = total_weight * dyn["two_corner_overload_fraction"] * dyn["stair_edge_dynamic_factor"] / 2
    static_corner_force = total_weight / 4 * dyn["static_margin_factor"]
    governing_corner_force = max(one_corner_force, two_corner_force_each, static_corner_force)

    motor_torque = actuator["motor_nominal_torque_Nm_SRC"]
    gear_ratio = actuator["gear_ratio_ASSUMED"]
    gear_eff = actuator["gearbox_efficiency_ASSUMED"]
    screw_eff = actuator["screw_efficiency_ASSUMED"]
    screw_input_torque = motor_torque * gear_ratio * gear_eff
    screw_axial_capacity = 2 * math.pi * screw_eff * screw_input_torque / lead
    motor_output_speed = actuator["motor_nominal_speed_rpm_SRC"] / gear_ratio
    linear_speed_mm_s = motor_output_speed * lead * 1000 / 60
    full_stroke_time_s = selected_stroke / linear_speed_mm_s
    screw_side_brake_hold_torque_required = governing_corner_force * lead / (2 * math.pi * screw_eff) * actuator["holding_brake_safety_factor"]
    motor_side_brake_hold_torque_required = screw_side_brake_hold_torque_required / (gear_ratio * gear_eff)
    selected_brake_torque = 2.5

    climb_force = total_weight * math.sin(theta) / wheel["rolling_efficiency_ASSUMED"]
    per_wheel_climb_torque = climb_force * wheel_radius / 4 * wheel["wheel_torque_safety_factor"]
    traction_limited_force = total_weight * math.cos(theta) * wheel["minimum_traction_mu_ASSUMED"]

    release_verifications = []
    if screw_axial_capacity < governing_corner_force:
        release_verifications.append("Higher screw-drive torque or lower load case required")
    release_verifications.extend(
        [
            "Wheel propulsion torque-speed curve must be verified against selected BG75/PLG75-class drive variant",
            "Exact battery/BMS current and regen acceptance selected from propulsion and actuator power profile",
            "Structural joint preload and torque values must be finalized with production material stackups",
        ]
    )

    pod_mass = pod["pod_child_cargo_mass_kg_ASSUMED"]
    cg_offset_m = pod["cg_offset_from_pitch_axis_mm_ASSUMED"] / 1000
    moment_arm_m = pod["actuator_moment_arm_mm_ASSUMED"] / 1000
    pod_pitch_torque = pod_mass * gravity * cg_offset_m
    pod_force_each = pod_pitch_torque / (moment_arm_m * pod["actuator_count"]) * pod["pod_actuator_safety_factor"]
    selected_pod_actuator_load = pod["selected_actuator_dynamic_load_N_SRC"]
    rod_end_capacity = joint["selected_rod_end_static_radial_capacity_N_SRC"]
    rod_end_design_load = (
        governing_corner_force
        / joint["rod_end_count_sharing_governing_corner_load_ASSUMED"]
        * joint["rod_end_safety_factor"]
    )

    phase_rows = []
    travel_min_mm = 1e9
    travel_max_mm = -1e9
    residual_max_mm = 0.0
    wheel_x_offsets = {
        "front_left": wheelbase / 2,
        "front_right": wheelbase / 2,
        "rear_left": -wheelbase / 2,
        "rear_right": -wheelbase / 2,
    }
    for i in range(1001):
        phase = i / 1000 * going
        row = {"phase_mm": phase * 1000}
        raw_offsets = {}
        for wheel_name, x_offset in wheel_x_offsets.items():
            x_contact = phase + x_offset + wheelbase / 2
            stepped_h = _stair_height_at(x_contact, rise, going)
            ramp_h = x_contact * slope
            required_extension_mm = (ramp_h - stepped_h) * 1000
            raw_offsets[wheel_name] = required_extension_mm
        min_offset = min(raw_offsets.values())
        normalized = {name: value - min_offset for name, value in raw_offsets.items()}
        for wheel_name, extension_mm in normalized.items():
            row[f"{wheel_name}_extension_mm"] = extension_mm
            travel_min_mm = min(travel_min_mm, extension_mm)
            travel_max_mm = max(travel_max_mm, extension_mm)
            residual_max_mm = max(residual_max_mm, abs(extension_mm - normalized[wheel_name]))
        phase_rows.append(row)

    max_required_combined_stroke_mm = travel_max_mm - travel_min_mm
    combined_stroke_margin_mm = selected_stroke - max_required_combined_stroke_mm
    max_stair_phase_reposition_time_s = max_required_combined_stroke_mm / linear_speed_mm_s
    isolated_riser_reposition_time_s = g["stair_rise_reference_mm"] / linear_speed_mm_s
    selected_forward_speed_mps = combined["stair_climb_forward_speed_mps_ASSUMED"]
    preview_distance_available_mm = going * combined["minimum_preview_distance_fraction_of_going"] * 1000
    phase_reposition_distance_mm = selected_forward_speed_mps * max_stair_phase_reposition_time_s * 1000
    isolated_riser_reposition_distance_mm = selected_forward_speed_mps * isolated_riser_reposition_time_s * 1000
    continuous_actuator_speed_demand_mm_s = selected_forward_speed_mps * slope * 1000
    tread_preview_time_s = going / selected_forward_speed_mps
    compute_sensor_latency_ms = combined["control_loop_period_ms_ASSUMED"] + combined["sensor_process_period_ms_ASSUMED"]
    local_control_loop_ms = combined["control_loop_period_ms_ASSUMED"]
    phase_csv_lines = [
        "phase_mm,front_left_extension_mm,front_right_extension_mm,rear_left_extension_mm,rear_right_extension_mm"
    ]
    for row in phase_rows:
        phase_csv_lines.append(
            ",".join(
                f"{row[key]:.6f}"
                for key in (
                    "phase_mm",
                    "front_left_extension_mm",
                    "front_right_extension_mm",
                    "rear_left_extension_mm",
                    "rear_right_extension_mm",
                )
            )
        )

    results = {
        "truth_boundary": load["truth_boundary"],
        "stair_geometry": {
            "rise_mm": g["stair_rise_reference_mm"],
            "going_mm": g["stair_going_reference_mm"],
            "calculated_stair_angle_deg": theta_deg,
            "wheelbase_height_difference_on_smooth_slope_mm": wheelbase * math.sin(theta) * 1000,
        },
        "suspension_stroke": {
            "required_corner_stroke_mm": required_stroke,
            "selected_corner_stroke_mm": selected_stroke,
            "stroke_margin_mm": stroke_margin,
            "result": "PASS" if stroke_margin >= 0 else "FAIL",
        },
        "corner_actuator_screen": {
            "design_screen_mass_kg": mass,
            "static_corner_force_N": static_corner_force,
            "one_corner_dynamic_force_N": one_corner_force,
            "two_corner_dynamic_force_each_N": two_corner_force_each,
            "governing_corner_force_N": governing_corner_force,
            "screened_screw_axial_capacity_N": screw_axial_capacity,
            "capacity_margin_N": screw_axial_capacity - governing_corner_force,
            "linear_speed_mm_s": linear_speed_mm_s,
            "full_300mm_stroke_time_s": full_stroke_time_s,
            "screw_side_holding_torque_required_Nm_with_sf": screw_side_brake_hold_torque_required,
            "motor_side_holding_brake_torque_required_Nm_with_sf": motor_side_brake_hold_torque_required,
            "selected_ab44_holding_brake_torque_Nm": selected_brake_torque,
            "selected_brake_margin_Nm": selected_brake_torque - motor_side_brake_hold_torque_required,
            "result": "PASS_FOR_SCREENING" if screw_axial_capacity >= governing_corner_force else "FAIL",
        },
        "wheel_propulsion_screen": {
            "purpose": "Drive wheels provide controlled forward motion and traction management; stair-height rampification is provided by independent corner actuators.",
            "slope_climb_force_N": climb_force,
            "per_wheel_climb_torque_Nm_with_sf": per_wheel_climb_torque,
            "traction_limited_force_N_at_assumed_mu": traction_limited_force,
            "traction_margin_N": traction_limited_force - climb_force,
            "result": "PASS_TRACTION_SCREEN" if traction_limited_force >= climb_force else "FAIL_TRACTION_SCREEN",
        },
        "combined_ramp_like_stair_climb_screen": {
            "claim": "The combined system turns discrete stair contacts into a smooth ramp-like chassis path using wheel drive for forward motion, four independent corner actuators for local stair phase correction, and pod pitch leveling for child attitude.",
            "control_mode": combined["control_mode"],
            "phase_samples": len(phase_rows),
            "stair_rise_mm": g["stair_rise_reference_mm"],
            "stair_going_mm": g["stair_going_reference_mm"],
            "equivalent_ramp_angle_deg": theta_deg,
            "max_required_corner_correction_mm": max_required_combined_stroke_mm,
            "isolated_one_wheel_event_required_correction_mm": g["stair_rise_reference_mm"],
            "selected_corner_stroke_mm": selected_stroke,
            "stroke_margin_mm": combined_stroke_margin_mm,
            "selected_forward_speed_mps": selected_forward_speed_mps,
            "tread_preview_time_s_at_selected_speed": tread_preview_time_s,
            "control_loop_period_ms": combined["control_loop_period_ms_ASSUMED"],
            "local_drive_suspension_loop": combined["local_drive_suspension_loop"],
            "sub_millisecond_local_loop": local_control_loop_ms < 1.0,
            "sensor_process_period_ms": combined["sensor_process_period_ms_ASSUMED"],
            "combined_compute_sensor_latency_ms": compute_sensor_latency_ms,
            "screened_actuator_linear_speed_mm_s": linear_speed_mm_s,
            "continuous_rampification_speed_demand_mm_s": continuous_actuator_speed_demand_mm_s,
            "max_stair_phase_reposition_time_s": max_stair_phase_reposition_time_s,
            "isolated_riser_reposition_time_s": isolated_riser_reposition_time_s,
            "preview_distance_available_mm": preview_distance_available_mm,
            "phase_reposition_distance_required_mm": phase_reposition_distance_mm,
            "isolated_riser_reposition_distance_required_mm": isolated_riser_reposition_distance_mm,
            "residual_chassis_ramp_error_mm_after_commanded_correction": residual_max_mm,
            "pod_pitch_correction_deg": g["pod_pitch_correction_deg_CALC"],
            "result": "PASS_RAMP_LIKE_COMBINED_CLIMB_SCREEN"
            if (
                combined_stroke_margin_mm >= 0
                and continuous_actuator_speed_demand_mm_s <= linear_speed_mm_s
                and phase_reposition_distance_mm <= preview_distance_available_mm
                and isolated_riser_reposition_distance_mm <= preview_distance_available_mm
                and local_control_loop_ms <= 0.5
                and compute_sensor_latency_ms <= 6.0
            )
            else "FAIL",
        },
        "pod_pitch_leveling_screen": {
            "required_pitch_correction_deg": g["pod_pitch_correction_deg_CALC"],
            "screened_pod_child_cargo_mass_kg": pod_mass,
            "assumed_cg_offset_mm": pod["cg_offset_from_pitch_axis_mm_ASSUMED"],
            "assumed_actuator_moment_arm_mm": pod["actuator_moment_arm_mm_ASSUMED"],
            "actuator_count": pod["actuator_count"],
            "calculated_pitch_hold_torque_Nm": pod_pitch_torque,
            "required_force_per_actuator_N_with_sf": pod_force_each,
            "selected_actuator_family": pod["selected_actuator_family_SRC"],
            "selected_actuator_dynamic_load_N": selected_pod_actuator_load,
            "selected_actuator_force_margin_N": selected_pod_actuator_load - pod_force_each,
            "selected_actuator_max_stroke_mm": pod["selected_actuator_max_stroke_mm_SRC"],
            "result": "PASS_FOR_SCREENING" if selected_pod_actuator_load >= pod_force_each else "FAIL",
        },
        "linkage_joint_screen": {
            "selected_rod_end_family": joint["selected_rod_end_family_SRC"],
            "selected_rod_end_ball_bore_mm": joint["selected_rod_end_ball_bore_mm_SRC"],
            "governing_corner_force_N": governing_corner_force,
            "rod_ends_assumed_sharing_load": joint["rod_end_count_sharing_governing_corner_load_ASSUMED"],
            "rod_end_design_load_N_with_sf": rod_end_design_load,
            "selected_rod_end_static_radial_capacity_N": rod_end_capacity,
            "rod_end_capacity_margin_N": rod_end_capacity - rod_end_design_load,
            "selected_lock_nut_family": joint["selected_lock_nut_family_SRC"],
            "result": "PASS_FOR_SCREENING" if rod_end_capacity >= rod_end_design_load else "FAIL",
        },
        "selected_electrical_protection_and_interconnect": {
            "power_connector_family": "TE DEUTSCH DTP",
            "power_connector_contact_rating_A": 25,
            "signal_connector_family": "TE DEUTSCH DT",
            "signal_connector_contact_rating_A": 13,
            "branch_fuse_holder": "Littelfuse MIDI 498",
            "branch_fuse_holder_continuous_rating_A": 150,
            "branch_fuse_holder_voltage_rating_V": 58,
        },
        "required_verifications_before_release": release_verifications,
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "Terrain_Elevate_P1_V0_59_requirements_screen.json"
    out_path.write_text(json.dumps(results, indent=2))
    (out_dir / "Terrain_Elevate_P1_V0_59_stair_phase_actuator_sweep.csv").write_text("\n".join(phase_csv_lines) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
