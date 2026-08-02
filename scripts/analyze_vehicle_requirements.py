#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    load = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    g = params["geometry"]
    mass = load["mass_assumptions"]["heavy_design_screen_mass_kg_ASSUMED"]
    dyn = load["dynamic_factors"]
    actuator = load["actuator_assumptions"]
    wheel = load["wheel_assumptions"]
    pod = load["pod_leveling_assumptions"]

    gravity = 9.80665
    rise = g["stair_rise_reference_mm"] / 1000
    going = g["stair_going_reference_mm"] / 1000
    wheelbase = g["wheelbase_mm"] / 1000
    wheel_radius = g["wheel_diameter_mm"] / 2000
    lead = 0.004
    theta = math.atan2(rise, going)
    theta_deg = math.degrees(theta)
    total_weight = mass * gravity

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
    brake_hold_torque_required = governing_corner_force * lead / (2 * math.pi * screw_eff) * actuator["holding_brake_safety_factor"]
    selected_brake_torque = 5.0

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
            "holding_brake_torque_required_Nm_with_sf": brake_hold_torque_required,
            "selected_ab60s_holding_brake_torque_Nm": selected_brake_torque,
            "selected_brake_margin_Nm": selected_brake_torque - brake_hold_torque_required,
            "result": "PASS_FOR_SCREENING" if screw_axial_capacity >= governing_corner_force else "FAIL",
        },
        "wheel_propulsion_screen": {
            "slope_climb_force_N": climb_force,
            "per_wheel_climb_torque_Nm_with_sf": per_wheel_climb_torque,
            "traction_limited_force_N_at_assumed_mu": traction_limited_force,
            "traction_margin_N": traction_limited_force - climb_force,
            "result": "PASS_TRACTION_SCREEN" if traction_limited_force >= climb_force else "FAIL_TRACTION_SCREEN",
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
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
