#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import trimesh


ROOT = Path(__file__).resolve().parents[1]


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def _has_module(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def main() -> None:
    errors: list[str] = []
    warnings: list[str] = []
    physics = _load(ROOT / "data" / "te_v059_physics_elements.json")
    params = _load(ROOT / "data" / "te_v059_parameters.json")
    load_cases = _load(ROOT / "data" / "te_v059_load_cases.json")
    requirements = _load(ROOT / "analysis_out" / "Terrain_Elevate_P1_V0_59_requirements_screen.json")
    joint_graph = _load(ROOT / "sim_out" / "Terrain_Elevate_P1_V0_59_joint_graph.json")

    installed = {name: _has_module(name) for name in ("cadquery", "scipy", "casadi", "trimesh", "pybullet")}
    for module_name in ("cadquery", "scipy", "casadi", "trimesh"):
        if not installed[module_name]:
            errors.append(f"Required simulation/math module is not importable: {module_name}")
    if not installed["pybullet"]:
        warnings.append("Optional PyBullet rigid-body contact backend is not installed on this host; MATLAB/SolidWorks/URDF remains the primary simulation handoff.")

    g = params["geometry"]
    actuator = load_cases["actuator_assumptions"]
    model = physics["actuator_models"]["corner_vertical_actuator"]
    screw_force = 2 * math.pi * actuator["screw_efficiency_ASSUMED"] * actuator["motor_nominal_torque_Nm_SRC"] * actuator["gear_ratio_ASSUMED"] * actuator["gearbox_efficiency_ASSUMED"] / (g["ball_screw_lead_mm_SRC"] / 1000)
    screw_speed = actuator["motor_nominal_speed_rpm_SRC"] / actuator["gear_ratio_ASSUMED"] * (g["ball_screw_lead_mm_SRC"] / 1000) / 60
    if abs(screw_force - model["screened_axial_force_N"]) > 1.0:
        errors.append("Physics actuator force does not match selected motor/gear/screw calculation")
    if abs(screw_speed - model["screened_linear_speed_mps"]) > 0.001:
        errors.append("Physics actuator speed does not match selected motor/gear/screw calculation")
    limits = joint_graph["limits"]
    if abs(limits["corner_prismatic_effort_N"] - screw_force) > 1.0:
        errors.append("URDF joint graph actuator effort does not match physics basis")
    if abs(limits["corner_prismatic_velocity_m_s"] - screw_speed) > 0.001:
        errors.append("URDF joint graph actuator speed does not match physics basis")

    combined = requirements["combined_ramp_like_stair_climb_screen"]
    timing = physics["sensor_and_control_timing"]
    if timing["local_motor_suspension_command_loop_s"] > 0.0005:
        errors.append("Local motor/suspension loop is not at or below 0.5 ms")
    if timing["corner_position_sensor"]["update_period_s"] > timing["local_motor_suspension_command_loop_s"]:
        errors.append("Corner position feedback is slower than the local suspension loop target")
    if timing["stair_preview_distance_sensor"]["process_period_s"] > 0.005:
        errors.append("Stair preview sensor process period exceeds the selected 5 ms target")
    if abs(timing["combined_compute_sensor_latency_s"] * 1000 - combined["combined_compute_sensor_latency_ms"]) > 0.05:
        errors.append("Physics timing pack does not match requirements screen timing")
    if timing["selected_forward_speed_mps"] < 0.2:
        errors.append("Physics timing pack forward speed is below no-break-stride acceptance target")

    contact = physics["contact_model"]["wheel_tire_on_dry_stair"]
    if contact["dynamic_friction_mu_ASSUMED"] < load_cases["wheel_assumptions"]["minimum_traction_mu_ASSUMED"]:
        errors.append("Physics tire contact dynamic friction is below the wheel traction screening assumption")
    if contact["normal_stiffness_N_per_m"] <= 0 or contact["normal_damping_Ns_per_m"] <= 0:
        errors.append("Contact model must include positive tire normal stiffness and damping")

    mesh_dir = ROOT / "sim_out" / "meshes"
    mesh_reports = {}
    for mesh_path in sorted(mesh_dir.glob("*.stl")):
        mesh = trimesh.load_mesh(mesh_path, force="mesh")
        extents = [float(v) for v in mesh.extents]
        mesh_reports[mesh_path.name] = {
            "vertices": int(len(mesh.vertices)),
            "faces": int(len(mesh.faces)),
            "extents_m": extents,
            "is_watertight": bool(mesh.is_watertight),
        }
        if len(mesh.vertices) < 4 or len(mesh.faces) < 2:
            errors.append(f"Mesh has too little geometry for simulation import: {mesh_path.name}")
        if not all(math.isfinite(v) and v > 0 for v in extents):
            errors.append(f"Mesh has invalid extents: {mesh_path.name}")

    result = {
        "result": "PASS" if not errors else "FAIL",
        "errors": errors,
        "warnings": warnings,
        "installed_modules": installed,
        "actuator_force_N": screw_force,
        "actuator_velocity_mps": screw_speed,
        "local_loop_s": timing["local_motor_suspension_command_loop_s"],
        "position_sensor_update_s": timing["corner_position_sensor"]["update_period_s"],
        "preview_sensor_process_s": timing["stair_preview_distance_sensor"]["process_period_s"],
        "mesh_reports": mesh_reports,
        "truth_boundary": physics["truth_boundary"],
    }
    out = ROOT / "analysis_out" / "Terrain_Elevate_P1_V0_59_physics_basis_audit.json"
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
