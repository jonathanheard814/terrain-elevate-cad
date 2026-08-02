#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load(path: Path) -> dict:
    return json.loads(path.read_text())


def main() -> None:
    criteria = _load(ROOT / "data" / "te_v059_acceptance_criteria.json")
    manifest = _load(ROOT / "cad_out" / "Terrain_Elevate_P1_V0_59_manifest.json")
    connectivity = _load(ROOT / "analysis_out" / "Terrain_Elevate_P1_V0_59_connectivity_audit.json")
    requirements = _load(ROOT / "analysis_out" / "Terrain_Elevate_P1_V0_59_requirements_screen.json")
    joint_graph = _load(ROOT / "sim_out" / "Terrain_Elevate_P1_V0_59_joint_graph.json")
    minimums = criteria["minimums"]

    errors: list[str] = []
    if manifest.get("component_count", 0) < minimums["cad_body_count"]:
        errors.append("Detailed CAD body count below acceptance minimum")
    if connectivity.get("largest_connected_group_fraction") != minimums["connectivity_largest_group_fraction"]:
        errors.append("Connectivity audit did not produce one complete assembly group")
    if connectivity.get("isolated_component_count") != minimums["isolated_components"]:
        errors.append("Connectivity audit still has isolated components")
    if connectivity.get("outside_largest_group_count") != minimums["outside_main_group_components"]:
        errors.append("Connectivity audit still has parts outside the main assembly")
    if abs(manifest.get("pose_exports", {}).get("stair_angle_deg", 0) - 36.03) > 0.05:
        errors.append("Stair-climb pose does not use 36.03 degree architecture angle")
    dof = joint_graph.get("degrees_of_freedom", {})
    if dof.get("total_commanded_dof") != minimums["commanded_dof"]:
        errors.append("Simulation model does not expose 9 commanded DOF")
    if dof.get("independent_corner_prismatic_actuators") != minimums["corner_prismatic_joints"]:
        errors.append("Simulation model does not expose four independent corner actuators")
    if dof.get("wheel_drive_joints") != minimums["wheel_drive_joints"]:
        errors.append("Simulation model does not expose four wheel-drive joints")
    if dof.get("pod_pitch_leveling_joint") != minimums["pod_pitch_joints"]:
        errors.append("Simulation model does not expose the pod pitch-leveling joint")
    if "CAD-derived" not in joint_graph.get("mesh_source", ""):
        errors.append("Simulation meshes are not declared as CAD-derived")
    if min(joint_graph.get("link_component_counts", {}).values(), default=0) < 1:
        errors.append("At least one simulation link has no CAD source components")
    if requirements.get("suspension_stroke", {}).get("result") != "PASS":
        errors.append("Suspension stroke requirement does not pass")
    if requirements.get("corner_actuator_screen", {}).get("result") != "PASS_FOR_SCREENING":
        errors.append("Corner actuator screen does not pass")
    if requirements.get("pod_pitch_leveling_screen", {}).get("result") != "PASS_FOR_SCREENING":
        errors.append("Pod pitch leveling screen does not pass")
    if requirements.get("wheel_propulsion_screen", {}).get("result") != "FAIL_TRACTION_SCREEN":
        errors.append("Wheel-only traction screen must remain honest and fail under current assumptions")

    result = {
        "intent": criteria["intent"],
        "result": "PASS" if not errors else "FAIL",
        "errors": errors,
        "checked_outputs": criteria["required_outputs"],
        "truths_enforced": criteria["required_truths"],
    }
    out = ROOT / "analysis_out" / "Terrain_Elevate_P1_V0_59_design_acceptance_audit.json"
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
