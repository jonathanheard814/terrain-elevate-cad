#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "sim_out"


def main() -> None:
    errors: list[str] = []
    urdf = SIM / "Terrain_Elevate_P1_V0_59_sim.urdf"
    graph = SIM / "Terrain_Elevate_P1_V0_59_joint_graph.json"
    if not urdf.exists():
        errors.append(f"Missing {urdf}")
    if not graph.exists():
        errors.append(f"Missing {graph}")

    if urdf.exists():
        root = ET.parse(urdf).getroot()
        links = {link.attrib["name"] for link in root.findall("link")}
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        required_links = {
            "chassis",
            "occupant_pod",
            "front_left_slider",
            "front_right_slider",
            "rear_left_slider",
            "rear_right_slider",
            "front_left_wheel",
            "front_right_wheel",
            "rear_left_wheel",
            "rear_right_wheel",
        }
        missing_links = sorted(required_links - links)
        if missing_links:
            errors.append(f"Missing required URDF links: {missing_links}")

        prismatic = [name for name, joint in joints.items() if joint.attrib.get("type") == "prismatic"]
        continuous = [name for name, joint in joints.items() if joint.attrib.get("type") == "continuous"]
        if len(prismatic) != 4:
            errors.append(f"Expected four independent prismatic corner actuators, got {len(prismatic)}")
        if len(continuous) != 4:
            errors.append(f"Expected four wheel drive joints, got {len(continuous)}")
        if "pod_pitch_leveling_joint" not in joints:
            errors.append("Missing pod pitch leveling joint")

        for mesh in root.findall(".//mesh"):
            filename = mesh.attrib["filename"].replace("meshes/", "")
            if not (SIM / "meshes" / filename).exists():
                errors.append(f"URDF references missing mesh: {filename}")

    if graph.exists():
        data = json.loads(graph.read_text())
        if data.get("degrees_of_freedom", {}).get("total_commanded_dof") != 9:
            errors.append("Simulation joint graph must expose 9 commanded DOF")
        if "CAD-derived" not in data.get("mesh_source", ""):
            errors.append("Simulation meshes must be grouped from the detailed CAD, not placeholder boxes")
        if len(data.get("link_component_counts", {})) < 14:
            errors.append("Simulation package must map CAD components onto all 14 mechanism links")
        terrain = data.get("terrain_mesh")
        if terrain and not (SIM / terrain).exists():
            errors.append(f"Missing terrain mesh: {terrain}")

    if errors:
        print("SIMULATION PACKAGE VALIDATION FAIL")
        for error in errors:
            print("-", error)
        raise SystemExit(1)
    print("SIMULATION PACKAGE VALIDATION PASS")


if __name__ == "__main__":
    main()
