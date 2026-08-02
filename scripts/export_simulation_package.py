#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import cadquery as cq
from cadquery import exporters

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.cad_model import Component, build_components


def _inertia_box(mass: float, sx: float, sy: float, sz: float) -> dict[str, float]:
    return {
        "ixx": mass / 12 * (sy * sy + sz * sz),
        "iyy": mass / 12 * (sx * sx + sz * sz),
        "izz": mass / 12 * (sx * sx + sy * sy),
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def _inertia_wheel(mass: float, radius: float, length: float) -> dict[str, float]:
    # Wheel axis is URDF Y. Cylinder inertia about its axis is Iyy.
    i_axis = 0.5 * mass * radius * radius
    i_transverse = mass / 12 * (3 * radius * radius + length * length)
    return {"ixx": i_transverse, "iyy": i_axis, "izz": i_transverse, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}


def _add_inertial(link: ET.Element, mass: float, inertia: dict[str, float]) -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.6g}")
    ET.SubElement(inertial, "inertia", **{k: f"{v:.9g}" for k, v in inertia.items()})


def _add_mesh(link: ET.Element, tag: str, filename: str) -> None:
    parent = ET.SubElement(link, tag)
    ET.SubElement(parent, "origin", xyz="0 0 0", rpy="0 0 0")
    geom = ET.SubElement(parent, "geometry")
    ET.SubElement(geom, "mesh", filename=f"meshes/{filename}", scale="0.001 0.001 0.001")


def _make_stair_mesh(path: Path, rise_mm: float, going_mm: float, width_mm: float = 864.0, steps: int = 6) -> None:
    solids = []
    for i in range(steps):
        solids.append(
            cq.Workplane("XY")
            .box(going_mm, width_mm, rise_mm * (i + 1))
            .translate((i * going_mm + going_mm / 2, 0, rise_mm * (i + 1) / 2 - 1))
            .val()
        )
    exporters.export(cq.Compound.makeCompound(solids), str(path), exportType="STL", tolerance=0.5, angularTolerance=0.25)


def _prefix_to_corner(prefix: str) -> str:
    return {"fl": "front_left", "fr": "front_right", "rl": "rear_left", "rr": "rear_right"}[prefix]


def _component_link(component: Component) -> str:
    name = component.name.lower()
    role = component.role.lower()
    if name.startswith("ref_") or component.material == "reference":
        return "reference"
    if any(token in name or token in role for token in ("seat", "pod", "five_point", "harness_bar", "electrak", "leveling", "mechanical_level_lock")):
        return "occupant_pod"
    for prefix in ("fl", "fr", "rl", "rr"):
        if name.startswith(prefix + "_"):
            corner = _prefix_to_corner(prefix)
            if any(token in name for token in ("tire", "hub_shell", "live_axle", "axle_inner_spacer", "axle_left_retaining_nut", "axle_right_retaining_nut", "6002_hub_bearing")):
                return f"{corner}_wheel"
            if any(token in name for token in ("fork", "wheel_drive", "planetary", "service_disc", "antirollback_dog")):
                return f"{corner}_wheel_module"
            if any(token in name for token in ("ballnut", "moving_slider", "slider_", "bellcrank", "pushrod", "rod_end", "clevis_m10", "jam_nut")):
                return f"{corner}_slider"
            return "chassis"
    return "chassis"


def _export_cad_link_meshes(mesh_dir: Path, params: dict) -> dict[str, int]:
    groups: dict[str, list[Component]] = {}
    for component in build_components(params, include_reference=False):
        link = _component_link(component)
        if link == "reference":
            continue
        groups.setdefault(link, []).append(component)

    component_counts: dict[str, int] = {}
    for link_name, components in groups.items():
        compound = cq.Compound.makeCompound([component.shape.val() for component in components])
        exporters.export(compound, str(mesh_dir / f"{link_name}.stl"), exportType="STL", tolerance=0.35, angularTolerance=0.2)
        component_counts[link_name] = len(components)
    return component_counts


def _joint(robot: ET.Element, name: str, joint_type: str, parent: str, child: str, xyz: tuple[float, float, float], axis: tuple[float, float, float], limit: dict[str, float] | None = None) -> None:
    joint = ET.SubElement(robot, "joint", name=name, type=joint_type)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=" ".join(f"{v:.6g}" for v in xyz), rpy="0 0 0")
    ET.SubElement(joint, "axis", xyz=" ".join(f"{v:.6g}" for v in axis))
    if limit:
        ET.SubElement(joint, "limit", **{k: f"{v:.6g}" for k, v in limit.items()})


def main() -> None:
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    mass_props = json.loads((ROOT / "data" / "te_v059_mass_properties.json").read_text())
    g = params["geometry"]

    out_dir = ROOT / "sim_out"
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    link_component_counts = _export_cad_link_meshes(mesh_dir, params)
    _make_stair_mesh(mesh_dir / "reference_stairs_203r_279g.stl", g["stair_rise_reference_mm"], g["stair_going_reference_mm"])

    robot = ET.Element("robot", name="terrain_elevate_p1_v059")
    for link_name, props in mass_props["links"].items():
        link = ET.SubElement(robot, "link", name=link_name)
        mass = props["mass_kg"]
        if "box_inertia_size_m" in props:
            inertia = _inertia_box(mass, *props["box_inertia_size_m"])
        else:
            inertia = _inertia_wheel(mass, props["cylinder_inertia_radius_m"], props["cylinder_inertia_length_m"])
        _add_inertial(link, mass, inertia)
        _add_mesh(link, "visual", f"{link_name}.stl")
        _add_mesh(link, "collision", f"{link_name}.stl")

    wheelbase_m = g["wheelbase_mm"] / 1000
    track_m = g["track_mm"] / 1000
    radius_m = g["wheel_diameter_mm"] / 2000
    stroke_m = g["corner_stroke_selected_mm_SRC"] / 1000
    pitch_limit = math.radians(g["pod_pitch_correction_deg_CALC"] + 5.0)
    actuator_effort_n = 2511.0
    wheel_effort_nm = 60.0

    _joint(robot, "pod_pitch_leveling_joint", "revolute", "chassis", "occupant_pod", (0, 0, 0.26), (0, 1, 0), {"lower": -pitch_limit, "upper": pitch_limit, "effort": 2000, "velocity": 0.35})

    corners = {
        "front_left": (wheelbase_m / 2, track_m / 2),
        "front_right": (wheelbase_m / 2, -track_m / 2),
        "rear_left": (-wheelbase_m / 2, track_m / 2),
        "rear_right": (-wheelbase_m / 2, -track_m / 2),
    }
    for name, (x, y) in corners.items():
        slider = f"{name}_slider"
        module = f"{name}_wheel_module"
        wheel = f"{name}_wheel"
        _joint(robot, f"{name}_corner_prismatic_actuator", "prismatic", "chassis", slider, (x, y, 0), (0, 0, 1), {"lower": 0, "upper": stroke_m, "effort": actuator_effort_n, "velocity": 0.022})
        _joint(robot, f"{name}_fork_carrier_fixed", "fixed", slider, module, (0, 0, -0.23), (0, 0, 1))
        _joint(robot, f"{name}_wheel_drive_joint", "continuous", module, wheel, (0, 0, -radius_m), (0, 1, 0), {"effort": wheel_effort_nm, "velocity": 35.0})

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    urdf_path = out_dir / "Terrain_Elevate_P1_V0_59_sim.urdf"
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)

    joint_graph = {
        "truth_boundary": mass_props["truth_boundary"],
        "format": "URDF plus STL collision/visual meshes for mechanism simulation import",
        "mesh_source": "CAD-derived grouped meshes from the detailed 793-body CadQuery assembly; not primitive placeholder boxes.",
        "link_component_counts": link_component_counts,
        "links": list(mass_props["links"].keys()),
        "degrees_of_freedom": {
            "pod_pitch_leveling_joint": 1,
            "independent_corner_prismatic_actuators": 4,
            "wheel_drive_joints": 4,
            "total_commanded_dof": 9
        },
        "limits": {
            "corner_stroke_m": stroke_m,
            "corner_prismatic_effort_N": actuator_effort_n,
            "corner_prismatic_velocity_m_s": 0.022,
            "pod_pitch_limit_rad": pitch_limit,
            "wheel_drive_effort_Nm": wheel_effort_nm
        },
        "coordinate_system": "meters, URDF frame; +X forward, +Y left, +Z up",
        "terrain_mesh": "meshes/reference_stairs_203r_279g.stl",
        "urdf": urdf_path.name,
    }
    graph_path = out_dir / "Terrain_Elevate_P1_V0_59_joint_graph.json"
    graph_path.write_text(json.dumps(joint_graph, indent=2))
    print(json.dumps({"status": "PASS", "urdf": str(urdf_path), "joint_graph": str(graph_path), "mesh_count": len(list(mesh_dir.glob('*.stl')))}, indent=2))


if __name__ == "__main__":
    main()
