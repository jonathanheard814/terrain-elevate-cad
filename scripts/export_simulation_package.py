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


def _check_derived_wheel_geometry(mass_props: dict, radius_m: float, length_m: float) -> None:
    """Fail loudly if mass_properties.json still carries stale wheel geometry.

    The values are no longer read for the export -- geometry comes from
    parameters.json -- but leaving a contradictory number in the data file
    would mislead anyone reading it, so the two must agree.
    """
    mismatches = []
    for name, props in mass_props["links"].items():
        if "cylinder_inertia_radius_m" not in props:
            continue
        if abs(props["cylinder_inertia_radius_m"] - radius_m) > 1e-6:
            mismatches.append(
                f"{name}: cylinder_inertia_radius_m={props['cylinder_inertia_radius_m']} "
                f"but wheel_diameter_mm implies {radius_m}"
            )
        if abs(props["cylinder_inertia_length_m"] - length_m) > 1e-6:
            mismatches.append(
                f"{name}: cylinder_inertia_length_m={props['cylinder_inertia_length_m']} "
                f"but wheel_width_mm implies {length_m}"
            )
    if mismatches:
        raise SystemExit(
            "mass_properties.json wheel geometry disagrees with parameters.json:\n  "
            + "\n  ".join(mismatches)
        )


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
            if any(token in name for token in ("tire", "hub_shell", "live_axle", "axle_inner_spacer", "axle_left_retaining_nut", "axle_right_retaining_nut", "6002_hub_bearing", "wheel_encoder_magnet_ring")):
                return f"{corner}_wheel"
            if any(token in name for token in ("fork", "wheel_drive", "planetary", "service_disc", "antirollback_dog", "wheel_encoder_pickup", "wheel_encoder_sensor")):
                return f"{corner}_wheel_module"
            if any(token in name for token in ("ballnut", "moving_slider", "slider_", "bellcrank", "pushrod", "rod_end", "clevis_m10", "jam_nut", "linear_scale_read_head")):
                return f"{corner}_slider"
            return "chassis"
    return "chassis"


def link_frame_origins_mm(g: dict) -> dict[str, tuple[float, float, float]]:
    """Absolute CAD position of every URDF link frame, in millimetres.

    A URDF link's mesh is drawn relative to that link's own frame, so geometry
    exported in absolute assembly coordinates gets displaced twice: once by the
    joint chain and again by the coordinates baked into the mesh. That is
    exactly what was happening -- front_left_wheel's mesh spans X 140..580 (its
    absolute position) while its link frame already sat at (360, 310, -445),
    so the wheel rendered at roughly double the track and wheelbase, below
    ground. Every link was wrong the same way, which is why the vehicle still
    settled upright and only the resting-height check caught it.

    These origins are read off the real CAD: the wheel frame is the wheel
    centre, and the pod frame is seat_roll_cross_shaft, the actual pitch pivot
    at z = 645 (the URDF previously assumed 260).
    """
    wb = g["wheelbase_mm"]
    tr = g["track_mm"]
    radius = g["wheel_diameter_mm"] / 2.0
    origins: dict[str, tuple[float, float, float]] = {
        "chassis": (0.0, 0.0, 0.0),
        "occupant_pod": (0.0, 0.0, 645.0),
    }
    for name, sx, sy in (("front_left", 1, 1), ("front_right", 1, -1),
                         ("rear_left", -1, 1), ("rear_right", -1, -1)):
        x = sx * wb / 2.0
        y = sy * tr / 2.0
        tower_x = x - sx * 100.0
        origins[f"{name}_slider"] = (tower_x, y, 455.0)
        origins[f"{name}_wheel_module"] = (x, y, radius)
        origins[f"{name}_wheel"] = (x, y, radius)
    return origins


def _export_cad_link_meshes(mesh_dir: Path, params: dict,
                            origins: dict[str, tuple[float, float, float]]) -> dict[str, int]:
    groups: dict[str, list[Component]] = {}
    for component in build_components(params, include_reference=False):
        link = _component_link(component)
        if link == "reference":
            continue
        groups.setdefault(link, []).append(component)

    component_counts: dict[str, int] = {}
    for link_name, components in groups.items():
        compound = cq.Compound.makeCompound([component.shape.val() for component in components])
        # Move the geometry into its own link frame so the joint chain is the
        # only thing positioning it.
        ox, oy, oz = origins[link_name]
        compound = compound.translate((-ox, -oy, -oz))
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
    load_cases = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    g = params["geometry"]
    actuator = load_cases["actuator_assumptions"]

    out_dir = ROOT / "sim_out"
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    # Wheel inertia radius/length are GEOMETRY, not mass properties, so they are
    # derived from parameters.json rather than read from mass_properties.json.
    # They were previously duplicated there and silently went stale when the
    # wheel grew 280 -> 430 mm: the URDF kept shipping 0.14 m wheel inertia
    # while the joint origins moved to the new 0.215 m radius. Inertia scales
    # with r^2, so that understated wheel rotational inertia by about 2.4x --
    # invisible in every kinematic screen, but corrupting to any dynamic
    # contact result. Deriving it here removes the second source of truth;
    # _check_derived_wheel_geometry below fails loudly if the data file still
    # disagrees, so a stale value cannot pass quietly again.
    wheel_inertia_radius_m = g["wheel_diameter_mm"] / 2000.0
    wheel_inertia_length_m = g["wheel_width_mm"] / 1000.0
    _check_derived_wheel_geometry(mass_props, wheel_inertia_radius_m, wheel_inertia_length_m)

    origins_mm = link_frame_origins_mm(g)
    link_component_counts = _export_cad_link_meshes(mesh_dir, params, origins_mm)
    _make_stair_mesh(mesh_dir / "reference_stairs_203r_279g.stl", g["stair_rise_reference_mm"], g["stair_going_reference_mm"])

    robot = ET.Element("robot", name="terrain_elevate_p1_v059")
    for link_name, props in mass_props["links"].items():
        link = ET.SubElement(robot, "link", name=link_name)
        mass = props["mass_kg"]
        if "box_inertia_size_m" in props:
            inertia = _inertia_box(mass, *props["box_inertia_size_m"])
        else:
            inertia = _inertia_wheel(mass, wheel_inertia_radius_m, wheel_inertia_length_m)
        _add_inertial(link, mass, inertia)
        _add_mesh(link, "visual", f"{link_name}.stl")
        _add_mesh(link, "collision", f"{link_name}.stl")

    wheelbase_m = g["wheelbase_mm"] / 1000
    track_m = g["track_mm"] / 1000
    radius_m = g["wheel_diameter_mm"] / 2000
    stroke_m = g["corner_stroke_selected_mm_SRC"] / 1000
    pitch_limit = math.radians(g["pod_pitch_correction_deg_CALC"] + 5.0)
    screw_lead_m = g["ball_screw_lead_mm_SRC"] / 1000
    motor_torque_nm = actuator["motor_nominal_torque_Nm_SRC"]
    gear_ratio = actuator["gear_ratio_ASSUMED"]
    gear_eff = actuator["gearbox_efficiency_ASSUMED"]
    screw_eff = actuator["screw_efficiency_ASSUMED"]
    actuator_effort_n = 2 * math.pi * screw_eff * motor_torque_nm * gear_ratio * gear_eff / screw_lead_m
    actuator_velocity_m_s = actuator["motor_nominal_speed_rpm_SRC"] / gear_ratio * screw_lead_m / 60
    wheel_effort_nm = 60.0

    # Joint origins are DIFFERENCES between link frames, derived from the same
    # table the meshes are translated by, so the kinematic chain and the
    # geometry can no longer disagree. They were previously hand-picked
    # offsets that did not correspond to the CAD at all -- the pod pitch axis
    # was at 260 mm when the real roll shaft it pivots on sits at 645.
    def _delta_m(child: str, parent: str) -> tuple[float, float, float]:
        c, p = origins_mm[child], origins_mm[parent]
        return ((c[0] - p[0]) / 1000.0, (c[1] - p[1]) / 1000.0, (c[2] - p[2]) / 1000.0)

    _joint(robot, "pod_pitch_leveling_joint", "revolute", "chassis", "occupant_pod", _delta_m("occupant_pod", "chassis"), (0, 1, 0), {"lower": -pitch_limit, "upper": pitch_limit, "effort": 2000, "velocity": 0.35})

    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        slider = f"{name}_slider"
        module = f"{name}_wheel_module"
        wheel = f"{name}_wheel"
        _joint(robot, f"{name}_corner_prismatic_actuator", "prismatic", "chassis", slider, _delta_m(slider, "chassis"), (0, 0, 1), {"lower": 0, "upper": stroke_m, "effort": actuator_effort_n, "velocity": actuator_velocity_m_s})
        _joint(robot, f"{name}_fork_carrier_fixed", "fixed", slider, module, _delta_m(module, slider), (0, 0, 1))
        _joint(robot, f"{name}_wheel_drive_joint", "continuous", module, wheel, _delta_m(wheel, module), (0, 1, 0), {"effort": wheel_effort_nm, "velocity": 35.0})

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    urdf_path = out_dir / "Terrain_Elevate_P1_V0_59_sim.urdf"
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)

    joint_graph = {
        "truth_boundary": mass_props["truth_boundary"],
        "format": "URDF plus STL collision/visual meshes for mechanism simulation import",
        "mesh_source": "CAD-derived grouped meshes from the detailed CadQuery assembly; not primitive placeholder boxes.",
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
            "corner_prismatic_velocity_m_s": actuator_velocity_m_s,
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
