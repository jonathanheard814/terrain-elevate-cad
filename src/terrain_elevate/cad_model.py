from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
from typing import Iterable

import cadquery as cq
from cadquery import exporters, importers
import ezdxf


@dataclass(frozen=True)
class Component:
    name: str
    role: str
    callout: str
    shape: cq.Workplane


def load_parameters(path: Path) -> dict:
    return json.loads(path.read_text())


def _box(name: str, role: str, callout: str, center, size) -> Component:
    solid = cq.Workplane("XY").box(*size).translate(center)
    return Component(name, role, callout, solid)


def _cylinder(name: str, role: str, callout: str, center, radius: float, length: float, axis: str) -> Component:
    solid = cq.Workplane("XY").cylinder(length, radius)
    if axis == "X":
        solid = solid.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        solid = solid.rotate((0, 0, 0), (1, 0, 0), 90)
    elif axis != "Z":
        raise ValueError(f"Unsupported cylinder axis {axis!r}")
    return Component(name, role, callout, solid.translate(center))


def _tube(name: str, role: str, callout: str, center, outer_radius: float, inner_radius: float, length: float, axis: str) -> Component:
    solid = cq.Workplane("XY").cylinder(length, outer_radius).cut(cq.Workplane("XY").cylinder(length + 2, inner_radius))
    if axis == "X":
        solid = solid.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        solid = solid.rotate((0, 0, 0), (1, 0, 0), 90)
    elif axis != "Z":
        raise ValueError(f"Unsupported tube axis {axis!r}")
    return Component(name, role, callout, solid.translate(center))


def _gusset(name: str, role: str, callout: str, center, length: float, height: float, thickness: float, flip_x: int) -> Component:
    points = [
        (-length / 2, -height / 2),
        (length / 2, -height / 2),
        (length / 2 * flip_x, height / 2),
    ]
    solid = cq.Workplane("XZ").polyline(points).close().extrude(thickness, both=True).translate(center)
    return Component(name, role, callout, solid)


def _add_chassis(components: list[Component], g: dict) -> None:
    wb = g["wheelbase_mm"]
    tr = g["track_mm"]
    z = g["frame_z_mm"]
    rail_len = wb + 260
    outer_y = tr / 2 + 55

    components.extend(
        [
            _tube("left_lower_frame_rail", "6061 rectangular tube main side rail", "1-12", (0, outer_y, z), 26, 19, rail_len, "X"),
            _tube("right_lower_frame_rail", "6061 rectangular tube main side rail", "1-12", (0, -outer_y, z), 26, 19, rail_len, "X"),
            _tube("front_crossmember", "front torsion crossmember", "1-12", (wb / 2 + 55, 0, z), 28, 20, tr + 210, "Y"),
            _tube("rear_crossmember", "rear torsion crossmember", "1-12", (-wb / 2 - 55, 0, z), 28, 20, tr + 210, "Y"),
            _box("battery_tray_pan", "low mounted battery tray between rails", "41-59", (-45, 0, z - 55), (360, 220, 28)),
            _box("front_sensor_bridge", "front lidar and edge sensor mounting bridge", "67", (wb / 2 + 80, 0, z + 175), (34, 580, 42)),
            _box("rear_sensor_bridge", "rear lidar and edge sensor mounting bridge", "67", (-wb / 2 - 80, 0, z + 175), (34, 580, 42)),
            _cylinder("fold_hinge_axis_left", "folding handle and chassis hinge datum", "1-12", (-120, outer_y, z + 80), 16, 145, "Y"),
            _cylinder("fold_hinge_axis_right", "folding handle and chassis hinge datum", "1-12", (-120, -outer_y, z + 80), 16, 145, "Y"),
        ]
    )


def _add_seat_pod(components: list[Component]) -> None:
    components.extend(
        [
            _tube("seat_roll_outer_front", "roll leveling outer frame tube", "13-26", (190, 0, 645), 22, 15, 470, "Y"),
            _tube("seat_roll_outer_rear", "roll leveling outer frame tube", "13-26", (-190, 0, 645), 22, 15, 470, "Y"),
            _tube("seat_roll_outer_left", "roll leveling outer frame side tube", "13-26", (0, 230, 645), 22, 15, 420, "X"),
            _tube("seat_roll_outer_right", "roll leveling outer frame side tube", "13-26", (0, -230, 645), 22, 15, 420, "X"),
            _box("seat_pan_shell", "child seat pan envelope with reclined base", "13-26", (30, 0, 725), (390, 330, 52)),
            _box("seat_back_shell", "child back support envelope", "13-26", (-165, 0, 850), (72, 390, 320)),
            _box("seat_left_side_retainer", "child side retention wall", "13-26", (25, 198, 835), (360, 38, 215)),
            _box("seat_right_side_retainer", "child side retention wall", "13-26", (25, -198, 835), (360, 38, 215)),
            _cylinder("seat_roll_bearing_left", "seat roll bearing envelope", "13-26", (0, 276, 645), 27, 56, "Y"),
            _cylinder("seat_roll_bearing_right", "seat roll bearing envelope", "13-26", (0, -276, 645), 27, 56, "Y"),
            _cylinder("seat_pitch_bearing_front", "seat pitch bearing envelope", "13-26", (222, 0, 710), 22, 68, "X"),
            _cylinder("seat_pitch_bearing_rear", "seat pitch bearing envelope", "13-26", (-222, 0, 710), 22, 68, "X"),
            _box("mechanical_level_lock", "no power positive seat leveling lock", "13-26", (-240, 0, 665), (120, 70, 44)),
            _box("five_point_harness_bar", "harness anchor crossbar", "13-26", (75, 0, 845), (255, 30, 34)),
        ]
    )


def _add_controls(components: list[Component]) -> None:
    components.extend(
        [
            _box("battery_pack_48v", "48 V battery envelope", "41-59", (-70, 0, 430), (310, 165, 118)),
            _box("bms_fuse_contactor", "BMS fuse contactor and service disconnect", "41-59", (175, 0, 430), (220, 112, 54)),
            _box("safety_supervisor_ecu", "independent safety supervisor ECU", "68", (45, 220, 530), (140, 100, 42)),
            _box("motor_control_ecu", "motor control electronics bank", "68", (45, -220, 530), (145, 108, 58)),
            _box("left_inverter_bank", "left branch inverter package", "69", (-230, 185, 500), (150, 82, 46)),
            _box("right_inverter_bank", "right branch inverter package", "69", (-230, -185, 500), (150, 82, 46)),
        ]
    )


def _add_handle_and_sensors(components: list[Component], g: dict) -> None:
    wb = g["wheelbase_mm"]
    components.extend(
        [
            _tube("handle_left_upright", "folding handle upright tube", "31-36", (-550, 285, 855), 17, 11, 610, "Z"),
            _tube("handle_right_upright", "folding handle upright tube", "31-36", (-550, -285, 855), 17, 11, 610, "Z"),
            _tube("handle_grip_deadman_bar", "operator grip with deadman release", "31-36", (-550, 0, 1160), 20, 13, 620, "Y"),
            _box("deadman_release_paddle", "deadman release paddle", "31-36", (-505, 0, 1210), (120, 14, 58)),
            _box("front_depth_lidar", "front stair depth sensor", "67", (wb / 2 + 190, 0, 675), (126, 56, 50)),
            _box("rear_depth_lidar", "rear stair depth sensor", "67", (-wb / 2 - 190, 0, 675), (126, 56, 50)),
            _box("dual_imu_mount", "redundant attitude sensor mount", "67", (0, 0, 586), (72, 44, 24)),
        ]
    )


def _add_corner(components: list[Component], g: dict, code: str, sx: int, sy: int) -> None:
    wb = g["wheelbase_mm"]
    tr = g["track_mm"]
    wheel_d = g["wheel_diameter_mm"]
    wheel_w = g["wheel_width_mm"]
    x = sx * wb / 2
    y = sy * tr / 2
    z_axle = wheel_d / 2
    tower_x = x - sx * 100
    side = "front" if sx > 0 else "rear"
    lr = "left" if sy > 0 else "right"
    prefix = f"{code.lower()}_"

    components.extend(
        [
            _tube(prefix + "tire_280x75", f"{side} {lr} pneumatic tire envelope", "60", (x, y, z_axle), wheel_d / 2, wheel_d / 2 - 24, wheel_w, "Y"),
            _tube(prefix + "hub_shell", f"{side} {lr} wheel hub shell", "60", (x, y, z_axle), 64, 22, wheel_w + 24, "Y"),
            _cylinder(prefix + "live_axle", f"{side} {lr} live axle", "60", (x, y, z_axle), 8.5, 138, "Y"),
            _box(prefix + "fork_outer_plate", f"{side} {lr} fork outer plate", "60", (x, y + sy * 58, z_axle + 126), (118, 16, 205)),
            _box(prefix + "fork_inner_plate", f"{side} {lr} fork inner plate", "60", (x, y - sy * 58, z_axle + 126), (118, 16, 205)),
            _box(prefix + "fork_bridge", f"{side} {lr} fork bridge", "60", (x, y, z_axle + 230), (132, 132, 34)),
            _cylinder(prefix + "steer_kingpin_bearing", f"{side} {lr} steering kingpin bearing stack", "60", (x, y, 445), 22, 270, "Z"),
            _box(prefix + "steer_center_lock", f"{side} {lr} steering positive center lock", "66", (x, y, 590), (92, 68, 34)),
            _box(prefix + "suspension_tower_cassette", f"{side} {lr} vertical suspension cassette", "61", (tower_x, y, 510), (92, 82, 430)),
            _cylinder(prefix + "swingarm_pivot_25mm", f"{side} {lr} 25 mm swingarm pivot shaft", "60", (x - sx * 108, y, 314), g["pivot_diameter_mm_OPEN"] / 2, 176, "Y"),
            _box(prefix + "boxed_lower_swingarm", f"{side} {lr} boxed lower swing arm", "60", (x - sx * 72, y, 312), (238, 42, 36)),
            _box(prefix + "upper_reaction_link", f"{side} {lr} upper reaction link", "60", (x - sx * 70, y, 455), (210, 32, 30)),
            _cylinder(prefix + "passive_spring_damper", f"{side} {lr} passive spring damper envelope", "61", (tower_x + sx * 42, y, 510), 18, 280, "Z"),
            _cylinder(prefix + "bnk1404_ball_screw", f"{side} {lr} THK BNK1404 ball screw envelope", "62", (tower_x, y, 512), g["ball_screw_diameter_mm_SRC"] / 2, 392, "Z"),
            _box(prefix + "ballnut_carriage_bridge", f"{side} {lr} ballnut carriage bridge", "62", (tower_x, y, 472), (82, 104, 54)),
            _box(prefix + "linear_guide_rail_a", f"{side} {lr} linear guide rail A", "63", (tower_x - 24, y + sy * 31, 508), (16, 12, 362)),
            _box(prefix + "linear_guide_rail_b", f"{side} {lr} linear guide rail B", "63", (tower_x + 24, y - sy * 31, 508), (16, 12, 362)),
            _box(prefix + "linear_guide_block_a", f"{side} {lr} linear guide carriage block A", "63", (tower_x - 24, y + sy * 31, 472), (36, 58, 30)),
            _box(prefix + "linear_guide_block_b", f"{side} {lr} linear guide carriage block B", "63", (tower_x + 24, y - sy * 31, 472), (36, 58, 30)),
            _cylinder(prefix + "actuator_motor_eci40", f"{side} {lr} Maxon EC-i 40 actuator motor envelope", "64", (tower_x, y + sy * 104, 728), 20, 82, "Y"),
            _cylinder(prefix + "gpx42_gearhead", f"{side} {lr} GPX42 12:1 gearhead envelope", "65", (tower_x, y + sy * 50, 728), 21, 58, "Y"),
            _cylinder(prefix + "ab28_power_off_brake", f"{side} {lr} AB28 power off brake envelope", "66", (tower_x, y + sy * 150, 728), 18, 32, "Y"),
            _box(prefix + "anti_drop_rack", f"{side} {lr} anti drop rack", "66", (tower_x - sx * 68, y, 510), (20, 24, 332)),
            _box(prefix + "anti_drop_pawl", f"{side} {lr} primary anti drop pawl", "66", (tower_x - sx * 46, y + sy * 34, 390), (42, 18, 92)),
            _cylinder(prefix + "wheel_drive_motor", f"{side} {lr} coaxial wheel drive motor", "69", (x, y - sy * 94, z_axle), 36, 82, "Y"),
            _gusset(prefix + "tower_frame_gusset", f"{side} {lr} triangular tower to rail gusset", "61", (tower_x + sx * 42, y, 375), 165, 160, 8, sx),
        ]
    )

    if sx < 0:
        components.extend(
            [
                _cylinder(prefix + "antirollback_dog_lock", f"{side} {lr} rear anti rollback dog lock", "66/70", (x, y + sy * 96, z_axle), 42, 26, "Y"),
                _box(prefix + "rear_module_mount_bracket", f"{side} {lr} rear module mounting bracket", "70", (x - sx * 122, y, 332), (30, 194, 184)),
            ]
        )
    else:
        components.append(_cylinder(prefix + "front_service_disc_brake", f"{side} {lr} front service brake", "60", (x, y + sy * 96, z_axle), 42, 26, "Y"))


def _add_reference_geometry(components: list[Component], g: dict) -> None:
    run = g["stair_going_reference_mm"]
    rise = g["stair_rise_reference_mm"]
    components.extend(
        [
            _box("ref_clear_stair_width_864", "864 mm reference stair clear width", "REQ", (0, 0, -20), (1160, 864, 8)),
            _box("ref_outer_width_gate_750", "750 mm stroller width limit reference", "REQ", (0, 0, -8), (1160, g["overall_width_gate_mm"], 5)),
        ]
    )
    for i in range(5):
        components.append(
            _box(
                f"ref_stair_{i + 1:02d}_203r_279g",
                "reference stair tread and riser block",
                "REQ",
                (450 + i * run, 0, i * rise / 2),
                (run, 864, (i + 1) * rise),
            )
        )


def build_components(params: dict) -> list[Component]:
    g = params["geometry"]
    components: list[Component] = []
    _add_chassis(components, g)
    _add_seat_pod(components)
    _add_controls(components)
    _add_handle_and_sensors(components, g)
    for code, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1)):
        _add_corner(components, g, code, sx, sy)
    _add_reference_geometry(components, g)
    return components


def _compound(components: Iterable[Component]) -> cq.Compound:
    return cq.Compound.makeCompound([component.shape.val() for component in components])


def _component_summary(component: Component) -> dict:
    solid = component.shape.val()
    bb = solid.BoundingBox()
    return {
        "name": component.name,
        "role": component.role,
        "callout": component.callout,
        "volume_mm3": solid.Volume(),
        "bounding_box_mm": [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax],
    }


def export_dxf(path: Path, g: dict) -> None:
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    wheelbase = g["wheelbase_mm"]
    track = g["track_mm"]
    width_gate = g["overall_width_gate_mm"]
    wheel_radius = g["wheel_diameter_mm"] / 2

    doc.layers.add("PACKAGE", color=2)
    doc.layers.add("WHEEL_CENTERS", color=1)
    doc.layers.add("FRAME", color=3)
    doc.layers.add("STAIR_REF", color=5)

    msp.add_lwpolyline(
        [
            (-wheelbase / 2 - 180, -width_gate / 2),
            (wheelbase / 2 + 180, -width_gate / 2),
            (wheelbase / 2 + 180, width_gate / 2),
            (-wheelbase / 2 - 180, width_gate / 2),
            (-wheelbase / 2 - 180, -width_gate / 2),
        ],
        dxfattribs={"layer": "PACKAGE"},
    )
    msp.add_lwpolyline(
        [
            (-wheelbase / 2 - 70, -track / 2 - 55),
            (wheelbase / 2 + 70, -track / 2 - 55),
            (wheelbase / 2 + 70, track / 2 + 55),
            (-wheelbase / 2 - 70, track / 2 + 55),
            (-wheelbase / 2 - 70, -track / 2 - 55),
        ],
        dxfattribs={"layer": "FRAME"},
    )
    for sx in (-1, 1):
        for sy in (-1, 1):
            msp.add_circle((sx * wheelbase / 2, sy * track / 2), wheel_radius, dxfattribs={"layer": "WHEEL_CENTERS"})

    rise = g["stair_rise_reference_mm"]
    going = g["stair_going_reference_mm"]
    x = wheelbase / 2 + 170
    z = -480
    for i in range(5):
        msp.add_lwpolyline(
            [(x + i * going, z), (x + (i + 1) * going, z), (x + (i + 1) * going, z + rise), (x + (i + 1) * going, z)],
            dxfattribs={"layer": "STAIR_REF"},
        )
        z += rise

    doc.saveas(path)


def export_model(params: dict, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    components = build_components(params)
    assembly = _compound(components)

    step_path = out_dir / "Terrain_Elevate_P1_V0_59_OCCT.step"
    stl_path = out_dir / "Terrain_Elevate_P1_V0_59_OCCT.stl"
    dxf_path = out_dir / "Terrain_Elevate_P1_V0_59_package.dxf"
    manifest_path = out_dir / "Terrain_Elevate_P1_V0_59_manifest.json"

    exporters.export(assembly, str(step_path), exportType="STEP")
    exporters.export(assembly, str(stl_path), exportType="STL", tolerance=0.25, angularTolerance=0.2)
    export_dxf(dxf_path, params["geometry"])

    imported = importers.importStep(str(step_path))
    imported_solid = imported.val()
    bb = imported_solid.BoundingBox()
    manifest = {
        "project": params["project"],
        "version": params["version"],
        "engine": "CadQuery 2.8 / OCCT 7.9",
        "truth_boundary": params["locked_constraints"]["truth_boundary"],
        "outputs": {
            "step_file": step_path.name,
            "stl_file": stl_path.name,
            "dxf_file": dxf_path.name,
            "manifest_file": manifest_path.name,
        },
        "output_bytes": {
            "step": step_path.stat().st_size,
            "stl": stl_path.stat().st_size,
            "dxf": dxf_path.stat().st_size,
        },
        "locked_constraints": params["locked_constraints"],
        "geometry": params["geometry"],
        "component_count": len(components),
        "component_volume_sum_mm3": sum(component.shape.val().Volume() for component in components),
        "reimported_step_volume_mm3": imported_solid.Volume(),
        "reimported_step_bounding_box_mm": [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax],
        "components": [_component_summary(component) for component in components],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2))
    return manifest


def manifest_as_jsonable(manifest: dict) -> dict:
    return json.loads(json.dumps(manifest))

