from __future__ import annotations

from dataclasses import dataclass
import json
import math
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
    material: str = "aluminum"


MATERIAL_COLORS = {
    "aluminum": cq.Color(0.72, 0.72, 0.70, 1.0),
    "dark_aluminum": cq.Color(0.28, 0.30, 0.32, 1.0),
    "steel": cq.Color(0.58, 0.60, 0.62, 1.0),
    "black_oxide_steel": cq.Color(0.06, 0.06, 0.065, 1.0),
    "rubber": cq.Color(0.015, 0.015, 0.018, 1.0),
    "plastic": cq.Color(0.10, 0.12, 0.16, 1.0),
    "electronics": cq.Color(0.04, 0.22, 0.13, 1.0),
    "copper": cq.Color(0.76, 0.36, 0.16, 1.0),
    "seat_shell": cq.Color(0.18, 0.22, 0.28, 1.0),
    "reference": cq.Color(0.68, 0.62, 0.50, 0.45),
}


def _infer_material(name: str, role: str, callout: str, material: str | None = None) -> str:
    if material:
        return material
    text = f"{name} {role} {callout}".lower()
    if "tire" in text:
        return "rubber"
    if any(token in text for token in ("bolt", "washer", "screw", "shaft", "pin", "rack", "pawl", "bearing", "rail")):
        return "steel"
    if any(token in text for token in ("motor", "gearhead", "brake", "ecu", "bms", "battery", "sensor", "lidar", "imu", "inverter")):
        return "electronics"
    if any(token in text for token in ("seat", "pod", "retainer", "shell", "paddle")):
        return "seat_shell"
    if "ref_" in name or callout == "REQ":
        return "reference"
    if "hub" in text or "tube" in text:
        return "dark_aluminum"
    return "aluminum"


def load_parameters(path: Path) -> dict:
    return json.loads(path.read_text())


def _box(name: str, role: str, callout: str, center, size, material: str | None = None) -> Component:
    solid = cq.Workplane("XY").box(*size).translate(center)
    return Component(name, role, callout, solid, _infer_material(name, role, callout, material))


def _soft_box(name: str, role: str, callout: str, center, size, radius: float = 8.0, rotate_y: float = 0.0, material: str | None = None) -> Component:
    solid = cq.Workplane("XY").box(*size)
    try:
        solid = solid.edges().fillet(radius)
    except Exception:
        pass
    if rotate_y:
        solid = solid.rotate((0, 0, 0), (0, 1, 0), rotate_y)
    return Component(name, role, callout, solid.translate(center), _infer_material(name, role, callout, material))


def _cylinder(name: str, role: str, callout: str, center, radius: float, length: float, axis: str, material: str | None = None) -> Component:
    solid = cq.Workplane("XY").cylinder(length, radius)
    if axis == "X":
        solid = solid.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        solid = solid.rotate((0, 0, 0), (1, 0, 0), 90)
    elif axis != "Z":
        raise ValueError(f"Unsupported cylinder axis {axis!r}")
    return Component(name, role, callout, solid.translate(center), _infer_material(name, role, callout, material))


def _tube(name: str, role: str, callout: str, center, outer_radius: float, inner_radius: float, length: float, axis: str, material: str | None = None) -> Component:
    solid = cq.Workplane("XY").cylinder(length, outer_radius).cut(cq.Workplane("XY").cylinder(length + 2, inner_radius))
    if axis == "X":
        solid = solid.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        solid = solid.rotate((0, 0, 0), (1, 0, 0), 90)
    elif axis != "Z":
        raise ValueError(f"Unsupported tube axis {axis!r}")
    return Component(name, role, callout, solid.translate(center), _infer_material(name, role, callout, material))


def _gusset(name: str, role: str, callout: str, center, length: float, height: float, thickness: float, flip_x: int, material: str | None = None) -> Component:
    points = [
        (-length / 2, -height / 2),
        (length / 2, -height / 2),
        (length / 2 * flip_x, height / 2),
    ]
    solid = cq.Workplane("XZ").polyline(points).close().extrude(thickness, both=True).translate(center)
    return Component(name, role, callout, solid, _infer_material(name, role, callout, material))


def _rod_xz(name: str, role: str, callout: str, start, end, radius: float, material: str | None = "steel") -> Component:
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = (dx * dx + dz * dz) ** 0.5
    angle_y = 90.0 - math.degrees(math.atan2(dz, dx))
    center = ((sx + ex) / 2, (sy + ey) / 2, (sz + ez) / 2)
    solid = cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (0, 1, 0), angle_y).translate(center)
    return Component(name, role, callout, solid, _infer_material(name, role, callout, material))


def _bolt_y(prefix: str, center, grip: float, diameter: float = 8.0) -> list[Component]:
    cx, cy, cz = center
    shank_r = diameter / 2
    head_r = diameter * 0.8125
    head_len = diameter
    washer_r = diameter * 0.95
    return [
        _cylinder(prefix + "_shank", "socket-head cap screw shank", "FASTENER", center, shank_r, grip, "Y", "black_oxide_steel"),
        _cylinder(prefix + "_head", "ISO 4762 socket-head cap screw head", "FASTENER", (cx, cy + grip / 2 + head_len / 2, cz), head_r, head_len, "Y", "black_oxide_steel"),
        _cylinder(prefix + "_washer", "flat washer", "FASTENER", (cx, cy - grip / 2 - 1.0, cz), washer_r, 2.0, "Y", "steel"),
    ]


def _bolt_x(prefix: str, center, grip: float, diameter: float = 8.0) -> list[Component]:
    cx, cy, cz = center
    shank_r = diameter / 2
    head_r = diameter * 0.8125
    head_len = diameter
    washer_r = diameter * 0.95
    return [
        _cylinder(prefix + "_shank", "socket-head cap screw shank", "FASTENER", center, shank_r, grip, "X", "black_oxide_steel"),
        _cylinder(prefix + "_head", "ISO 4762 socket-head cap screw head", "FASTENER", (cx + grip / 2 + head_len / 2, cy, cz), head_r, head_len, "X", "black_oxide_steel"),
        _cylinder(prefix + "_washer", "flat washer", "FASTENER", (cx - grip / 2 - 1.0, cy, cz), washer_r, 2.0, "X", "steel"),
    ]


def _assembly_from_components(components: Iterable[Component]) -> cq.Assembly:
    assembly = cq.Assembly(name="Terrain_Elevate_P1_V0_59")
    for component in components:
        color = MATERIAL_COLORS.get(component.material, MATERIAL_COLORS["aluminum"])
        assembly.add(component.shape, name=component.name, color=color)
    return assembly


def _rotated_y(component: Component, angle_deg: float, origin=(0.0, 0.0, 0.0), suffix: str = "") -> Component:
    shape = component.shape.rotate(origin, (origin[0], origin[1] + 1.0, origin[2]), angle_deg)
    return Component(component.name + suffix, component.role, component.callout, shape, component.material)


def _category_for(component: Component) -> str:
    text = f"{component.name} {component.role}".lower()
    if any(token in text for token in ("bolt", "washer", "screw shank", "fastener")):
        return "fastener"
    if any(token in text for token in ("seat", "pod", "five_point", "leveling")):
        return "occupant_pod"
    if any(token in text for token in ("harness", "connector", "bus", "cable", "inhibit", "watchdog")):
        return "electrical_interconnect"
    if any(token in text for token in ("tire", "wheel", "hub", "axle", "brake", "motor")):
        return "wheel_drive"
    if any(token in text for token in ("guide", "ball_screw", "stroke", "carriage", "slider", "tower", "swingarm", "link", "clevis", "pawl", "rack", "rod_end", "pushrod", "rocker")):
        return "suspension_corner"
    if any(token in text for token in ("battery", "ecu", "bms", "inverter", "sensor", "lidar", "imu")):
        return "controls_power_sensing"
    if "ref_" in component.name:
        return "reference"
    return "structure"


def _count_by(items: Iterable[str]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for item in items:
        counts[item] = counts.get(item, 0) + 1
    return dict(sorted(counts.items()))


def _is_occupant_pod_leveling_group(component: Component) -> bool:
    text = f"{component.name} {component.role}".lower()
    return any(
        token in text
        for token in (
            "seat_",
            "pod",
            "five_point",
            "harness",
            "mechanical_level_lock",
            "electrak",
            "leveling",
        )
    )


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
            _soft_box("battery_tray_pan", "low mounted battery tray between rails", "41-59", (-45, 0, z - 55), (360, 220, 28), 6),
            _soft_box("front_sensor_bridge", "front lidar and edge sensor mounting bridge", "67", (wb / 2 + 80, 0, z + 175), (34, 580, 42), 6),
            _soft_box("rear_sensor_bridge", "rear lidar and edge sensor mounting bridge", "67", (-wb / 2 - 80, 0, z + 175), (34, 580, 42), 6),
            _cylinder("fold_hinge_axis_left", "folding handle and chassis hinge shaft", "1-12", (-540, outer_y, z + 95), 16, 145, "Y", "steel"),
            _cylinder("fold_hinge_axis_right", "folding handle and chassis hinge shaft", "1-12", (-540, -outer_y, z + 95), 16, 145, "Y", "steel"),
            _soft_box("handle_left_lower_hinge_bracket", "bolted handle hinge bracket tied into left rail", "31-36", (-540, outer_y, z + 95), (120, 42, 84), 6, material="steel"),
            _soft_box("handle_right_lower_hinge_bracket", "bolted handle hinge bracket tied into right rail", "31-36", (-540, -outer_y, z + 95), (120, 42, 84), 6, material="steel"),
            _soft_box("handle_left_hinge_backstay", "left hinge backstay from handle bracket to chassis rail", "31-36", (-485, outer_y, z + 30), (150, 36, 92), 5, material="steel"),
            _soft_box("handle_right_hinge_backstay", "right hinge backstay from handle bracket to chassis rail", "31-36", (-485, -outer_y, z + 30), (150, 36, 92), 5, material="steel"),
            _soft_box("seat_left_roll_trunnion_stand", "seat roll trunnion stand tied to left chassis rail", "13-26", (0, 265, z + 165), (86, 58, 260), 7, material="steel"),
            _soft_box("seat_right_roll_trunnion_stand", "seat roll trunnion stand tied to right chassis rail", "13-26", (0, -265, z + 165), (86, 58, 260), 7, material="steel"),
            _soft_box("battery_front_hold_down_strap", "battery hold-down strap bolted to tray", "41-59", (75, 0, z + 15), (18, 250, 20), 4, material="steel"),
            _soft_box("battery_rear_hold_down_strap", "battery hold-down strap bolted to tray", "41-59", (-215, 0, z + 15), (18, 250, 20), 4, material="steel"),
            _soft_box("battery_tray_left_hanger", "left battery tray hanger bracket to chassis rail", "41-59", (-45, 225, z - 20), (335, 240, 78), 5, material="steel"),
            _soft_box("battery_tray_right_hanger", "right battery tray hanger bracket to chassis rail", "41-59", (-45, -225, z - 20), (335, 240, 78), 5, material="steel"),
            _soft_box("front_sensor_left_stanchion", "front sensor bridge left vertical stanchion", "67", (wb / 2 + 55, 250, z + 92), (34, 34, 200), 5, material="steel"),
            _soft_box("front_sensor_right_stanchion", "front sensor bridge right vertical stanchion", "67", (wb / 2 + 55, -250, z + 92), (34, 34, 200), 5, material="steel"),
            _soft_box("rear_sensor_left_stanchion", "rear sensor bridge left vertical stanchion", "67", (-wb / 2 - 55, 250, z + 92), (34, 34, 200), 5, material="steel"),
            _soft_box("rear_sensor_right_stanchion", "rear sensor bridge right vertical stanchion", "67", (-wb / 2 - 55, -250, z + 92), (34, 34, 200), 5, material="steel"),
        ]
    )
    for i, y_side in enumerate((outer_y, -outer_y)):
        components.extend(_bolt_y(f"handle_hinge_m8_{i}_front", (-560, y_side, z + 95), 86))
        components.extend(_bolt_y(f"handle_hinge_m8_{i}_rear", (-520, y_side, z + 95), 86))
    for i, y_side in enumerate((265, -265)):
        components.extend(_bolt_y(f"seat_trunnion_stand_m8_{i}_front", (-28, y_side, z + 80), 72))
        components.extend(_bolt_y(f"seat_trunnion_stand_m8_{i}_rear", (28, y_side, z + 80), 72))


def _add_seat_pod(components: list[Component]) -> None:
    components.extend(
        [
            _tube("seat_roll_outer_front", "roll leveling outer frame tube", "13-26", (190, 0, 645), 22, 15, 470, "Y"),
            _tube("seat_roll_outer_rear", "roll leveling outer frame tube", "13-26", (-190, 0, 645), 22, 15, 470, "Y"),
            _tube("seat_roll_outer_left", "roll leveling outer frame side tube", "13-26", (0, 230, 645), 22, 15, 420, "X"),
            _tube("seat_roll_outer_right", "roll leveling outer frame side tube", "13-26", (0, -230, 645), 22, 15, 420, "X"),
            _soft_box("seat_pan_shell", "rounded child seat pan envelope with reclined base", "13-26", (30, 0, 725), (390, 330, 52), 18, -6),
            _soft_box("seat_back_shell", "rounded child back support envelope", "13-26", (-165, 0, 850), (72, 390, 320), 20, -10),
            _soft_box("seat_left_side_retainer", "rounded child side retention wall", "13-26", (25, 198, 835), (360, 38, 215), 14, -6),
            _soft_box("seat_right_side_retainer", "rounded child side retention wall", "13-26", (25, -198, 835), (360, 38, 215), 14, -6),
            _cylinder("seat_roll_bearing_left", "seat roll bearing envelope", "13-26", (0, 276, 645), 27, 56, "Y"),
            _cylinder("seat_roll_bearing_right", "seat roll bearing envelope", "13-26", (0, -276, 645), 27, 56, "Y"),
            _cylinder("seat_roll_cross_shaft", "continuous roll shaft tying pod into chassis trunnions", "13-26", (0, 0, 645), 12.5, 610, "Y", "steel"),
            _cylinder("seat_pitch_bearing_front", "seat pitch bearing envelope", "13-26", (222, 0, 710), 22, 68, "X"),
            _cylinder("seat_pitch_bearing_rear", "seat pitch bearing envelope", "13-26", (-222, 0, 710), 22, 68, "X"),
            _cylinder("seat_left_electrak_md_body", "left Thomson Electrak MD pod pitch actuator body", "13-26", (-150, 150, 610), 34, 118, "X", "dark_aluminum"),
            _cylinder("seat_right_electrak_md_body", "right Thomson Electrak MD pod pitch actuator body", "13-26", (-150, -150, 610), 34, 118, "X", "dark_aluminum"),
            _rod_xz("seat_leveling_actuator_left_extension_tube", "left Electrak MD stainless extension tube between chassis and pod", "13-26", (-180, 150, 565), (-80, 150, 760), 12, "steel"),
            _rod_xz("seat_leveling_actuator_right_extension_tube", "right Electrak MD stainless extension tube between chassis and pod", "13-26", (-180, -150, 565), (-80, -150, 760), 12, "steel"),
            _cylinder("seat_left_leveling_clevis_lower_pin", "left pod leveling lower clevis pin", "13-26", (-180, 150, 565), 7, 70, "Y", "steel"),
            _cylinder("seat_left_leveling_clevis_upper_pin", "left pod leveling upper clevis pin", "13-26", (-80, 150, 760), 7, 70, "Y", "steel"),
            _cylinder("seat_right_leveling_clevis_lower_pin", "right pod leveling lower clevis pin", "13-26", (-180, -150, 565), 7, 70, "Y", "steel"),
            _cylinder("seat_right_leveling_clevis_upper_pin", "right pod leveling upper clevis pin", "13-26", (-80, -150, 760), 7, 70, "Y", "steel"),
            _soft_box("seat_pitch_stop_front", "front mechanical pod pitch stop", "13-26", (255, 0, 662), (42, 370, 34), 5, material="steel"),
            _soft_box("seat_pitch_stop_rear", "rear mechanical pod pitch stop", "13-26", (-255, 0, 662), (42, 370, 34), 5, material="steel"),
            _soft_box("seat_front_pitch_stop_mount", "front pitch stop mount tied to roll frame", "13-26", (225, 0, 645), (70, 390, 22), 4, material="steel"),
            _soft_box("seat_harness_left_anchor_strap", "left harness strap bracket tied to retainer wall", "13-26", (75, 118, 845), (34, 180, 22), 4, material="steel"),
            _soft_box("seat_harness_right_anchor_strap", "right harness strap bracket tied to retainer wall", "13-26", (75, -118, 845), (34, 180, 22), 4, material="steel"),
            _soft_box("seat_harness_bar_backing_plate", "harness bar backing plate tied into both anchor straps", "13-26", (75, 0, 845), (285, 250, 18), 4, material="steel"),
            _soft_box("mechanical_level_lock", "no power positive seat leveling lock", "13-26", (-240, 0, 665), (120, 70, 44), 6),
            _soft_box("five_point_harness_bar", "harness anchor crossbar", "13-26", (75, 0, 845), (255, 30, 34), 6),
        ]
    )
    for i, y_side in enumerate((276, -276)):
        components.extend(_bolt_y(f"seat_roll_bearing_retainer_m6_{i}_a", (-18, y_side, 645), 68, 6))
        components.extend(_bolt_y(f"seat_roll_bearing_retainer_m6_{i}_b", (18, y_side, 645), 68, 6))


def _add_controls(components: list[Component]) -> None:
    components.extend(
        [
            _soft_box("battery_pack_48v", "48 V battery envelope", "41-59", (-70, 0, 430), (310, 165, 118), 10),
            _soft_box("bms_fuse_contactor", "BMS fuse contactor and service disconnect", "41-59", (175, 0, 430), (220, 112, 54), 8),
            _soft_box("safety_supervisor_ecu", "independent safety supervisor ECU", "68", (45, 220, 530), (140, 100, 42), 6),
            _soft_box("motor_control_ecu", "motor control electronics bank", "68", (45, -220, 530), (145, 108, 58), 6),
            _soft_box("left_inverter_bank", "left branch inverter package", "69", (-230, 185, 500), (150, 82, 46), 6),
            _soft_box("right_inverter_bank", "right branch inverter package", "69", (-230, -185, 500), (150, 82, 46), 6),
            _soft_box("hardwired_inhibit_relay_module", "hardwired gate-enable inhibit relay module", "68", (210, 145, 500), (90, 62, 36), 5, material="electronics"),
            _soft_box("external_watchdog_module", "independent external watchdog module", "68", (210, -145, 500), (90, 62, 36), 5, material="electronics"),
            _soft_box("safety_modules_mounting_plate", "watchdog and hardwired inhibit mounting plate tied to electronics bank", "68", (170, 0, 500), (125, 340, 16), 5, material="steel"),
            _soft_box("left_littelfuse_midi498_fuse_block", "Littelfuse MIDI 498 left branch fuse holder", "69", (-55, 240, 462), (115, 44, 34), 5, material="electronics"),
            _soft_box("right_littelfuse_midi498_fuse_block", "Littelfuse MIDI 498 right branch fuse holder", "69", (-55, -240, 462), (115, 44, 34), 5, material="electronics"),
            _cylinder("positive_dc_bus_bar", "48 V positive copper bus bar", "41-59", (0, 72, 488), 4, 390, "X", "copper"),
            _cylinder("negative_dc_bus_bar", "48 V negative copper bus bar", "41-59", (0, -72, 488), 4, 390, "X", "copper"),
            _cylinder("safety_inhibit_harness_left", "hardwired inhibit harness to left drive branch", "68", (-120, 210, 545), 5, 260, "X", "plastic"),
            _cylinder("safety_inhibit_harness_right", "hardwired inhibit harness to right drive branch", "68", (-120, -210, 545), 5, 260, "X", "plastic"),
        ]
    )
    for i, y_side in enumerate((185, -185, 240, -240)):
        components.extend(_bolt_y(f"electronics_mount_m5_{i}_a", (-280, y_side, 528), 38, 5))
        components.extend(_bolt_y(f"electronics_mount_m5_{i}_b", (-180, y_side, 528), 38, 5))


def _add_handle_and_sensors(components: list[Component], g: dict) -> None:
    wb = g["wheelbase_mm"]
    components.extend(
        [
            _tube("handle_left_upright", "folding handle upright tube", "31-36", (-550, 285, 855), 17, 11, 610, "Z"),
            _tube("handle_right_upright", "folding handle upright tube", "31-36", (-550, -285, 855), 17, 11, 610, "Z"),
            _tube("handle_grip_deadman_bar", "operator grip with deadman release", "31-36", (-550, 0, 1160), 20, 13, 620, "Y"),
            _soft_box("deadman_release_paddle", "deadman release paddle", "31-36", (-505, 0, 1210), (120, 14, 58), 5),
            _soft_box("handle_left_lower_yoke_link", "left lower handle yoke linking upright to hinge shaft", "31-36", (-545, 325, 665), (48, 124, 390), 5, material="steel"),
            _soft_box("handle_right_lower_yoke_link", "right lower handle yoke linking upright to hinge shaft", "31-36", (-545, -325, 665), (48, 124, 390), 5, material="steel"),
            _soft_box("deadman_paddle_hinge_tab", "deadman paddle hinge tab connected to handle grip", "31-36", (-525, 0, 1185), (52, 54, 36), 4, material="steel"),
            _soft_box("front_depth_lidar", "front stair depth sensor", "67", (wb / 2 + 190, 0, 675), (126, 56, 50), 6),
            _soft_box("rear_depth_lidar", "rear stair depth sensor", "67", (-wb / 2 - 190, 0, 675), (126, 56, 50), 6),
            _soft_box("dual_imu_mount", "redundant attitude sensor mount", "67", (0, 0, 586), (72, 44, 24), 5),
            _soft_box("front_lidar_mounting_arm", "front lidar bracket tied to sensor bridge", "67", (wb / 2 + 130, 0, 615), (150, 66, 88), 5, material="steel"),
            _soft_box("rear_lidar_mounting_arm", "rear lidar bracket tied to sensor bridge", "67", (-wb / 2 - 130, 0, 615), (150, 66, 88), 5, material="steel"),
            _soft_box("imu_mounting_plate_to_frame", "IMU plate tied to center crossmember", "67", (0, 0, 586), (96, 96, 24), 5, material="steel"),
            _soft_box("imu_crossmember_to_trunnions", "IMU crossmember spanning seat trunnion stands", "67", (0, 0, 585), (90, 560, 24), 5, material="steel"),
            _soft_box("front_sensor_connector_block", "sealed front sensor connector block", "67", (wb / 2 + 105, 90, 624), (58, 38, 28), 4, material="plastic"),
            _soft_box("rear_sensor_connector_block", "sealed rear sensor connector block", "67", (-wb / 2 - 105, -90, 624), (58, 38, 28), 4, material="plastic"),
            _cylinder("front_sensor_harness_conduit", "front sensor harness conduit", "67", (wb / 2 + 35, 0, 624), 6, 250, "Y", "plastic"),
            _cylinder("rear_sensor_harness_conduit", "rear sensor harness conduit", "67", (-wb / 2 - 35, 0, 624), 6, 250, "Y", "plastic"),
            _soft_box("front_sensor_connector_mount_plate", "front sensor connector plate tied to bridge arm and conduit", "67", (wb / 2 + 78, 45, 624), (96, 118, 18), 4, material="steel"),
            _soft_box("rear_sensor_connector_mount_plate", "rear sensor connector plate tied to bridge arm and conduit", "67", (-wb / 2 - 78, -45, 624), (96, 118, 18), 4, material="steel"),
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
    rail_y = sy * (tr / 2 + 55)
    selected_stroke = g.get("corner_stroke_selected_mm_SRC", 300.0)
    screw_len = g.get("ball_screw_shaft_length_mm_SRC", 430.0)
    side = "front" if sx > 0 else "rear"
    lr = "left" if sy > 0 else "right"
    prefix = f"{code.lower()}_"

    components.extend(
        [
            _tube(prefix + "tire_280x75", f"{side} {lr} pneumatic tire envelope", "60", (x, y, z_axle), wheel_d / 2, wheel_d / 2 - 24, wheel_w, "Y"),
            _tube(prefix + "hub_shell", f"{side} {lr} wheel hub shell", "60", (x, y, z_axle), 64, 22, wheel_w + 24, "Y"),
            _cylinder(prefix + "live_axle", f"{side} {lr} live axle", "60", (x, y, z_axle), 8.5, 138, "Y"),
            _cylinder(prefix + "inner_6002_hub_bearing", f"{side} {lr} sealed inner wheel hub bearing", "60", (x, y - sy * 32, z_axle), 16, 9, "Y", "steel"),
            _cylinder(prefix + "outer_6002_hub_bearing", f"{side} {lr} sealed outer wheel hub bearing", "60", (x, y + sy * 32, z_axle), 16, 9, "Y", "steel"),
            _cylinder(prefix + "axle_inner_spacer", f"{side} {lr} wheel bearing inner spacer", "60", (x, y, z_axle), 12, 44, "Y", "steel"),
            _cylinder(prefix + "axle_left_retaining_nut", f"{side} {lr} DIN 985 axle lock nut", "FASTENER", (x, y + sy * 76, z_axle), 11, 9, "Y", "black_oxide_steel"),
            _cylinder(prefix + "axle_right_retaining_nut", f"{side} {lr} DIN 985 axle lock nut", "FASTENER", (x, y - sy * 76, z_axle), 11, 9, "Y", "black_oxide_steel"),
            _soft_box(prefix + "fork_outer_plate", f"{side} {lr} fork outer plate", "60", (x, y + sy * 58, z_axle + 126), (118, 16, 205), 5),
            _soft_box(prefix + "fork_inner_plate", f"{side} {lr} fork inner plate", "60", (x, y - sy * 58, z_axle + 126), (118, 16, 205), 5),
            _soft_box(prefix + "fork_bridge", f"{side} {lr} fork bridge", "60", (x, y, z_axle + 230), (132, 132, 34), 5),
            _soft_box(prefix + "fork_upper_double_shear_yoke", f"{side} {lr} fork upper double-shear yoke", "60", (x, y, z_axle + 278), (74, 148, 42), 4, material="steel"),
            _soft_box(prefix + "fork_lower_double_shear_yoke", f"{side} {lr} fork lower double-shear yoke", "60", (x, y, z_axle + 82), (74, 148, 42), 4, material="steel"),
            _soft_box(prefix + "tower_lower_frame_lug", f"{side} {lr} bolted lower tower-to-frame lug", "61", (tower_x, rail_y, 370), (120, 36, 70), 5),
            _soft_box(prefix + "tower_upper_frame_lug", f"{side} {lr} bolted upper tower-to-frame lug", "61", (tower_x, rail_y, 610), (120, 36, 70), 5),
            _soft_box(prefix + "tower_top_thrust_block", f"{side} {lr} ball-screw top thrust bearing block", "62", (tower_x, y, 725), (96, 96, 42), 6, material="steel"),
            _soft_box(prefix + "tower_bottom_thrust_block", f"{side} {lr} ball-screw lower support bearing block", "62", (tower_x, y, 300), (96, 96, 42), 6, material="steel"),
            _cylinder(prefix + "steer_kingpin_bearing", f"{side} {lr} steering kingpin bearing stack", "60", (x, y, 445), 22, 270, "Z"),
            _cylinder(prefix + "kingpin_upper_taper_bearing", f"{side} {lr} upper kingpin bearing cone envelope", "60", (x, y, 545), 27, 22, "Z", "steel"),
            _cylinder(prefix + "kingpin_lower_taper_bearing", f"{side} {lr} lower kingpin bearing cone envelope", "60", (x, y, 345), 27, 22, "Z", "steel"),
            _cylinder(prefix + "kingpin_preload_locknut", f"{side} {lr} kingpin preload locknut", "FASTENER", (x, y, 585), 18, 12, "Z", "black_oxide_steel"),
            _soft_box(prefix + "steer_center_lock", f"{side} {lr} steering positive center lock", "66", (x, y, 590), (92, 68, 34), 5),
            _soft_box(prefix + "suspension_tower_cassette", f"{side} {lr} vertical suspension cassette", "61", (tower_x, y, 510), (92, 82, 430), 6),
            _cylinder(prefix + "swingarm_pivot_25mm", f"{side} {lr} 25 mm swingarm pivot shaft", "60", (x - sx * 108, y, 314), g["swingarm_pivot_diameter_mm"] / 2, 176, "Y"),
            _rod_xz(prefix + "lower_swingarm_left_link", f"{side} {lr} lower swing arm left tubular link", "60", (tower_x, y + sy * 35, 360), (x, y + sy * 35, z_axle + 105), 11, "steel"),
            _rod_xz(prefix + "lower_swingarm_right_link", f"{side} {lr} lower swing arm right tubular link", "60", (tower_x, y - sy * 35, 360), (x, y - sy * 35, z_axle + 105), 11, "steel"),
            _rod_xz(prefix + "upper_reaction_left_link", f"{side} {lr} upper reaction left tubular link", "60", (tower_x, y + sy * 28, 515), (x, y + sy * 28, z_axle + 250), 9, "steel"),
            _rod_xz(prefix + "upper_reaction_right_link", f"{side} {lr} upper reaction right tubular link", "60", (tower_x, y - sy * 28, 515), (x, y - sy * 28, z_axle + 250), 9, "steel"),
            _soft_box(prefix + "swingarm_clevis_at_tower", f"{side} {lr} tower clevis for lower swing arm", "60", (tower_x, y, 360), (46, 118, 54), 4, material="steel"),
            _soft_box(prefix + "swingarm_clevis_at_fork", f"{side} {lr} fork clevis for lower swing arm", "60", (x, y, z_axle + 105), (54, 118, 54), 4, material="steel"),
            _cylinder(prefix + "passive_spring_damper", f"{side} {lr} passive spring damper envelope", "61", (tower_x + sx * 42, y, 510), 18, 280, "Z"),
            _cylinder(prefix + "bnk1404_ball_screw", f"{side} {lr} THK BNK1404 ball screw 430 mm shaft envelope", "62", (tower_x, y, 512), g["ball_screw_diameter_mm_SRC"] / 2, screw_len, "Z"),
            _cylinder(prefix + "stroke_300mm_travel_datum", f"{side} {lr} selected 300 mm independent corner stroke datum", "62", (tower_x + sx * 34, y, 512), 2.5, selected_stroke, "Z", "copper"),
            _box(prefix + "ballnut_carriage_bridge", f"{side} {lr} ballnut carriage bridge", "62", (tower_x, y, 472), (82, 104, 54)),
            _soft_box(prefix + "moving_slider_plate", f"{side} {lr} guided moving slider plate connecting ballnut to links", "62", (tower_x + sx * 4, y, 472), (110, 132, 18), 4, material="steel"),
            _soft_box(prefix + "slider_left_double_shear_tab", f"{side} {lr} slider left pushrod clevis tab", "62", (tower_x + sx * 22, y + sy * 58, 472), (44, 16, 88), 3, material="steel"),
            _soft_box(prefix + "slider_right_double_shear_tab", f"{side} {lr} slider right pushrod clevis tab", "62", (tower_x + sx * 22, y - sy * 58, 472), (44, 16, 88), 3, material="steel"),
            _cylinder(prefix + "slider_rocker_shaft", f"{side} {lr} through rocker shaft linking slider tabs", "62", (tower_x + sx * 22, y, 472), 8, 150, "Y", "steel"),
            _soft_box(prefix + "slider_bellcrank_left_plate", f"{side} {lr} left bellcrank plate from ballnut slider to pushrods", "62", (tower_x + sx * 34, y + sy * 38, 440), (58, 12, 116), 3, sx * 8, material="steel"),
            _soft_box(prefix + "slider_bellcrank_right_plate", f"{side} {lr} right bellcrank plate from ballnut slider to pushrods", "62", (tower_x + sx * 34, y - sy * 38, 440), (58, 12, 116), 3, sx * 8, material="steel"),
            _rod_xz(prefix + "left_pushrod_slider_to_fork", f"{side} {lr} left M10 rod-end pushrod from slider bellcrank to fork yoke", "62", (tower_x + sx * 45, y + sy * 42, 440), (x, y + sy * 42, z_axle + 278), 7, "steel"),
            _rod_xz(prefix + "right_pushrod_slider_to_fork", f"{side} {lr} right M10 rod-end pushrod from slider bellcrank to fork yoke", "62", (tower_x + sx * 45, y - sy * 42, 440), (x, y - sy * 42, z_axle + 278), 7, "steel"),
            _box(prefix + "hsr15_linear_guide_rail_a", f"{side} {lr} THK HSR15 vertical guide rail A", "63", (tower_x - 24, y + sy * 31, 508), (16, 12, 400)),
            _box(prefix + "hsr15_linear_guide_rail_b", f"{side} {lr} THK HSR15 vertical guide rail B", "63", (tower_x + 24, y - sy * 31, 508), (16, 12, 400)),
            _soft_box(prefix + "hsr15c_guide_block_a", f"{side} {lr} THK HSR15C guide block A", "63", (tower_x - 24, y + sy * 31, 472), (24, 47, 56.6), 3, material="steel"),
            _soft_box(prefix + "hsr15c_guide_block_b", f"{side} {lr} THK HSR15C guide block B", "63", (tower_x + 24, y - sy * 31, 472), (24, 47, 56.6), 3, material="steel"),
            _cylinder(prefix + "actuator_motor_eci40", f"{side} {lr} Maxon EC-i 40 actuator motor envelope", "64", (tower_x, y + sy * 104, 728), 20, 82, "Y"),
            _cylinder(prefix + "gpx42_gearhead", f"{side} {lr} GPX42 12:1 gearhead envelope", "65", (tower_x, y + sy * 50, 728), 21, 58, "Y"),
            _cylinder(prefix + "ab60s_power_off_holding_brake", f"{side} {lr} maxon AB 60 S 5 Nm normally-engaged holding brake", "66", (tower_x, y + sy * 162, 728), 30, 39, "Y", "steel"),
            _box(prefix + "anti_drop_rack", f"{side} {lr} anti drop rack", "66", (tower_x - sx * 68, y, 510), (20, 24, 332)),
            _box(prefix + "anti_drop_pawl", f"{side} {lr} primary anti drop pawl", "66", (tower_x - sx * 46, y + sy * 34, 390), (42, 18, 92)),
            _soft_box(prefix + "upper_limit_switch", f"{side} {lr} upper travel limit switch", "67/68", (tower_x + sx * 70, y, 690), (38, 18, 24), 3, material="electronics"),
            _soft_box(prefix + "lower_limit_switch", f"{side} {lr} lower travel limit switch", "67/68", (tower_x + sx * 70, y, 332), (38, 18, 24), 3, material="electronics"),
            _soft_box(prefix + "upper_polyurethane_bump_stop", f"{side} {lr} upper compression bump stop", "66", (tower_x + sx * 4, y, 688), (52, 52, 22), 6, material="rubber"),
            _soft_box(prefix + "lower_polyurethane_bump_stop", f"{side} {lr} lower rebound bump stop", "66", (tower_x + sx * 4, y, 328), (52, 52, 22), 6, material="rubber"),
            _cylinder(prefix + "bg75_wheel_drive_motor", f"{side} {lr} Dunkermotoren BG75-class wheel drive motor envelope", "69", (x, y - sy * 122, z_axle), 37.5, 96, "Y"),
            _cylinder(prefix + "plg75_wheel_planetary_gearbox", f"{side} {lr} PLG75-class planetary gearbox wheel reduction envelope", "69", (x, y - sy * 70, z_axle), 39, 58, "Y", "steel"),
            _soft_box(prefix + "deutsch_dtp_power_connector", f"{side} {lr} TE DEUTSCH DTP sealed power connector", "69", (tower_x + sx * 82, y + sy * 96, 610), (47.27, 27.15, 22.05), 3, material="plastic"),
            _soft_box(prefix + "deutsch_dt_signal_connector", f"{side} {lr} TE DEUTSCH DT sealed signal connector", "67/68", (tower_x + sx * 82, y - sy * 96, 650), (44.02, 22.25, 36.45), 3, material="plastic"),
            _cylinder(prefix + "corner_power_harness_conduit", f"{side} {lr} sealed power harness conduit", "69", (tower_x + sx * 40, y + sy * 96, 610), 7, 160, "X", "plastic"),
            _cylinder(prefix + "corner_signal_harness_conduit", f"{side} {lr} sealed signal harness conduit", "67/68", (tower_x + sx * 40, y - sy * 96, 620), 5, 160, "X", "plastic"),
            _gusset(prefix + "tower_frame_gusset", f"{side} {lr} triangular tower to rail gusset", "61", (tower_x + sx * 42, y, 375), 165, 160, 8, sx),
        ]
    )

    for i, z_mount in enumerate((370, 610)):
        components.extend(_bolt_y(prefix + f"tower_lug_m8_{i}_upper", (tower_x - 32, rail_y, z_mount + 18), 76))
        components.extend(_bolt_y(prefix + f"tower_lug_m8_{i}_lower", (tower_x + 32, rail_y, z_mount - 18), 76))

    for i, z_fastener in enumerate((350, 430, 510, 590, 670)):
        components.extend(_bolt_y(prefix + f"guide_rail_a_m5_{i}", (tower_x - 24, y + sy * 46, z_fastener), 28, 5))
        components.extend(_bolt_y(prefix + f"guide_rail_b_m5_{i}", (tower_x + 24, y - sy * 46, z_fastener), 28, 5))

    for i, z_block in enumerate((300, 725)):
        components.extend(_bolt_y(prefix + f"thrust_block_m8_a_{i}", (tower_x - 28, y, z_block), 110))
        components.extend(_bolt_y(prefix + f"thrust_block_m8_b_{i}", (tower_x + 28, y, z_block), 110))

    for i, pin_z in enumerate((z_axle + 105, z_axle + 250, 360, 515)):
        pin_x = x if i < 2 else tower_x
        components.extend(_bolt_y(prefix + f"suspension_link_pin_{i}", (pin_x, y, pin_z), 146, 10))
        components.append(_tube(prefix + f"iglidur_g_pivot_bushing_{i}", f"{side} {lr} iglidur G pivot bushing sleeve", "60", (pin_x, y, pin_z), 15, 10.5, 104, "Y", "plastic"))
        components.append(_tube(prefix + f"aurora_m10_rod_end_outer_race_{i}", f"{side} {lr} Aurora AM-M10T rod-end bearing race at suspension pivot", "60", (pin_x, y + sy * 49, pin_z), 13.5, 5.0, 14.0, "Y", "steel"))
        components.append(_cylinder(prefix + f"din985_m10_locknut_{i}", f"{side} {lr} M10 prevailing torque lock nut at suspension pin", "FASTENER", (pin_x, y - sy * 84, pin_z), 12, 10, "Y", "black_oxide_steel"))

    for i, (px, pz) in enumerate(((tower_x + sx * 45, 440), (x, z_axle + 278))):
        components.extend(_bolt_y(prefix + f"pushrod_clevis_m10_{i}", (px, y + sy * 42, pz), 92, 10))
        components.append(_tube(prefix + f"pushrod_aurora_m10_left_rod_end_{i}", f"{side} {lr} left AM-M10T pushrod rod-end head", "62", (px, y + sy * 42, pz), 13.5, 5.0, 14.0, "Y", "steel"))
        components.append(_cylinder(prefix + f"pushrod_left_m10_jam_nut_{i}", f"{side} {lr} left pushrod M10 jam nut", "FASTENER", (px - sx * 28, y + sy * 42, pz), 11, 7, "X", "black_oxide_steel"))
        components.extend(_bolt_y(prefix + f"pushrod_clevis_m10_right_{i}", (px, y - sy * 42, pz), 92, 10))
        components.append(_tube(prefix + f"pushrod_aurora_m10_right_rod_end_{i}", f"{side} {lr} right AM-M10T pushrod rod-end head", "62", (px, y - sy * 42, pz), 13.5, 5.0, 14.0, "Y", "steel"))
        components.append(_cylinder(prefix + f"pushrod_right_m10_jam_nut_{i}", f"{side} {lr} right pushrod M10 jam nut", "FASTENER", (px - sx * 28, y - sy * 42, pz), 11, 7, "X", "black_oxide_steel"))

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


def build_components(params: dict, include_reference: bool = False) -> list[Component]:
    g = params["geometry"]
    components: list[Component] = []
    _add_chassis(components, g)
    _add_seat_pod(components)
    _add_controls(components)
    _add_handle_and_sensors(components, g)
    for code, sx, sy in (("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1)):
        _add_corner(components, g, code, sx, sy)
    if include_reference:
        _add_reference_geometry(components, g)
    return components


def build_stair_climb_pose_components(params: dict) -> list[Component]:
    """Return a visual kinematic pose: chassis pitched to stair angle, pod held level.

    The normal shop assembly is kept as the primary audited build. This second
    export makes the corrected vehicle architecture visible: the mobility
    chassis follows the 36.03 degree staircase while the occupant pod remains
    level relative to gravity.
    """
    g = params["geometry"]
    pitch = -g["stair_angle_deg_CALC"]
    pivot = (0.0, 0.0, 645.0)
    posed: list[Component] = []
    for component in build_components(params, include_reference=False):
        if _is_occupant_pod_leveling_group(component):
            posed.append(Component(component.name + "_level_pod_pose", component.role, component.callout, component.shape, component.material))
        else:
            posed.append(_rotated_y(component, pitch, pivot, "_pitched_chassis_pose"))

    # Reference stair blocks are kept unrotated so the pitched chassis can be
    # inspected against the actual rise/run target.
    _add_reference_geometry(posed, g)
    return posed


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
    compound = _compound(components)

    step_path = out_dir / "Terrain_Elevate_P1_V0_59_OCCT.step"
    stl_path = out_dir / "Terrain_Elevate_P1_V0_59_OCCT.stl"
    pose_step_path = out_dir / "Terrain_Elevate_P1_V0_59_stair_climb_pose.step"
    pose_stl_path = out_dir / "Terrain_Elevate_P1_V0_59_stair_climb_pose.stl"
    dxf_path = out_dir / "Terrain_Elevate_P1_V0_59_package.dxf"
    manifest_path = out_dir / "Terrain_Elevate_P1_V0_59_manifest.json"

    step_assembly = _assembly_from_components(components)
    step_assembly.save(str(step_path), exportType="STEP")
    exporters.export(compound, str(stl_path), exportType="STL", tolerance=0.25, angularTolerance=0.2)
    pose_components = build_stair_climb_pose_components(params)
    pose_assembly = _assembly_from_components(pose_components)
    pose_assembly.save(str(pose_step_path), exportType="STEP")
    exporters.export(_compound(pose_components), str(pose_stl_path), exportType="STL", tolerance=0.25, angularTolerance=0.2)
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
            "stair_climb_pose_step_file": pose_step_path.name,
            "stair_climb_pose_stl_file": pose_stl_path.name,
            "dxf_file": dxf_path.name,
            "manifest_file": manifest_path.name,
        },
        "output_bytes": {
            "step": step_path.stat().st_size,
            "stl": stl_path.stat().st_size,
            "stair_climb_pose_step": pose_step_path.stat().st_size,
            "stair_climb_pose_stl": pose_stl_path.stat().st_size,
            "dxf": dxf_path.stat().st_size,
        },
        "pose_exports": {
            "shop_assembly": "Primary connected and audited assembly in neutral/shop pose.",
            "stair_climb_pose": "Kinematic visual pose: chassis rotated to the 36.03 degree stair angle about the occupant roll/pitch reference region while pod components remain level; reference stair geometry included.",
            "stair_angle_deg": params["geometry"]["stair_angle_deg_CALC"],
        },
        "locked_constraints": params["locked_constraints"],
        "geometry": params["geometry"],
        "sourced_parts": [
            {
                "callout": "62",
                "part": "THK BNK1404-3RRG2+430LC7Y",
                "modeled_as": "14 mm screw shaft, 430 mm shaft length, 300 mm travel datum, fixed/support bearing blocks",
                "source_values": {
                    "stroke_mm": 300.0,
                    "lead_mm": 4.0,
                    "dynamic_load_rating_kN": 4.2,
                    "static_load_rating_kN": 7.6,
                    "recommended_fixed_support": "EK12 or FK12",
                    "recommended_supported_support": "EF12 or FF12",
                },
            },
            {
                "callout": "62",
                "part": "THK FK12/FF12 screw support unit family",
                "modeled_as": "fixed and supported end bearing housings tied into the suspension tower",
                "source_values": {
                    "ball_screw_shaft_outer_diameter_mm": 14.0,
                    "recommended_by": "THK BNK1404-3RRG2+430LC7Y detail specification",
                },
            },
            {
                "callout": "63",
                "part": "THK HSR15C linear guide block with HSR15 rail",
                "modeled_as": "paired vertical guide rails and two HSR15C carriage blocks per corner",
                "source_values": {
                    "block_height_mm": 24.0,
                    "block_width_mm": 47.0,
                    "block_length_mm": 56.6,
                    "dynamic_load_rating_kN": 10.9,
                    "static_load_rating_kN": 15.7,
                    "rail_tap": "M5",
                },
            },
            {
                "callout": "64",
                "part": "maxon EC-i 40 screened actuator motor family",
                "modeled_as": "40 mm diameter motor envelope with gearhead/brake stack",
                "source_values": {
                    "diameter_mm": 40.0,
                    "nominal_power_W": 100.0,
                },
            },
            {
                "callout": "13-26",
                "part": "Thomson Electrak MD MDxxA200 pod pitch-leveling actuator family",
                "modeled_as": "paired compact linear actuator bodies, extension tubes, and clevis pins controlling 36.03 degree pod pitch correction",
                "source_values": {
                    "selected_dynamic_load_N": 2000.0,
                    "maximum_stroke_mm": 300.0,
                    "supported_input_voltages_VDC": "12/24/48 family",
                    "protection_class_family": "IP67/IP69K static, IP66 dynamic",
                },
            },
            {
                "callout": "66",
                "part": "maxon AB 60 S 24 VDC 5.0 Nm holding brake",
                "modeled_as": "60 mm diameter, 39 mm long normally-engaged brake envelope at each suspension actuator",
                "source_values": {
                    "holding_torque_Nm": 5.0,
                    "calculated_required_holding_torque_Nm_with_safety_factor": 2.97,
                    "nominal_voltage_VDC": 24.0,
                    "unpowered_state": "braked",
                    "not_for_dynamic_braking": True,
                },
            },
            {
                "callout": "69",
                "part": "Dunkermotoren BG75-class BLDC motor with PLG75-class planetary gearbox",
                "modeled_as": "75 mm class wheel drive motor plus PLG75-class planetary wheel reduction at each wheel",
                "source_values": {
                    "screen_requirement_per_wheel_torque_Nm": 53.8,
                    "manufacturer_family_power_range_W": "BG family includes high-power BLDC gearmotor variants; BG75 dMove class up to 810 W per official press/spec summary",
                    "planetary_gearbox_family_continuous_torque_Nm": "PLG family up to 130 Nm",
                },
            },
            {
                "callout": "68/69",
                "part": "NXP S32K3 / STM32G474 / TI DRV8353 screened control and gate-driver families",
                "modeled_as": "electronics housings with hardwired inhibit/watchdog and local motor-control package envelopes",
            },
            {
                "callout": "67/68/69",
                "part": "TE Connectivity DEUTSCH DTP and DT sealed connector families",
                "modeled_as": "DTP power connectors and DT signal connectors at each corner and sensor branch",
                "source_values": {
                    "DTP_contact_current_A": 25,
                    "DTP_wire_range_AWG": "10-14",
                    "DTP_ip_rating": "IP68/IP6K9K family",
                    "DT_contact_current_A": 13,
                },
            },
            {
                "callout": "69",
                "part": "Littelfuse MIDI 498 58 V bolt-down high-current fuse holder",
                "modeled_as": "left and right branch protection fuse blocks",
                "source_values": {
                    "continuous_current_A": 150,
                    "max_current_A": 200,
                    "voltage_rating_V": 58,
                },
            },
            {
                "callout": "60",
                "part": "igus iglidur G pivot bushing material and igubal EGLM-25 spherical bearing family",
                "modeled_as": "polymer pivot sleeves at suspension link pins and 25 mm misalignment bearing envelope at swing-arm pivots",
                "source_values": {
                    "iglidur_g_compressive_strength_MPa": 78.0,
                    "eglm_25_shaft_diameter_mm": 25.0,
                    "maintenance": "dry-running/lubrication-free family",
                },
            },
            {
                "callout": "60/62",
                "part": "Aurora AM-M10T metric male PTFE-lined rod end",
                "modeled_as": "M10 spherical rod-end bearing heads at suspension pivots and slider-to-fork pushrods",
                "source_values": {
                    "ball_bore_mm": 10.0,
                    "ball_width_mm": 14.0,
                    "head_diameter_mm": 27.0,
                    "static_radial_load_capacity_N": 50227.0,
                    "screened_design_load_N_with_safety_factor": 1985.85,
                },
            },
            {
                "callout": "FASTENER",
                "part": "ISO 4762 socket-head cap screw families and DIN 985 / ISO 10511 prevailing torque lock nuts",
                "modeled_as": "M5/M6/M8/M10 shanks, socket-heads, washers, axle nuts, kingpin locknuts, pushrod jam nuts, and clevis lock nuts",
                "source_values": {
                    "m8_socket_head_diameter_mm_modeled": 13.0,
                    "m8_socket_head_height_mm_modeled": 8.0,
                    "m8_din985_drive_size_mm": 13.0,
                    "m8_din985_height_mm": 8.0,
                },
            },
        ],
        "ebom_summary": {
            "cad_body_count": len(components),
            "by_material": _count_by(component.material for component in components),
            "by_callout": _count_by(component.callout for component in components),
            "by_subsystem": _count_by(_category_for(component) for component in components),
        },
        "step_stl_reference_geometry": "excluded; reference stair and width datums are carried in DXF/package context",
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
