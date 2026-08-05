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


def _shell_box(name: str, role: str, callout: str, center, size, wall_mm: float, radius: float = 8.0, material: str | None = None) -> Component:
    """A real hollow cover panel: an outer filleted box minus a smaller
    inner box, leaving a wall of thickness wall_mm -- a genuinely buildable
    thin-walled shell (composite/sheet-metal cover), not a solid block
    standing in for one."""
    ox, oy, oz = size
    outer = cq.Workplane("XY").box(ox, oy, oz)
    try:
        outer = outer.edges().fillet(radius)
    except Exception:
        pass
    inner_size = (max(ox - 2 * wall_mm, 1.0), max(oy - 2 * wall_mm, 1.0), max(oz - 2 * wall_mm, 1.0))
    inner = cq.Workplane("XY").box(*inner_size)
    solid = outer.cut(inner)
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


def _tread_lug(name: str, role: str, callout: str, wheel_center, wheel_radius: float, angle_deg: float, width: float, material: str = "rubber") -> Component:
    cx, cy, cz = wheel_center
    angle = math.radians(angle_deg)
    x = cx + math.sin(angle) * wheel_radius
    z = cz + math.cos(angle) * wheel_radius
    solid = (
        cq.Workplane("XY")
        .box(38, width, 10)
        .rotate((0, 0, 0), (0, 1, 0), angle_deg)
        .translate((x, cy, z))
    )
    return Component(name, role, callout, solid, material)


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
    if any(token in text for token in ("tire", "wheel", "hub", "axle", "brake", "motor", "encoder")):
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
            _soft_box("front_sensor_bridge", "front O1D100 preview and edge sensor mounting bridge", "67", (wb / 2 + 80, 0, z + 175), (34, 580, 42), 6),
            _soft_box("rear_sensor_bridge", "rear O1D100 preview and edge sensor mounting bridge", "67", (-wb / 2 - 80, 0, z + 175), (34, 580, 42), 6),
            _cylinder("fold_hinge_axis_left", "folding handle and chassis hinge shaft", "1-12", (-540, outer_y, z + 95), 16, 145, "Y", "steel"),
            _cylinder("fold_hinge_axis_right", "folding handle and chassis hinge shaft", "1-12", (-540, -outer_y, z + 95), 16, 145, "Y", "steel"),
            _soft_box("handle_left_lower_hinge_bracket", "bolted handle hinge bracket tied into left rail", "31-36", (-540, outer_y, z + 95), (120, 42, 84), 6, material="steel"),
            _soft_box("handle_right_lower_hinge_bracket", "bolted handle hinge bracket tied into right rail", "31-36", (-540, -outer_y, z + 95), (120, 42, 84), 6, material="steel"),
            _soft_box("handle_left_hinge_backstay", "left hinge backstay from handle bracket to chassis rail", "31-36", (-485, outer_y, z + 30), (150, 36, 92), 5, material="steel"),
            _soft_box("handle_right_hinge_backstay", "right hinge backstay from handle bracket to chassis rail", "31-36", (-485, -outer_y, z + 30), (150, 36, 92), 5, material="steel"),
            _soft_box("seat_left_roll_trunnion_stand", "seat roll trunnion stand tied to left chassis rail", "13-26", (0, 265, z + 165), (86, 58, 260), 7, material="steel"),
            _soft_box("seat_right_roll_trunnion_stand", "seat roll trunnion stand tied to right chassis rail", "13-26", (0, -265, z + 165), (86, 58, 260), 7, material="steel"),
            # Battery is a slide-out pack on the tray below, released by two
            # independent spring latches (not a single central lock, per
            # requirement that one lock failure must not create an unsafe
            # state) plus a deliberate two-hand pull -- not bolted straps.
            _soft_box("battery_quick_release_rail_left", "left slide-out rail, battery pack removal", "41-59", (-60, 85, z - 55), (280, 14, 20), 4, material="steel"),
            _soft_box("battery_quick_release_rail_right", "right slide-out rail, battery pack removal", "41-59", (-60, -85, z - 55), (280, 14, 20), 4, material="steel"),
            _soft_box("battery_quick_release_latch_housing_left", "left positive spring latch housing, independent of right latch", "41-59", (100, 85, z - 55), (36, 24, 20), 4, material="steel"),
            _soft_box("battery_quick_release_latch_housing_right", "right positive spring latch housing, independent of left latch", "41-59", (100, -85, z - 55), (36, 24, 20), 4, material="steel"),
            _cylinder("battery_quick_release_latch_pin_left", "left latch pin, spring-extended into tray keeper when locked", "41-59", (100, 85, z - 55), 5, 30, "Y", "steel"),
            _cylinder("battery_quick_release_latch_pin_right", "right latch pin, spring-extended into tray keeper when locked", "41-59", (100, -85, z - 55), 5, 30, "Y", "steel"),
            _soft_box("battery_quick_release_pull_handle", "two-hand deliberate release pull tab linked to both latch pins", "41-59", (115, 0, z - 55), (20, 60, 20), 4, material="plastic"),
            _soft_box("battery_latch_engagement_sensor_left", "left latch full-engagement confirmation sensor, not credited as the structural lock", "67", (100, 85, z - 55), (10, 8, 10), 2, material="electronics"),
            _soft_box("battery_latch_engagement_sensor_right", "right latch full-engagement confirmation sensor, not credited as the structural lock", "67", (100, -85, z - 55), (10, 8, 10), 2, material="electronics"),
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
    # Pod is a removable carrier (car-seat / travel-system style), not
    # permanently bolted: the roll shaft still rotates freely in its
    # bearings for active leveling, but axial retention is now a positive
    # rotary latch pair (independent left/right, same Southco R4 family as
    # the battery quick-release) instead of bolts, so a caregiver can lift
    # the whole pod off without tools -- and folding the now-lighter,
    # pod-free chassis is correspondingly easier.
    for i, y_side in enumerate((276, -276)):
        lr = "left" if y_side > 0 else "right"
        sign = 1 if y_side > 0 else -1
        components.extend(
            [
                _soft_box(f"pod_release_latch_housing_{i}", f"{lr} positive rotary latch retaining pod roll shaft, independent of other side", "13-26", (0, y_side, 645), (46, 30, 34), 4, material="steel"),
                _cylinder(f"pod_release_latch_pin_{i}", f"{lr} pod release latch pin, spring-extended into shaft retaining groove when locked", "13-26", (0, y_side, 645), 5, 26, "X", "steel"),
                _soft_box(f"pod_release_confirm_sensor_{i}", f"{lr} pod latch full-engagement confirmation sensor, not credited as the structural lock", "67", (0, y_side + sign * 18, 645), (10, 8, 10), 2, material="electronics"),
            ]
        )
    components.append(_soft_box("pod_release_grip", "two-hand deliberate pod release grip, actuates both latches together", "13-26", (0, 0, 645), (40, 40, 30), 5, material="plastic"))


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
            # Handle control interface: real momentary pushbuttons (not a
            # touchscreen/single-sensor dependency), a haptic actuator that
            # gives the caregiver physical feedback (assist engaged, stair
            # detected, fault) without requiring them to look away from the
            # child, and a status indicator as a secondary visual channel.
            _soft_box("handle_grip_power_button", "sealed power/standby momentary pushbutton", "31-36", (-550, 80, 1170), (20, 20, 16), 4, material="electronics"),
            _soft_box("handle_grip_assist_mode_button", "sealed assist-level cycle momentary pushbutton", "31-36", (-550, -80, 1170), (20, 20, 16), 4, material="electronics"),
            _cylinder("handle_haptic_feedback_motor", "haptic feedback actuator in grip -- assist/stair-detect/fault feedback the caregiver can feel without looking away from the child", "31-36", (-550, 0, 1160), 6, 20, "Y", "electronics"),
            _soft_box("handle_status_indicator_led", "sealed multi-color status indicator, secondary to haptic/audible feedback", "31-36", (-550, 160, 1170), (14, 14, 10), 3, material="electronics"),
            _soft_box("handle_left_lower_yoke_link", "left lower handle yoke linking upright to hinge shaft", "31-36", (-545, 325, 665), (48, 124, 390), 5, material="steel"),
            _soft_box("handle_right_lower_yoke_link", "right lower handle yoke linking upright to hinge shaft", "31-36", (-545, -325, 665), (48, 124, 390), 5, material="steel"),
            _soft_box("deadman_paddle_hinge_tab", "deadman paddle hinge tab connected to handle grip", "31-36", (-525, 0, 1185), (52, 54, 36), 4, material="steel"),
            _soft_box("front_ifm_o1d100_distance_sensor", "front ifm O1D100 stair preview distance sensor", "67", (wb / 2 + 190, 0, 675), (59, 42, 52), 5, material="electronics"),
            _soft_box("rear_ifm_o1d100_distance_sensor", "rear ifm O1D100 stair preview distance sensor", "67", (-wb / 2 - 190, 0, 675), (59, 42, 52), 5, material="electronics"),
            _soft_box("dual_imu_mount", "redundant attitude sensor mount", "67", (0, 0, 586), (72, 44, 24), 5),
            _soft_box("front_o1d100_mounting_arm", "front O1D100 bracket tied to sensor bridge", "67", (wb / 2 + 130, 0, 615), (150, 66, 88), 5, material="steel"),
            _soft_box("rear_o1d100_mounting_arm", "rear O1D100 bracket tied to sensor bridge", "67", (-wb / 2 - 130, 0, 615), (150, 66, 88), 5, material="steel"),
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
    # The tower/kingpin/steering/actuator coordinates below were tuned as
    # absolute literals against the original 280 mm-diameter (140 mm radius)
    # wheel. z0 rigidly translates that whole cassette up/down so it stays
    # correctly positioned above whatever wheel radius parameters.json now
    # specifies, without re-deriving each coordinate by hand. This is a
    # simplification: a real swing-arm/pushrod linkage's angles would also
    # change (not just translate) with a different wheel radius -- a full
    # kinematic re-solve for the new radius is still open.
    z0 = z_axle - 140.0
    tower_x = x - sx * 100
    rail_y = sy * (tr / 2 + 55)
    selected_stroke = g.get("corner_stroke_selected_mm_SRC", 300.0)
    screw_len = g.get("ball_screw_shaft_length_mm_SRC", 430.0)
    side = "front" if sx > 0 else "rear"
    lr = "left" if sy > 0 else "right"
    prefix = f"{code.lower()}_"

    components.extend(
        [
            _tube(prefix + f"tire_{wheel_d:.0f}x{wheel_w:.0f}", f"{side} {lr} pneumatic tire envelope", "60", (x, y, z_axle), wheel_d / 2, wheel_d / 2 - 24, wheel_w, "Y"),
            _tube(prefix + "hub_shell", f"{side} {lr} wheel hub shell", "60", (x, y, z_axle), 64, 22, wheel_w + 24, "Y"),
            _cylinder(prefix + "live_axle", f"{side} {lr} live axle", "60", (x, y, z_axle), 8.5, 138, "Y"),
            _cylinder(prefix + "wheel_encoder_magnet_ring", f"{side} {lr} magnetic wheel encoder target ring", "67/69", (x, y + sy * 52, z_axle), 48, 5, "Y", "steel"),
            _soft_box(prefix + "wheel_encoder_pickup_bracket", f"{side} {lr} wheel speed encoder pickup bracket tied to fork", "67/69", (x + sx * 44, y + sy * 61, z_axle + 42), (52, 14, 80), 3, material="steel"),
            _soft_box(prefix + "wheel_encoder_sensor", f"{side} {lr} sealed wheel speed encoder sensor", "67/69", (x + sx * 44, y + sy * 72, z_axle + 42), (28, 16, 20), 2, material="electronics"),
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
            _soft_box(prefix + "tower_lower_frame_lug", f"{side} {lr} bolted lower tower-to-frame lug", "61", (tower_x, rail_y, 370 + z0), (120, 36, 70), 5),
            _soft_box(prefix + "tower_upper_frame_lug", f"{side} {lr} bolted upper tower-to-frame lug", "61", (tower_x, rail_y, 610 + z0), (120, 36, 70), 5),
            # FK12/FF12-style screw support units bolt to the OUTSIDE of the
            # cassette end faces -- they are 96 x 96, larger than the 92 x 82
            # cassette cross-section, so they cannot sit inside it. They were
            # centred on the cassette end planes (725 / 300), straddling the
            # wall by roughly half their height. Moved clear: the cassette
            # spans 295..725 in local z, so a 42 mm block sits fully outside at
            # 725 + 21 and 295 - 21.
            _soft_box(prefix + "tower_top_thrust_block", f"{side} {lr} ball-screw top thrust bearing block", "62", (tower_x, y, 746 + z0), (96, 96, 42), 6, material="steel"),
            _soft_box(prefix + "tower_bottom_thrust_block", f"{side} {lr} ball-screw lower support bearing block", "62", (tower_x, y, 274 + z0), (96, 96, 42), 6, material="steel"),
            _cylinder(prefix + "steer_kingpin_bearing", f"{side} {lr} steering kingpin bearing stack", "60", (x, y, 445 + z0), 22, 270, "Z"),
            _cylinder(prefix + "kingpin_upper_taper_bearing", f"{side} {lr} upper kingpin bearing cone envelope", "60", (x, y, 545 + z0), 27, 22, "Z", "steel"),
            _cylinder(prefix + "kingpin_lower_taper_bearing", f"{side} {lr} lower kingpin bearing cone envelope", "60", (x, y, 345 + z0), 27, 22, "Z", "steel"),
            _cylinder(prefix + "kingpin_preload_locknut", f"{side} {lr} kingpin preload locknut", "FASTENER", (x, y, 585 + z0), 18, 12, "Z", "black_oxide_steel"),
            _soft_box(prefix + "steer_center_lock", f"{side} {lr} steering positive center lock", "66", (x, y, 590 + z0), (92, 68, 34), 5),
            # A cassette is a HOUSING: the ball screw, guide rails, slider
            # plate and spring all live inside it. Modelled as a solid block it
            # was the single worst interference source in the assembly -- 9469
            # cm3 across 294 pairs, about 30% of all structural interference,
            # simply because every component it contains was buried in solid
            # material. Hollow shell with a 3 mm wall.
            _shell_box(prefix + "suspension_tower_cassette", f"{side} {lr} vertical suspension cassette", "61", (tower_x, y, 510 + z0), (92, 82, 430), 3.0, 6),
            _cylinder(prefix + "swingarm_pivot_25mm", f"{side} {lr} 25 mm swingarm pivot shaft", "60", (x - sx * 108, y, 314 + z0), g["swingarm_pivot_diameter_mm"] / 2, 176, "Y"),
            _rod_xz(prefix + "lower_swingarm_left_link", f"{side} {lr} lower swing arm left tubular link", "60", (tower_x, y + sy * 35, 360 + z0), (x, y + sy * 35, z_axle + 105), 11, "steel"),
            _rod_xz(prefix + "lower_swingarm_right_link", f"{side} {lr} lower swing arm right tubular link", "60", (tower_x, y - sy * 35, 360 + z0), (x, y - sy * 35, z_axle + 105), 11, "steel"),
            _rod_xz(prefix + "upper_reaction_left_link", f"{side} {lr} upper reaction left tubular link", "60", (tower_x, y + sy * 28, 515 + z0), (x, y + sy * 28, z_axle + 250), 9, "steel"),
            _rod_xz(prefix + "upper_reaction_right_link", f"{side} {lr} upper reaction right tubular link", "60", (tower_x, y - sy * 28, 515 + z0), (x, y - sy * 28, z_axle + 250), 9, "steel"),
            _soft_box(prefix + "swingarm_clevis_at_tower", f"{side} {lr} tower clevis for lower swing arm", "60", (tower_x, y, 360 + z0), (46, 118, 54), 4, material="steel"),
            _soft_box(prefix + "swingarm_clevis_at_fork", f"{side} {lr} fork clevis for lower swing arm", "60", (x, y, z_axle + 105), (54, 118, 54), 4, material="steel"),
            _cylinder(prefix + "passive_spring_damper", f"{side} {lr} passive spring damper envelope", "61", (tower_x + sx * 42, y, 510 + z0), 18, 280, "Z"),
            _cylinder(prefix + "bnk2010_ball_screw", f"{side} {lr} THK BNK2010 ball screw 499 mm shaft envelope", "62", (tower_x, y, 512 + z0), g["ball_screw_diameter_mm_SRC"] / 2, screw_len, "Z"),
            _cylinder(prefix + "stroke_300mm_travel_datum", f"{side} {lr} selected 300 mm independent corner stroke datum", "62", (tower_x + sx * 34, y, 512 + z0), 2.5, selected_stroke, "Z", "copper"),
            _box(prefix + "ballnut_carriage_bridge", f"{side} {lr} ballnut carriage bridge", "62", (tower_x, y, 472 + z0), (82, 104, 54)),
            _soft_box(prefix + "moving_slider_plate", f"{side} {lr} guided moving slider plate connecting ballnut to links", "62", (tower_x + sx * 4, y, 472 + z0), (110, 132, 18), 4, material="steel"),
            _soft_box(prefix + "slider_left_double_shear_tab", f"{side} {lr} slider left pushrod clevis tab", "62", (tower_x + sx * 22, y + sy * 58, 472 + z0), (44, 16, 88), 3, material="steel"),
            _soft_box(prefix + "slider_right_double_shear_tab", f"{side} {lr} slider right pushrod clevis tab", "62", (tower_x + sx * 22, y - sy * 58, 472 + z0), (44, 16, 88), 3, material="steel"),
            _cylinder(prefix + "slider_rocker_shaft", f"{side} {lr} through rocker shaft linking slider tabs", "62", (tower_x + sx * 22, y, 472 + z0), 8, 150, "Y", "steel"),
            _soft_box(prefix + "slider_bellcrank_left_plate", f"{side} {lr} left bellcrank plate from ballnut slider to pushrods", "62", (tower_x + sx * 34, y + sy * 38, 440 + z0), (58, 12, 116), 3, sx * 8, material="steel"),
            _soft_box(prefix + "slider_bellcrank_right_plate", f"{side} {lr} right bellcrank plate from ballnut slider to pushrods", "62", (tower_x + sx * 34, y - sy * 38, 440 + z0), (58, 12, 116), 3, sx * 8, material="steel"),
            _rod_xz(prefix + "left_pushrod_slider_to_fork", f"{side} {lr} left M10 rod-end pushrod from slider bellcrank to fork yoke", "62", (tower_x + sx * 45, y + sy * 42, 440 + z0), (x, y + sy * 42, z_axle + 278), 7, "steel"),
            _rod_xz(prefix + "right_pushrod_slider_to_fork", f"{side} {lr} right M10 rod-end pushrod from slider bellcrank to fork yoke", "62", (tower_x + sx * 45, y - sy * 42, 440 + z0), (x, y - sy * 42, z_axle + 278), 7, "steel"),
            _box(prefix + "hsr15_linear_guide_rail_a", f"{side} {lr} THK HSR15 vertical guide rail A", "63", (tower_x - 24, y + sy * 31, 508 + z0), (16, 12, 400)),
            _box(prefix + "hsr15_linear_guide_rail_b", f"{side} {lr} THK HSR15 vertical guide rail B", "63", (tower_x + 24, y - sy * 31, 508 + z0), (16, 12, 400)),
            _soft_box(prefix + "magnetic_linear_scale", f"{side} {lr} absolute linear position scale for corner actuator", "67/68", (tower_x + sx * 58, y, 508 + z0), (10, 16, 330), 2, material="electronics"),
            _soft_box(prefix + "linear_scale_read_head", f"{side} {lr} moving read head tied to ballnut slider", "67/68", (tower_x + sx * 58, y, 472 + z0), (24, 24, 30), 2, material="electronics"),
            _soft_box(prefix + "hsr15c_guide_block_a", f"{side} {lr} THK HSR15C guide block A", "63", (tower_x - 24, y + sy * 31, 472 + z0), (24, 47, 56.6), 3, material="steel"),
            _soft_box(prefix + "hsr15c_guide_block_b", f"{side} {lr} THK HSR15C guide block B", "63", (tower_x + 24, y - sy * 31, 472 + z0), (24, 47, 56.6), 3, material="steel"),
            _soft_box(prefix + "actuator_top_motor_mount_plate", f"{side} {lr} coaxial motor mount plate bolted to screw thrust block", "64/65", (tower_x, y, 748 + z0), (86, 86, 10), 3, material="steel"),
            _cylinder(prefix + "ruland_beam_coupling_motor_to_screw", f"{side} {lr} flexible coupling between GPX52 output and BNK2010 screw", "64/65", (tower_x, y, 735 + z0), 16, 32, "Z", "steel"),
            _cylinder(prefix + "screw_upper_shaft_collar", f"{side} {lr} clamp collar locating screw shaft below coupling", "62", (tower_x, y, 712 + z0), 17, 12, "Z", "black_oxide_steel"),
            _cylinder(prefix + "gpx52_gearhead", f"{side} {lr} Maxon GPX52 3.9:1 coaxial gearhead", "65", (tower_x, y, 790 + z0), 26, 50.2, "Z"),
            _cylinder(prefix + "actuator_motor_eci52", f"{side} {lr} Maxon EC-i 52 part 633919 actuator motor", "64", (tower_x, y, 860 + z0), 26, 90, "Z"),
            _cylinder(prefix + "ab44_power_off_holding_brake", f"{side} {lr} maxon AB 44 part 386054 2.5 Nm normally-engaged motor brake", "66", (tower_x, y, 926 + z0), 22, 26.9, "Z", "steel"),
            _tube(prefix + "motor_stack_protective_shroud", f"{side} {lr} sealed protective tube shroud around motor brake stack", "64/65/66", (tower_x, y, 858 + z0), 36, 28, 190, "Z", "plastic"),
            _box(prefix + "anti_drop_rack", f"{side} {lr} anti drop rack", "66", (tower_x - sx * 68, y, 510 + z0), (20, 24, 332)),
            _box(prefix + "anti_drop_pawl", f"{side} {lr} primary anti drop pawl", "66", (tower_x - sx * 46, y + sy * 34, 390 + z0), (42, 18, 92)),
            _soft_box(prefix + "upper_limit_switch", f"{side} {lr} upper travel limit switch", "67/68", (tower_x + sx * 70, y, 690 + z0), (38, 18, 24), 3, material="electronics"),
            _soft_box(prefix + "lower_limit_switch", f"{side} {lr} lower travel limit switch", "67/68", (tower_x + sx * 70, y, 332 + z0), (38, 18, 24), 3, material="electronics"),
            _soft_box(prefix + "corner_load_pin_interface_module", f"{side} {lr} load-sensing pin amplifier and strain-relief module", "67/68", (tower_x + sx * 84, y, 430 + z0), (48, 38, 30), 3, material="electronics"),
            _cylinder(prefix + "load_pin_signal_harness_loop", f"{side} {lr} load pin signal harness loop", "67/68", (tower_x + sx * 60, y, 430 + z0), 4, 78, "X", "plastic"),
            _soft_box(prefix + "upper_polyurethane_bump_stop", f"{side} {lr} upper compression bump stop", "66", (tower_x + sx * 4, y, 688 + z0), (52, 52, 22), 6, material="rubber"),
            _soft_box(prefix + "lower_polyurethane_bump_stop", f"{side} {lr} lower rebound bump stop", "66", (tower_x + sx * 4, y, 328 + z0), (52, 52, 22), 6, material="rubber"),
            _cylinder(prefix + "bg75_wheel_drive_motor", f"{side} {lr} Dunkermotoren BG75-class wheel drive motor envelope", "69", (x, y - sy * 122, z_axle), 37.5, 96, "Y"),
            _cylinder(prefix + "plg75_wheel_planetary_gearbox", f"{side} {lr} PLG75-class planetary gearbox wheel reduction envelope", "69", (x, y - sy * 70, z_axle), 39, 58, "Y", "steel"),
            _soft_box(prefix + "deutsch_dtp_power_connector", f"{side} {lr} TE DEUTSCH DTP sealed power connector", "69", (tower_x + sx * 82, y + sy * 96, 610 + z0), (47.27, 27.15, 22.05), 3, material="plastic"),
            _soft_box(prefix + "deutsch_dt_signal_connector", f"{side} {lr} TE DEUTSCH DT sealed signal connector", "67/68", (tower_x + sx * 82, y - sy * 96, 650 + z0), (44.02, 22.25, 36.45), 3, material="plastic"),
            _cylinder(prefix + "corner_power_harness_conduit", f"{side} {lr} sealed power harness conduit", "69", (tower_x + sx * 40, y + sy * 96, 610 + z0), 7, 160, "X", "plastic"),
            _cylinder(prefix + "corner_signal_harness_conduit", f"{side} {lr} sealed signal harness conduit", "67/68", (tower_x + sx * 40, y - sy * 96, 620 + z0), 5, 160, "X", "plastic"),
            _soft_box(prefix + "harness_backbone_rail", f"{side} {lr} harness backbone rail tied to suspension tower", "67/68/69", (tower_x + sx * 96, y + sy * 116, 535 + z0), (28, 28, 390), 3, material="steel"),
            _soft_box(prefix + "harness_backbone_standoff_upper", f"{side} {lr} upper harness rail standoff to tower cassette", "67/68/69", (tower_x + sx * 48, y + sy * 58, 642 + z0), (118, 128, 18), 3, material="steel"),
            # Dropped 11 mm: at z 426 this 18 mm-thick standoff spanned
            # 417..435 and ran into the lower pushrod rod-end head, which
            # occupies 426.5..453.5. Now clears it by ~2.5 mm.
            _soft_box(prefix + "harness_backbone_standoff_lower", f"{side} {lr} lower harness rail standoff to tower cassette", "67/68/69", (tower_x + sx * 48, y + sy * 58, 415 + z0), (118, 128, 18), 3, material="steel"),
            _gusset(prefix + "tower_frame_gusset", f"{side} {lr} triangular tower to rail gusset", "61", (tower_x + sx * 42, y, 375 + z0), 165, 160, 8, sx),
        ]
    )

    # Enclosed corner shroud: the fixed (non-steering) ball-screw/motor/
    # gearhead/brake stack is otherwise a stack of bare cylinders/boxes.
    # This is a real hollow cover panel (see _shell_box), not decoration --
    # anti-pinch guarding around the moving ballnut carriage, and weather/
    # debris sealing over the exposed screw and motor stack.
    components.append(
        _shell_box(
            prefix + "corner_tower_shroud",
            f"{side} {lr} weather/anti-pinch cover over the corner actuator stack",
            "61/62/64/65/66",
            (tower_x, y, 630 + z0),
            (180, 180, 720),
            4.0,
            50.0,
            material="dark_aluminum",
        )
    )

    for motor_bolt_idx, (mx, my) in enumerate(((-28, -28), (-28, 28), (28, -28), (28, 28))):
        components.extend(_bolt_y(prefix + f"motor_mount_m5_bolt_{motor_bolt_idx}", (tower_x + mx, y + my, 748 + z0), 96, 5))

    for lug_idx, lug_angle in enumerate(range(0, 360, 15)):
        components.append(
            _tread_lug(
                prefix + f"tire_tread_lug_{lug_idx:02d}",
                f"{side} {lr} molded stair-grip tire tread lug",
                "60",
                (x, y, z_axle),
                wheel_d / 2,
                lug_angle,
                wheel_w + 10,
            )
        )

    for bolt_idx, bolt_angle in enumerate(range(0, 360, 30)):
        a = math.radians(bolt_angle)
        bolt_x = x + math.sin(a) * 52
        bolt_z = z_axle + math.cos(a) * 52
        components.extend(_bolt_y(prefix + f"wheel_hub_m6_flange_bolt_{bolt_idx:02d}", (bolt_x, y, bolt_z), wheel_w + 42, 6))
        components.append(_cylinder(prefix + f"wheel_hub_din985_m6_locknut_{bolt_idx:02d}", f"{side} {lr} wheel hub M6 locknut", "FASTENER", (bolt_x, y - sy * (wheel_w / 2 + 31), bolt_z), 8, 6, "Y", "black_oxide_steel"))

    for clip_idx, clip_z in enumerate((360, 410, 460, 510, 560, 610, 660, 710)):
        clip_x = tower_x + sx * 96
        clip_y = y + sy * 116
        clip_z = clip_z + z0
        components.append(_soft_box(prefix + f"hellermann_hdm312_harness_mount_{clip_idx}", f"{side} {lr} HellermannTyton HDM312 harness tie mount", "67/68/69", (clip_x, clip_y, clip_z), (36.3, 19.3, 16.7), 2, material="plastic"))
        components.extend(_bolt_y(prefix + f"harness_mount_m5_screw_{clip_idx}", (clip_x, clip_y, clip_z), 26, 5))

    for contact_idx, contact_y in enumerate((-9, 9)):
        components.append(_cylinder(prefix + f"dtp_size12_power_contact_{contact_idx}_positive", f"{side} {lr} DEUTSCH DTP size 12 power contact", "69", (tower_x + sx * 82, y + sy * (96 + contact_y), 615 + z0), 2.1, 23, "Z", "copper"))
        components.append(_cylinder(prefix + f"dtp_size12_power_contact_{contact_idx}_negative", f"{side} {lr} DEUTSCH DTP size 12 power contact", "69", (tower_x + sx * 82, y + sy * (96 + contact_y), 605 + z0), 2.1, 23, "Z", "copper"))
    components.append(_soft_box(prefix + "dtp_wedgelock", f"{side} {lr} DEUTSCH DTP wedgelock/contact retainer", "69", (tower_x + sx * 82, y + sy * 96, 596 + z0), (38, 18, 6), 1, material="plastic"))
    for contact_idx, contact_y in enumerate((-10.5, -3.5, 3.5, 10.5)):
        components.append(_cylinder(prefix + f"dt_size16_signal_contact_upper_{contact_idx}", f"{side} {lr} DEUTSCH DT size 16 signal contact", "67/68", (tower_x + sx * 82, y - sy * (96 + contact_y), 657 + z0), 1.3, 20, "Z", "copper"))
        components.append(_cylinder(prefix + f"dt_size16_signal_contact_lower_{contact_idx}", f"{side} {lr} DEUTSCH DT size 16 signal contact", "67/68", (tower_x + sx * 82, y - sy * (96 + contact_y), 643 + z0), 1.3, 20, "Z", "copper"))
    components.append(_soft_box(prefix + "dt_wedgelock", f"{side} {lr} DEUTSCH DT wedgelock/contact retainer", "67/68", (tower_x + sx * 82, y - sy * 96, 632 + z0), (36, 18, 5), 1, material="plastic"))

    for sensor_idx, (sensor_x, sensor_y, sensor_z) in enumerate(
        (
            (x + sx * 32, y + sy * 61, z_axle + 18),
            (x + sx * 56, y + sy * 61, z_axle + 18),
            # Coaxial pair, both driven along +Y. An M3 bolt here spans -11 to
            # +12 mm about its centre (washer behind, head in front), so the
            # two need at least 23 mm of separation or the head of one lands
            # inside the washer of the other. They were at +/-10. The pair at
            # index 4/5 below already sits at +/-12 and just clears; these now
            # sit at +/-13 for a little margin.
            (tower_x + sx * 58, y - 13, 472 + z0),
            (tower_x + sx * 58, y + 13, 472 + z0),
            (tower_x + sx * 84, y - 12, 430 + z0),
            (tower_x + sx * 84, y + 12, 430 + z0),
        )
    ):
        components.extend(_bolt_y(prefix + f"sensor_mount_m3_screw_{sensor_idx}", (sensor_x, sensor_y, sensor_z), 18, 3))

    for i, z_mount in enumerate((370, 610)):
        z_mount = z_mount + z0
        components.extend(_bolt_y(prefix + f"tower_lug_m8_{i}_upper", (tower_x - 32, rail_y, z_mount + 18), 76))
        components.extend(_bolt_y(prefix + f"tower_lug_m8_{i}_lower", (tower_x + 32, rail_y, z_mount - 18), 76))

    for i, z_fastener in enumerate((350, 430, 510, 590, 670)):
        z_fastener = z_fastener + z0
        components.extend(_bolt_y(prefix + f"guide_rail_a_m5_{i}", (tower_x - 24, y + sy * 46, z_fastener), 28, 5))
        components.extend(_bolt_y(prefix + f"guide_rail_b_m5_{i}", (tower_x + 24, y - sy * 46, z_fastener), 28, 5))

    for i, z_block in enumerate((300, 725)):
        z_block = z_block + z0
        components.extend(_bolt_y(prefix + f"thrust_block_m8_a_{i}", (tower_x - 28, y, z_block), 110))
        components.extend(_bolt_y(prefix + f"thrust_block_m8_b_{i}", (tower_x + 28, y, z_block), 110))

    for i, pin_z in enumerate((z_axle + 105, z_axle + 250, 360 + z0, 515 + z0)):
        pin_x = x if i < 2 else tower_x
        # Each of these pivots previously carried TWO pins on the same axis at
        # the same point: a plain M10 link bolt and a separate load-sensing pin
        # element. They occupied the same hole -- 16 of the assembly's last 21
        # interferences. The sourced part register is explicit that the
        # Strainsert load pins "replace clevis/shear pins directly", so the
        # instrumented pin IS the pivot pin and the plain bolt was redundant.
        #
        # Sizing also corrected: both former pins were Ø10 inside an iglidur
        # bushing bored Ø21, leaving 5.5 mm of radial slop in what is supposed
        # to be a bearing fit. The pin is now Ø20.8, a 0.1 mm radial clearance
        # in that bore, and long enough to reach the DIN 985 locknut that
        # retains it rather than stopping short in mid-air.
        components.append(_tube(prefix + f"iglidur_g_pivot_bushing_{i}", f"{side} {lr} iglidur G pivot bushing sleeve", "60", (pin_x, y, pin_z), 15, 10.5, 104, "Y", "plastic"))
        components.append(_tube(prefix + f"aurora_m10_rod_end_outer_race_{i}", f"{side} {lr} Aurora AM-M10T rod-end bearing race at suspension pivot", "60", (pin_x, y + sy * 49, pin_z), 13.5, 5.0, 14.0, "Y", "steel"))
        components.append(_cylinder(prefix + f"load_sensing_pivot_pin_{i}", f"{side} {lr} Strainsert load-sensing suspension pivot pin", "67/68", (pin_x, y, pin_z), 10.4, 156, "Y", "steel"))
        components.append(_cylinder(prefix + f"din985_m10_locknut_{i}", f"{side} {lr} M10 prevailing torque lock nut at suspension pin", "FASTENER", (pin_x, y - sy * 84, pin_z), 12, 10, "Y", "black_oxide_steel"))

    for i, (px, pz) in enumerate(((tower_x + sx * 45, 440 + z0), (x, z_axle + 278))):
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
                _box(prefix + "rear_module_mount_bracket", f"{side} {lr} rear module mounting bracket", "70", (x - sx * 122, y, 332 + z0), (30, 194, 184)),
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


#: Interference-resolution priority, highest first. When two solids overlap,
#: the LOWER-priority one is cut so the pair meets on a real mating surface
#: instead of sharing volume. The ordering is functional, not arbitrary:
#: purchased parts and fasteners cannot be machined to suit, so everything
#: fabricated yields to them; brackets and covers yield to primary structure.
#: A fastener cutting a plate is exactly a clearance hole, which is what the
#: assembly was missing entirely.
_PRIORITY_RULES: tuple[tuple[int, tuple[str, ...]], ...] = (
    # Fasteners rank ABOVE everything, including purchased parts. A bolt is not
    # a part competing for space -- it is the definition of a hole in whatever
    # it passes through, so it must always be the cutter and never the cut.
    # Ranking them below purchased components carved the shanks off every bolt
    # threading into a motor or bearing housing.
    (200, ("bolt", "screw", "washer", "locknut", "_nut", "nut_", "shaft_collar")),
    # Purchased components: dimensionally fixed by the supplier.
    (100, ("bnk2010", "hsr15", "fk12", "ff12", "maxon", "ec_i_52", "gpx52", "ab44",
           "bg75", "plg75", "electrak", "o1d100", "tf1", "rfc", "novotechnik",
           "deutsch", "littelfuse", "orion", "iglidur", "igubal", "aurora",
           "strainsert", "hellermann", "thrust_block", "bearing", "encoder",
           "battery_pack", "bms", "charger", "dc_dc", "contactor", "fuse")),
    # Rotating/ground-contact hardware.
    (80, ("tire_", "hub_", "wheel_", "rim_", "tread_lug", "kingpin", "pivot")),
    # Primary structure.
    (70, ("chassis", "main_rail", "crossmember", "cassette", "tower", "frame_",
          "swingarm", "fork_", "trunnion", "hinge")),
    # Secondary structure.
    (50, ("bracket", "mount", "gusset", "clevis", "yoke", "lug", "standoff",
          "plate", "bridge", "carriage", "rail")),
    # Enclosures and trim yield to everything they wrap.
    (30, ("shell", "cover", "shroud", "panel", "fender", "pad", "seat_")),
    # Routing and soft goods yield to all hardware.
    (10, ("harness", "cable", "loom", "grommet", "tie")),
)

#: If cutting would remove more than this fraction of a part, the pair is a
#: layout fault (two things designed into the same space), not a mating detail.
#: Those are reported rather than silently carved away.
_MAX_CUT_FRACTION = 0.60


def _priority(name: str) -> int:
    lowered = name.lower()
    for rank, tokens in _PRIORITY_RULES:
        if any(t in lowered for t in tokens):
            return rank
    return 40  # unclassified fabricated part


def resolve_interferences(components: list[Component]) -> tuple[list[Component], list[dict]]:
    """Cut overlapping solids so no two parts occupy the same volume.

    Returns the resolved components and a list of unresolved layout faults --
    pairs where cutting would destroy the smaller part, which means the two
    were designed into the same space and need moving, not carving.
    """
    shapes = {}
    boxes = {}
    volumes = {}
    for c in components:
        s = c.shape.val()
        shapes[c.name] = s
        boxes[c.name] = s.BoundingBox()
        volumes[c.name] = s.Volume()

    names = [c.name for c in components]
    faults: list[dict] = []

    # DECIDE first, APPLY once. Cutting a part pair-by-pair as clashes were
    # found made every subsequent boolean on that part slower, because each cut
    # complicates its B-rep -- the chassis was re-cut by several hundred bolts
    # one at a time. That took ~15 hours. Deciding every cut against the
    # ORIGINAL geometry and then applying one batched cut per part does the
    # same work in a small fraction of the boolean operations.
    #
    # Deciding on original geometry is also conservative in the right
    # direction: cuts only ever remove material, so a decision made against an
    # uncut tool can only over-cut slightly, never leave residual overlap.
    cutters: dict[str, list[str]] = {}

    n = len(names)
    for i in range(n):
        a = names[i]
        ba = boxes[a]
        for j in range(i + 1, n):
            b = names[j]
            bb_ = boxes[b]
            if not (ba.xmin < bb_.xmax and bb_.xmin < ba.xmax
                    and ba.ymin < bb_.ymax and bb_.ymin < ba.ymax
                    and ba.zmin < bb_.zmax and bb_.zmin < ba.zmax):
                continue
            try:
                inter = shapes[a].intersect(shapes[b])
                vol = inter.Volume() if inter is not None else 0.0
            except Exception:  # noqa: BLE001
                continue
            if vol <= 1.0:
                continue

            pa, pb = _priority(a), _priority(b)
            if pa < pb:
                low, high = a, b
            elif pb < pa:
                low, high = b, a
            else:
                low, high = (a, b) if a < b else (b, a)  # deterministic tie-break
            frac_low = vol / volumes[low] if volumes[low] > 0 else 1.0
            frac_high = vol / volumes[high] if volumes[high] > 0 else 1.0

            # Normally the lower-priority part yields. But if that would carve
            # away most of it, the small part is EMBEDDED in the large one --
            # a contact seated in a connector body, a guide block wrapping its
            # rail, a bearing in its housing. The physically correct answer
            # there is a cavity in the larger part, not a mutilated small one,
            # so the cut direction reverses. Only when neither part can absorb
            # the cut are the two genuinely designed into the same space.
            if frac_low <= _MAX_CUT_FRACTION:
                victim, tool = low, high
            elif frac_high <= _MAX_CUT_FRACTION:
                victim, tool = high, low
            else:
                faults.append({
                    "part": low,
                    "clashes_with": high,
                    "fraction_of_part": round(frac_low, 3),
                    "fraction_of_other": round(frac_high, 3),
                    "volume_mm3": round(vol, 1),
                })
                continue
            cutters.setdefault(victim, []).append(tool)

    for victim, tools in cutters.items():
        try:
            cut = shapes[victim].cut(*[shapes[t] for t in tools])
            if cut is not None and cut.Volume() > 1.0:
                shapes[victim] = cut
        except Exception:  # noqa: BLE001
            # Fall back to one-at-a-time for this part only, so a single
            # problematic boolean cannot abandon all of its cuts.
            for t in tools:
                try:
                    cut = shapes[victim].cut(shapes[t])
                    if cut is not None and cut.Volume() > 1.0:
                        shapes[victim] = cut
                except Exception:  # noqa: BLE001
                    continue

    resolved = [
        Component(c.name, c.role, c.callout, cq.Workplane(obj=shapes[c.name]), c.material)
        for c in components
    ]
    return resolved, faults


def build_components(params: dict, include_reference: bool = False,
                     resolve: bool = True) -> list[Component]:
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
    if resolve:
        components, _ = resolve_interferences(components)
    return components


def _is_folding_handle_group(component: Component) -> bool:
    """Parts that physically move with the push handle when it folds down
    about fold_hinge_axis_left/right -- not the hinge itself or its
    chassis-side mounting brackets/backstays, which stay fixed."""
    name = component.name.lower()
    return name.startswith(
        (
            "handle_left_upright",
            "handle_right_upright",
            "handle_grip_deadman_bar",
            "handle_left_lower_yoke_link",
            "handle_right_lower_yoke_link",
            "deadman_release_paddle",
            "deadman_paddle_hinge_tab",
            "handle_grip_power_button",
            "handle_grip_assist_mode_button",
            "handle_haptic_feedback_motor",
            "handle_status_indicator_led",
        )
    )


def _is_removable_pod_group(component: Component) -> bool:
    """The car-seat-style removable pod carrier: everything that lifts off
    with it when the caregiver releases pod_release_latch_pin_0/1, as
    opposed to the chassis-side trunnion stands and latch receivers, which
    stay behind."""
    name = component.name.lower()
    if "trunnion" in name:
        return False
    return name.startswith("seat_") or name in ("mechanical_level_lock", "five_point_harness_bar")


def build_folded_pose_components(params: dict) -> list[Component]:
    """Return a visual kinematic pose: the pod removed (car-seat/travel-
    system style, via pod_release_latch_pin_0/1) and the folding handle
    assembly rotated down about its real modeled hinge axis
    (fold_hinge_axis_left/right); everything else held at rest position.

    This is a partial fold: the corner suspension towers have no parked/
    retracted pose modeled yet, so the reported envelope is a conservative
    (upper-bound) estimate for the pod-removed, handle-folded chassis -- a
    design that also parks the corner actuators would only get smaller, not
    bigger, than what this pose reports.
    """
    g = params["geometry"]
    hinge_pivot = (-540.0, 0.0, g["frame_z_mm"] + 95.0)
    fold_angle_deg = 100.0
    posed: list[Component] = []
    for component in build_components(params, include_reference=False):
        if _is_removable_pod_group(component):
            continue
        if _is_folding_handle_group(component):
            posed.append(_rotated_y(component, fold_angle_deg, hinge_pivot, "_folded_pose"))
        else:
            posed.append(component)
    return posed


def folded_pose_bounding_box_mm(params: dict) -> list[float]:
    compound = _compound(build_folded_pose_components(params))
    bb = compound.BoundingBox()
    return [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]


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


def export_isometric_views(params: dict, out_dir: Path) -> None:
    """Real vector-projection renders of the actual CAD geometry (SVG, so it
    works headless with no display/GPU) -- not a mockup or concept render."""
    out_dir.mkdir(parents=True, exist_ok=True)
    view_opt = {
        "width": 1600,
        "height": 1200,
        "projectionDir": (1, -1, 0.75),
        "showAxes": False,
        "showHidden": False,
        "strokeWidth": 0.4,
    }
    exporters.export(
        _compound(build_components(params, include_reference=False)),
        str(out_dir / "Terrain_Elevate_P1_V0_59_isometric.svg"),
        exportType="SVG",
        opt=view_opt,
    )
    exporters.export(
        _compound(build_stair_climb_pose_components(params)),
        str(out_dir / "Terrain_Elevate_P1_V0_59_stair_climb_pose_isometric.svg"),
        exportType="SVG",
        opt=view_opt,
    )
    exporters.export(
        _compound(build_folded_pose_components(params)),
        str(out_dir / "Terrain_Elevate_P1_V0_59_folded_pose_isometric.svg"),
        exportType="SVG",
        opt=view_opt,
    )


def export_model(params: dict, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    g = params["geometry"]
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
                "part": "THK BNK2010-2.5RRG2+499LC7Y",
                "modeled_as": "20 mm screw shaft, 499 mm shaft length, 300 mm travel datum, fixed/support bearing blocks",
                "source_values": {
                    "stroke_mm": 300.0,
                    "lead_mm": 10.0,
                    "dynamic_load_rating_kN": 11.1,
                    "static_load_rating_kN": 22.0,
                    "recommended_fixed_support": "EK12 or FK12",
                    "recommended_supported_support": "EF12 or FF12",
                },
            },
            {
                "callout": "62",
                "part": "THK FK12/FF12 screw support unit family",
                "modeled_as": "fixed and supported end bearing housings tied into the suspension tower",
                "source_values": {
                    "ball_screw_shaft_outer_diameter_mm": 20.0,
                    "recommended_by": "THK BNK2010-2.5RRG2+499LC7Y detail specification",
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
                "part": "maxon EC-i 52 part 633919 screened actuator motor",
                "modeled_as": "52 mm diameter motor envelope with gearhead/brake stack",
                "source_values": {
                    "diameter_mm": 52.0,
                    "nominal_power_W": 420.0,
                    "nominal_torque_Nm": 1.02,
                    "nominal_speed_rpm": 3990.0,
                    "source_status": "exact_catalog_part",
                },
            },
            {
                "callout": "65",
                "part": "maxon GPX52 UP 3.9:1 one-stage planetary gearhead",
                "modeled_as": "52 mm diameter, 50.2 mm long coaxial gearhead envelope in each corner actuator stack",
                "source_values": {
                    "ratio": 3.9,
                    "diameter_mm": 52.0,
                    "length_mm": 50.2,
                    "max_continuous_torque_Nm": 7.5,
                    "max_efficiency": 0.95,
                    "max_input_speed_rpm": 6000,
                    "source_status": "configured_to_order_catalog_product",
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
                    "source_status": "configured_to_order_catalog_product",
                },
            },
            {
                "callout": "66",
                "part": "maxon AB 44 24 VDC 2.5 Nm holding brake, part 386054",
                "modeled_as": "44 mm diameter, 26.9 mm long normally-engaged motor-side brake envelope at each suspension actuator",
                "source_values": {
                    "holding_torque_Nm": 2.5,
                    "calculated_motor_side_required_holding_torque_Nm_with_safety_factor": 0.331,
                    "nominal_voltage_VDC": 24.0,
                    "unpowered_state": "braked",
                    "not_for_dynamic_braking": True,
                    "source_status": "catalog part; must be ordered/assembled in compatible maxon motor combination",
                },
            },
            {
                "callout": "69",
                "part": "Dunkermotoren BG75 dMove BLDC motor with configured PLG75 planetary gearbox",
                "modeled_as": "75 mm class wheel drive motor plus PLG75-class planetary wheel reduction at each wheel",
                "source_values": {
                    "screen_requirement_per_wheel_torque_Nm": 53.8,
                    "manufacturer_family_power_range_W": "BG family includes high-power BLDC gearmotor variants; BG75 dMove class up to 810 W per official press/spec summary",
                    "planetary_gearbox_family_continuous_torque_Nm": "PLG family up to 130 Nm",
                    "source_status": "configured_to_order_catalog_product",
                },
            },
            {
                "callout": "68/69",
                "part": "NXP S32K3 / STM32G474 / TI DRV8353 screened control and gate-driver families",
                "modeled_as": "electronics housings with hardwired inhibit/watchdog and local motor-control package envelopes",
            },
            {
                "callout": "67",
                "part": "ifm O1D100 IO-Link photoelectric distance sensor",
                "modeled_as": "front and rear 59 x 42 x 52 mm stair preview distance sensor bodies on bridge brackets",
                "source_values": {
                    "measuring_range_m": "0.2-10",
                    "protection": "IP67",
                    "connector": "M12 A-coded 5 pin",
                    "source_status": "exact_catalog_part",
                },
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
                "callout": "67/69",
                "part": "Novotechnik RFC-4800 sealed 360 degree non-contact wheel angle/speed sensor configuration",
                "modeled_as": "magnetic target ring on each wheel hub plus fixed pickup bracket and sealed sensor on each fork",
                "source_values": {
                    "wheel_encoder_count": 4,
                    "range_deg": 360,
                    "protection_options": "IP67/IP68/IP69 by configuration",
                    "purpose": "wheel speed feedback for coordinated traction and stair-phase advance control",
                    "source_status": "configured_to_order_catalog_product",
                },
            },
            {
                "callout": "67/68",
                "part": "Novotechnik TF1 inductive absolute linear position transducer",
                "modeled_as": "linear scale on each suspension tower with read head attached to the moving ballnut slider",
                "source_values": {
                    "axis_count": 4,
                    "range_mm": "100-1000 by configuration",
                    "protection": "IP67",
                    "interfaces": "IO-Link/CANopen/SSI/Voltage/Current by configuration",
                    "purpose": "closed-loop independent corner height control",
                    "source_status": "configured_to_order_catalog_product",
                },
            },
            {
                "callout": "67/68",
                "part": "Strainsert CPA/CBA load pin/load bolt configuration",
                "modeled_as": "load-sensing pin elements at suspension pivots with sealed signal amplifier/strain-relief module per corner",
                "source_values": {
                    "sensed_corner_modules": 4,
                    "purpose": "load sharing, contact detection, and whole-stroller response trigger",
                    "source_status": "configured_to_order_real_product",
                },
            },
            {
                "callout": "60",
                "part": "Molded pneumatic stair-grip tire tread lug pattern",
                "modeled_as": f"twelve molded rubber tread lugs per {g['wheel_diameter_mm']:.0f} x {g['wheel_width_mm']:.0f} mm tire, four ground-contact wheels only",
                "source_values": {
                    "tread_lugs_per_wheel": 12,
                    "ground_contact_wheel_count": 4,
                    "added_helper_wheels": 0,
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
