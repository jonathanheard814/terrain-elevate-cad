#!/usr/bin/env python3
# Terrain Elevate V0.59 FreeCAD/OpenCascade CAD Factory
#
# Run with:
#   freecadcmd scripts/build_te_v059_freecad.py
#
# Outputs:
#   cad_out/Terrain_Elevate_V0_59_native.FCStd
#   cad_out/Terrain_Elevate_V0_59.step
#   cad_out/Terrain_Elevate_V0_59_manifest.json
#
# Truth boundary:
#   Native FreeCAD/OpenCascade solids and STEP export when executed in FreeCAD.
#   This is not FEA, dynamic contact simulation, certification, or physical testing.

import os, json, math, sys, traceback
from pathlib import Path

import FreeCAD as App
import Part
import Import

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "cad_out"
OUT.mkdir(exist_ok=True)

PARAMS = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
G = PARAMS["geometry"]

DOC = App.newDocument("Terrain_Elevate_V0_59_native")

def v(x, y, z):
    return App.Vector(float(x), float(y), float(z))

def add_part(name, label=None):
    part = DOC.addObject("App::Part", name)
    part.Label = label or name
    return part

def set_meta(obj, callout, purpose, classification="ASM"):
    obj.addProperty("App::PropertyString", "TE_Callout", "TerrainElevate", "Visual label group").TE_Callout = str(callout)
    obj.addProperty("App::PropertyString", "TE_Purpose", "TerrainElevate", "Functional purpose").TE_Purpose = purpose
    obj.addProperty("App::PropertyString", "TE_Class", "TerrainElevate", "Value/source class").TE_Class = classification

def add_box(parent, name, center, size, callout, purpose, classification="ASM"):
    sx, sy, sz = [float(i) for i in size]
    cx, cy, cz = [float(i) for i in center]
    shape = Part.makeBox(sx, sy, sz, v(cx-sx/2, cy-sy/2, cz-sz/2))
    obj = DOC.addObject("Part::Feature", name)
    obj.Shape = shape
    obj.Label = name
    set_meta(obj, callout, purpose, classification)
    if parent:
        parent.addObject(obj)
    return obj

def add_cylinder(parent, name, axis, center, radius, length, callout, purpose, classification="ASM"):
    cx, cy, cz = [float(i) for i in center]
    radius = float(radius); length = float(length)
    axis = axis.upper()
    if axis == "X":
        shape = Part.makeCylinder(radius, length, v(cx-length/2, cy, cz), v(1,0,0))
    elif axis == "Y":
        shape = Part.makeCylinder(radius, length, v(cx, cy-length/2, cz), v(0,1,0))
    elif axis == "Z":
        shape = Part.makeCylinder(radius, length, v(cx, cy, cz-length/2), v(0,0,1))
    else:
        raise ValueError("Bad axis " + str(axis))
    obj = DOC.addObject("Part::Feature", name)
    obj.Shape = shape
    obj.Label = name
    set_meta(obj, callout, purpose, classification)
    if parent:
        parent.addObject(obj)
    return obj

def add_joint_marker(parent, name, axis, center, length, callout, purpose):
    # Thin construction-axis solid so the joint datum survives STEP export.
    return add_cylinder(parent, name, axis, center, 3.0, length, callout, purpose, "DATUM")

def build_chassis():
    part = add_part("00_CHASSIS_FRAME", "00_CHASSIS_FRAME")
    wb, tr, z0 = G["wheelbase_mm"], G["track_mm"], G["frame_z_mm"]
    add_box(part, "CHASSIS_left_lower_box_rail", (0,tr/2+55,z0), (920,38,48), "1-12", "left lower structural rail")
    add_box(part, "CHASSIS_right_lower_box_rail", (0,-tr/2-55,z0), (920,38,48), "1-12", "right lower structural rail")
    add_box(part, "CHASSIS_front_crossmember", (wb/2+50,0,z0), (70,790,48), "1-12", "front crossmember")
    add_box(part, "CHASSIS_rear_crossmember", (-wb/2-50,0,z0), (70,790,48), "1-12", "rear crossmember")
    add_box(part, "CHASSIS_mid_battery_crossmember", (0,0,z0-18), (90,610,38), "1-12", "battery support crossmember")
    add_cylinder(part, "CHASSIS_fold_hinge_axis_left", "Y", (0,tr/2+55,z0+65), 18, 120, "1-12", "fold hinge bearing envelope")
    add_cylinder(part, "CHASSIS_fold_hinge_axis_right", "Y", (0,-tr/2-55,z0+65), 18, 120, "1-12", "fold hinge bearing envelope")
    add_box(part, "CHASSIS_primary_latch_left", (90,tr/2+55,z0+92), (130,58,38), "1-12", "primary structural latch")
    add_box(part, "CHASSIS_primary_latch_right", (-90,-tr/2-55,z0+92), (130,58,38), "1-12", "primary structural latch")
    add_box(part, "CHASSIS_rear_module_mount_plate_L", (-wb/2-78,tr/2,z0), (18,170,160), "70", "rear module mounting load path")
    add_box(part, "CHASSIS_rear_module_mount_plate_R", (-wb/2-78,-tr/2,z0), (18,170,160), "70", "rear module mounting load path")
    return part

def build_seat():
    part = add_part("01_SEAT_LEVELING_CHILD_POD", "01_SEAT_LEVELING_CHILD_POD")
    items = [
        ("SEAT_outer_roll_frame_front",(190,0,640),(38,480,48),"13-26","roll-leveling frame"),
        ("SEAT_outer_roll_frame_rear",(-190,0,640),(38,480,48),"13-26","roll-leveling frame"),
        ("SEAT_outer_roll_frame_left",(0,220,640),(420,38,48),"13-26","roll-leveling frame"),
        ("SEAT_outer_roll_frame_right",(0,-220,640),(420,38,48),"13-26","roll-leveling frame"),
        ("SEAT_pitch_inner_frame_front",(145,0,710),(34,390,42),"13-26","pitch-leveling frame"),
        ("SEAT_pitch_inner_frame_rear",(-145,0,710),(34,390,42),"13-26","pitch-leveling frame"),
        ("SEAT_child_pod_back",(-165,0,835),(66,390,315),"13-26","child back support shell envelope"),
        ("SEAT_child_pod_pan",(20,0,720),(400,330,62),"13-26","child seat pan envelope"),
        ("SEAT_child_left_sidewall",(20,190,830),(360,44,220),"13-26","child side retention"),
        ("SEAT_child_right_sidewall",(20,-190,830),(360,44,220),"13-26","child side retention"),
        ("SEAT_harness_anchor_bar",(70,0,835),(260,28,34),"13-26","five-point harness anchoring"),
        ("SEAT_positive_center_lock",(225,0,657),(130,62,44),"13-26","seat leveling lock"),
        ("SEAT_no_power_level_hold_lock",(-225,0,657),(130,62,44),"13-26","mechanical no-power hold"),
    ]
    for name, center, size, callout, purpose in items:
        add_box(part, name, center, size, callout, purpose)
    add_cylinder(part, "SEAT_roll_axis_left", "Y", (0,270,640), 28, 52, "13-26", "roll bearing envelope")
    add_cylinder(part, "SEAT_roll_axis_right", "Y", (0,-270,640), 28, 52, "13-26", "roll bearing envelope")
    add_cylinder(part, "SEAT_pitch_axis_front", "X", (210,0,710), 23, 56, "13-26", "pitch bearing envelope")
    add_cylinder(part, "SEAT_pitch_axis_rear", "X", (-210,0,710), 23, 56, "13-26", "pitch bearing envelope")
    return part

def build_power_control():
    part = add_part("02_POWER_CONTROL_SAFETY_ELECTRONICS", "02_POWER_CONTROL_SAFETY_ELECTRONICS")
    items = [
        ("POWER_48V_battery_pack_envelope",(-55,0,430),(315,165,118),"41-59","battery pack envelope"),
        ("POWER_BMS_fuse_contactor_disconnect",(180,0,430),(225,112,52),"41-59","BMS/fuse/contactor/service disconnect"),
        ("CTRL_S32K344_safety_supervisor",(40,212,530),(140,100,42),"68","independent safety supervisor"),
        ("CTRL_STM32G474_motor_control_bank",(40,-212,532),(145,108,58),"68","motor control domain"),
        ("PWR_DRV8363_left_inverter_bank",(-220,185,500),(150,82,46),"69","left branch inverter bank"),
        ("PWR_DRV8363_right_inverter_bank",(-220,-185,500),(150,82,46),"69","right branch inverter bank"),
    ]
    for name, center, size, callout, purpose in items:
        add_box(part, name, center, size, callout, purpose)
    return part

def build_handle_sensor_refs():
    handle = add_part("03_HANDLE_INTERFACE", "03_HANDLE_INTERFACE")
    add_cylinder(handle, "HANDLE_left_upright", "Z", (-540,285,835), 17, 585, "31-36", "handle support")
    add_cylinder(handle, "HANDLE_right_upright", "Z", (-540,-285,835), 17, 585, "31-36", "handle support")
    add_cylinder(handle, "HANDLE_grip_crossbar_with_deadman", "Y", (-540,0,1135), 20, 620, "31-36", "user grip and deadman")
    add_box(handle, "HANDLE_left_force_sensor", (-540,175,1135), (66,54,34), "31-36", "user input force sensing")
    add_box(handle, "HANDLE_right_force_sensor", (-540,-175,1135), (66,54,34), "31-36", "user input force sensing")
    add_box(handle, "HANDLE_deadman_release_paddle", (-495,0,1192), (115,13,58), "31-36", "deadman safety release")

    sensors = add_part("04_SENSOR_ARRAY", "04_SENSOR_ARRAY")
    add_box(sensors, "SENSOR_front_depth_lidar", (555,0,675), (126,56,50), "67", "front stair/terrain sensing")
    add_box(sensors, "SENSOR_rear_depth_lidar", (-555,0,675), (126,56,50), "67", "rear stair/terrain sensing")
    for sx in [1,-1]:
        for sy in [1,-1]:
            add_box(sensors, "SENSOR_corner_step_edge_%s%s" % ("F" if sx>0 else "R","L" if sy>0 else "R"),
                    (sx*385, sy*405, 515), (42,28,42), "67", "corner step edge sensing")
    add_box(sensors, "SENSOR_IMU_dual_mount", (0,0,586), (70,42,24), "67", "body attitude sensing")

    refs = add_part("99_REFERENCE_GEOMETRY", "99_REFERENCE_GEOMETRY")
    add_box(refs, "REF_864mm_clear_stair_width_plate", (0,0,-20), (1160,864,8), "REQ", "verification stair width datum", "REF")
    add_box(refs, "REF_750mm_outer_width_gate", (0,0,-8), (1160,G["overall_width_gate_mm"],5), "REQ", "outer packaging width datum", "REF")
    rise = G["stair_rise_reference_mm"]; run = G["stair_going_reference_mm"]
    for i in range(5):
        add_box(refs, "REF_stair_%02d_203R_279G" % (i+1), (450+i*run,0,i*rise/2), (run,864,(i+1)*rise), "REQ", "stair geometry datum", "REF")

def build_corner(code, sx, sy):
    part = add_part("10_CORNER_%s_STEER_SUSPENSION_CASSETTE" % code, "10_CORNER_%s_STEER_SUSPENSION_CASSETTE" % code)
    wb, tr = G["wheelbase_mm"], G["track_mm"]
    x = sx*wb/2
    y = sy*tr/2
    tower_x = x - sx*95

    add_cylinder(part, code+"_WHEEL_280x70_tire", "Y", (x,y,140), G["wheel_diameter_mm"]/2, G["wheel_width_mm"], "60", "rolling wheel tire envelope")
    add_cylinder(part, code+"_WHEEL_hub_shell", "Y", (x,y,140), 63, 96, "60", "wheel hub load path")
    add_cylinder(part, code+"_WHEEL_live_axle", "Y", (x,y,140), 8.5, 132, "60", "wheel axle")
    add_box(part, code+"_FORK_left_plate", (x,y+sy*55,265), (110,18,188), "60", "wheel fork structural plate")
    add_box(part, code+"_FORK_right_plate", (x,y-sy*55,265), (110,18,188), "60", "wheel fork structural plate")
    add_box(part, code+"_FORK_bridge", (x,y,365), (126,128,34), "60", "fork bridge load path")

    add_joint_marker(part, code+"_JOINT_AXIS_STEER_KINGPIN_Z", "Z", (x,y,440), 300, "60", "revolute steering axis datum")
    add_cylinder(part, code+"_STEER_kingpin_bearing_stack", "Z", (x,y,440), 22, 270, "60", "kingpin bearing envelope")
    add_box(part, code+"_STEER_positive_center_lock", (x,y,585), (92,68,34), "66", "steering/trim center lock")

    add_box(part, code+"_SUSP_tower_cassette", (tower_x,y,505), (88,76,430), "61", "suspension tower load path")
    add_joint_marker(part, code+"_JOINT_AXIS_SWING_ARM_PIVOT_Y", "Y", (x-sx*104,y,310), 170, "60", "swing arm pivot datum")
    add_cylinder(part, code+"_SUSP_swingarm_pivot_shaft_25mm", "Y", (x-sx*104,y,310), G["swingarm_pivot_diameter_mm"]/2, 170, "60", "25 mm swingarm pivot shaft")
    add_box(part, code+"_SUSP_boxed_swing_arm_lower", (x-sx*70,y,310), (230,40,36), "60", "boxed lower swing arm")
    add_box(part, code+"_SUSP_upper_reaction_link", (x-sx*68,y,452), (205,30,30), "60", "upper reaction link")
    add_cylinder(part, code+"_SUSP_passive_spring_damper", "Z", (tower_x+sx*40,y,508), 18, 270, "61", "passive spring/damper envelope")

    add_joint_marker(part, code+"_JOINT_AXIS_BALLSCREW_Z", "Z", (tower_x,y,510), 430, "62", "ball screw slider datum")
    add_cylinder(part, code+"_SUSP_BNK1404_ball_screw", "Z", (tower_x,y,510), G["ball_screw_diameter_mm_SRC"]/2, 390, "62", "THK BNK1404 ball screw envelope", "SRC")
    add_box(part, code+"_SUSP_ballnut_carriage_bridge", (tower_x,y,470), (80,100,54), "62", "ballnut carriage bridge")
    add_box(part, code+"_SUSP_linear_guide_rail_A", (tower_x-24,y+sy*30,505), (16,12,360), "63", "linear guide rail")
    add_box(part, code+"_SUSP_linear_guide_rail_B", (tower_x+24,y-sy*30,505), (16,12,360), "63", "linear guide rail")
    add_box(part, code+"_SUSP_linear_guide_block_A", (tower_x-24,y+sy*30,470), (34,56.6,28), "63", "linear guide carriage block")
    add_box(part, code+"_SUSP_linear_guide_block_B", (tower_x+24,y-sy*30,470), (34,56.6,28), "63", "linear guide carriage block")

    add_cylinder(part, code+"_MOTOR_ECi40_100W_48V_envelope", "Y", (tower_x,y+sy*100,725), 20, 80, "64", "maxon EC-i 40 motor envelope")
    add_cylinder(part, code+"_GEAR_GPX42_12to1_envelope", "Y", (tower_x,y+sy*48,725), 21, 56, "65", "GPX42 12:1 gearhead envelope")
    add_cylinder(part, code+"_BRAKE_AB28_envelope", "Y", (tower_x,y+sy*145,725), 18, 30, "66", "power-off holding brake")
    add_box(part, code+"_SAFETY_anti_drop_rack", (tower_x-sx*66,y,505), (20,24,330), "66", "anti-drop rack")
    add_box(part, code+"_SAFETY_pawl_body_primary", (tower_x-sx*44,y+sy*32,385), (42,18,92), "66", "anti-drop pawl body")
    add_box(part, code+"_SENSOR_AS5600_screw_encoder", (tower_x+sx*48,y,690), (30,30,18), "67", "screw position sensing")
    add_box(part, code+"_SENSOR_pawl_state_inductive", (tower_x-sx*88,y+sy*52,385), (26,18,18), "67", "pawl state sensing")

    add_cylinder(part, code+"_DRIVE_wheel_coaxial_motor", "Y", (x,y-sy*92,140), 36, 80, "69", "wheel drive motor envelope")
    if sx < 0:
        add_cylinder(part, code+"_REAR_antirollback_dog_lock", "Y", (x,y+sy*92,140), 42, 24, "66/70", "rear anti-rollback lock")
        add_box(part, code+"_REAR_callout70_mounting_bracket", (x-sx*120,y,330), (28,190,180), "70", "rear module mounting bracket")
        for ix in [-1,0,1]:
            for iz in [-1,1]:
                add_cylinder(part, code+"_REAR_M8_mount_bolt_%s_%s"%(ix,iz), "X", (x-sx*134,y+ix*45,330+iz*45), 4, 38, "70", "rear module M8 mounting bolt")
    else:
        add_cylinder(part, code+"_FRONT_service_disc_brake", "Y", (x,y+sy*92,140), 42, 24, "60", "front service brake envelope")

def build_all():
    build_chassis()
    build_seat()
    build_power_control()
    build_handle_sensor_refs()
    for code, sx, sy in [("FL",1,1),("FR",1,-1),("RL",-1,1),("RR",-1,-1)]:
        build_corner(code, sx, sy)

def analyze_doc():
    DOC.recompute()
    solids = []
    total_volume = 0.0
    bbox = None
    for obj in DOC.Objects:
        if hasattr(obj, "Shape") and not obj.Shape.isNull():
            vol = obj.Shape.Volume
            total_volume += vol
            solids.append({
                "name": obj.Name,
                "label": obj.Label,
                "volume_mm3": vol,
                "callout": getattr(obj, "TE_Callout", ""),
                "purpose": getattr(obj, "TE_Purpose", ""),
                "class": getattr(obj, "TE_Class", "")
            })
            bb = obj.Shape.BoundBox
            if bbox is None:
                bbox = [bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax]
            else:
                bbox = [min(bbox[0], bb.XMin), min(bbox[1], bb.YMin), min(bbox[2], bb.ZMin),
                        max(bbox[3], bb.XMax), max(bbox[4], bb.YMax), max(bbox[5], bb.ZMax)]
    return solids, total_volume, bbox

def main():
    build_all()
    DOC.recompute()

    fcstd = OUT / "Terrain_Elevate_V0_59_native.FCStd"
    step = OUT / "Terrain_Elevate_V0_59.step"

    DOC.saveAs(str(fcstd))
    export_objs = [obj for obj in DOC.Objects if hasattr(obj, "Shape") and not obj.Shape.isNull()]
    Import.export(export_objs, str(step))

    solids, total_volume, bbox = analyze_doc()
    manifest = {
        "project": PARAMS["project"],
        "version": PARAMS["version"],
        "engine": "FreeCAD/OpenCascade",
        "native_document": str(fcstd.name),
        "step_file": str(step.name),
        "solid_body_count": len(solids),
        "total_volume_mm3": total_volume,
        "bounding_box_mm": bbox,
        "locked_constraints": PARAMS["locked_constraints"],
        "truth_boundary": "Generated by FreeCAD/OpenCascade. Not FEA, dynamic contact, physical test, or production release.",
        "outputs_nonempty": {
            "fcstd_bytes": fcstd.stat().st_size if fcstd.exists() else 0,
            "step_bytes": step.stat().st_size if step.exists() else 0
        },
        "solids": solids
    }
    (OUT / "Terrain_Elevate_V0_59_manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps({
        "status": "PASS",
        "fcstd": str(fcstd),
        "step": str(step),
        "solid_body_count": len(solids),
        "bbox": bbox,
        "total_volume_mm3": total_volume
    }, indent=2))

if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
