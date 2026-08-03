#!/usr/bin/env python3
import json
from pathlib import Path

import ezdxf
from cadquery import importers

out = Path("cad_out")
manifest_path = out / "Terrain_Elevate_P1_V0_59_manifest.json"
step = out / "Terrain_Elevate_P1_V0_59_OCCT.step"
stl = out / "Terrain_Elevate_P1_V0_59_OCCT.stl"
pose_step = out / "Terrain_Elevate_P1_V0_59_stair_climb_pose.step"
pose_stl = out / "Terrain_Elevate_P1_V0_59_stair_climb_pose.stl"
dxf = out / "Terrain_Elevate_P1_V0_59_package.dxf"

errors = []
for p in [manifest_path, step, stl, pose_step, pose_stl, dxf]:
    if not p.exists():
        errors.append(f"Missing {p}")
    elif p.stat().st_size <= 0:
        errors.append(f"Empty {p}")

if manifest_path.exists():
    m = json.loads(manifest_path.read_text())
    if m.get("component_count", 0) < 1300:
        errors.append(f"Expected at least 1300 connected/sourced assembly components, got {m.get('component_count')}")
    if not m.get("reimported_step_bounding_box_mm"):
        errors.append("Missing reimported STEP bounding box")
    if "stair_climb_pose_step_file" not in m.get("outputs", {}):
        errors.append("Missing stair-climb pose STEP export")
    if abs(m.get("pose_exports", {}).get("stair_angle_deg", 0) - 36.03) > 0.05:
        errors.append("Stair-climb pose must use the 36.03 degree stair angle")
    if m.get("locked_constraints", {}).get("main_ground_contact_wheels") != 4:
        errors.append("Prototype must have exactly four ground-contact wheels")
    sourced = json.dumps(m.get("sourced_parts", [])).lower()
    for required_part in ("bnk2010", "hsr15", "ec-i 52", "gpx52", "ab 44", "electrak md", "deutsch dtp", "midi 498", "iglidur g", "am-m10t", "din 985", "encoder", "load-sensing"):
        if required_part not in sourced:
            errors.append(f"Required sourced part missing from manifest: {required_part}")
    ebom = m.get("ebom_summary", {})
    if not ebom.get("by_material") or not ebom.get("by_subsystem"):
        errors.append("Manifest must include material and subsystem EBOM summaries")
    forbidden = ("extra helper wheel", "belt drive", "crawler track", "anti tip roller", "anti-tip roller")
    generated_text = json.dumps(m.get("components", [])).lower()
    for phrase in forbidden:
        if phrase in generated_text:
            errors.append(f"Forbidden generated geometry phrase found in manifest: {phrase}")

if step.exists():
    try:
        imported = importers.importStep(str(step))
        if imported.val().Volume() <= 0:
            errors.append("Reimported STEP has no measurable volume")
    except Exception as exc:
        errors.append(f"Unable to reimport STEP: {exc}")

if pose_step.exists():
    try:
        imported_pose = importers.importStep(str(pose_step))
        if imported_pose.val().Volume() <= 0:
            errors.append("Reimported stair-climb pose STEP has no measurable volume")
    except Exception as exc:
        errors.append(f"Unable to reimport stair-climb pose STEP: {exc}")

if dxf.exists():
    try:
        parsed = ezdxf.readfile(dxf)
        if len(parsed.modelspace()) < 1:
            errors.append("DXF contains no modelspace entities")
    except Exception as exc:
        errors.append(f"Unable to parse DXF: {exc}")

if errors:
    print("CAD VALIDATION FAIL")
    for e in errors:
        print("-", e)
    raise SystemExit(1)

print("CAD VALIDATION PASS")
print("STEP bytes:", step.stat().st_size)
print("STL bytes:", stl.stat().st_size)
print("DXF bytes:", dxf.stat().st_size)
print("Manifest:", manifest_path)
