#!/usr/bin/env python3
import json
from pathlib import Path

import ezdxf
from cadquery import importers

out = Path("cad_out")
manifest_path = out / "Terrain_Elevate_P1_V0_59_manifest.json"
step = out / "Terrain_Elevate_P1_V0_59_OCCT.step"
stl = out / "Terrain_Elevate_P1_V0_59_OCCT.stl"
dxf = out / "Terrain_Elevate_P1_V0_59_package.dxf"

errors = []
for p in [manifest_path, step, stl, dxf]:
    if not p.exists():
        errors.append(f"Missing {p}")
    elif p.stat().st_size <= 0:
        errors.append(f"Empty {p}")

if manifest_path.exists():
    m = json.loads(manifest_path.read_text())
    if m.get("component_count", 0) < 790:
        errors.append(f"Expected at least 790 connected/sourced assembly components, got {m.get('component_count')}")
    if not m.get("reimported_step_bounding_box_mm"):
        errors.append("Missing reimported STEP bounding box")
    if m.get("locked_constraints", {}).get("main_ground_contact_wheels") != 4:
        errors.append("Prototype must have exactly four ground-contact wheels")
    sourced = json.dumps(m.get("sourced_parts", [])).lower()
    for required_part in ("bnk1404", "hsr15", "ab 60", "electrak md", "deutsch dtp", "midi 498", "iglidur g", "am-m10t", "din 985"):
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
