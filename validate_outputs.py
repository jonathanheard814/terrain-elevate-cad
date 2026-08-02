#!/usr/bin/env python3
import json
from pathlib import Path

out = Path("cad_out")
manifest_path = out / "Terrain_Elevate_V0_59_manifest.json"
fcstd = out / "Terrain_Elevate_V0_59_native.FCStd"
step = out / "Terrain_Elevate_V0_59.step"

errors = []
for p in [manifest_path, fcstd, step]:
    if not p.exists():
        errors.append(f"Missing {p}")
    elif p.stat().st_size <= 0:
        errors.append(f"Empty {p}")

if manifest_path.exists():
    m = json.loads(manifest_path.read_text())
    if m.get("solid_body_count", 0) < 80:
        errors.append(f"Expected at least 80 solid bodies, got {m.get('solid_body_count')}")
    if not m.get("bounding_box_mm"):
        errors.append("Missing bounding box")
    if "extra helper wheels" in json.dumps(m).lower():
        errors.append("Forbidden extra helper wheels detected in manifest text")

if errors:
    print("CAD VALIDATION FAIL")
    for e in errors:
        print("-", e)
    raise SystemExit(1)

print("CAD VALIDATION PASS")
print("FCStd bytes:", fcstd.stat().st_size)
print("STEP bytes:", step.stat().st_size)
print("Manifest:", manifest_path)
