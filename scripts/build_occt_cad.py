#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.cad_model import export_model, load_parameters, manifest_as_jsonable


def main() -> None:
    params = load_parameters(ROOT / "data" / "te_v059_parameters.json")
    manifest = export_model(params, ROOT / "cad_out")
    print(json.dumps(manifest_as_jsonable({
        "status": "PASS",
        "engine": manifest["engine"],
        "component_count": manifest["component_count"],
        "outputs": manifest["outputs"],
        "output_bytes": manifest["output_bytes"],
        "reimported_step_bounding_box_mm": manifest["reimported_step_bounding_box_mm"],
    }), indent=2))


if __name__ == "__main__":
    main()
