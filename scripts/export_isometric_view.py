#!/usr/bin/env python3
"""Real vector-projection SVG renders of the actual CAD geometry: shop
assembly, stair-climb pose, and pod-removed/handle-folded pose. Not a
mockup -- these are exported straight from the same solids as the STEP/STL.
"""
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.cad_model import export_isometric_views, load_parameters


def main() -> None:
    params = load_parameters(ROOT / "data" / "te_v059_parameters.json")
    export_isometric_views(params, ROOT / "cad_out")
    print("Wrote isometric SVG views to cad_out/")


if __name__ == "__main__":
    main()
