#!/usr/bin/env python3
"""Fold-to-trunk / stroller-size envelope check.

Computes the real CAD bounding box with the folding handle assembly rotated
down about its modeled hinge axis (src/terrain_elevate/cad_model.py,
build_folded_pose_components) and compares it against representative trunk
cargo-opening and stroller folded-footprint reference figures.

Truth boundary: this is a CAD bounding-box CALC, not a physical fit test, and
it is informational -- it does not gate the overall build. Only the handle
fold is modeled; the seat pod and corner suspension have no fold mechanism in
this CAD yet, so this is a partial/conservative (upper-bound) estimate. The
trunk/stroller reference figures are representative, not measurements of a
specific target vehicle or competitor product.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.cad_model import folded_pose_bounding_box_mm, load_parameters

TRUNK_CARGO_OPENING_MM_ASSUMED = {
    "width": 1000.0,
    "height": 700.0,
    "depth": 900.0,
    "reason": "Representative mid-size sedan/hatchback trunk cargo opening; typical "
    "published figures fall in the 900-1100 mm width x 650-800 mm height range. Not a "
    "specific vehicle's measured opening.",
}
STROLLER_FOLDED_FOOTPRINT_MM_ASSUMED = {
    "length": 850.0,
    "width": 600.0,
    "height": 350.0,
    "reason": "Representative folded footprint of a full-size (non-umbrella) consumer "
    "stroller; typical published folded dimensions fall in the 750-950 mm length x "
    "500-650 mm width x 280-420 mm height range. Not a specific product's measured spec.",
}


def main() -> None:
    params = load_parameters(ROOT / "data" / "te_v059_parameters.json")
    bbox = folded_pose_bounding_box_mm(params)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox
    length_mm = xmax - xmin
    width_mm = ymax - ymin
    height_mm = zmax - zmin

    trunk = TRUNK_CARGO_OPENING_MM_ASSUMED
    stroller = STROLLER_FOLDED_FOOTPRINT_MM_ASSUMED

    fits_trunk = length_mm <= trunk["depth"] and width_mm <= trunk["width"] and height_mm <= trunk["height"]

    result = {
        "truth_boundary": (
            "CAD bounding-box CALC with only the folding handle re-posed about its "
            "modeled hinge axis. Not a physical fit test, and informational only -- "
            "does not gate the overall build. Corner suspension stroke and the seat "
            "pod are not folded in this CAD yet, so this is a partial, conservative "
            "(upper-bound) estimate of the true folded envelope."
        ),
        "folded_bounding_box_mm": bbox,
        "folded_length_mm": length_mm,
        "folded_width_mm": width_mm,
        "folded_height_mm": height_mm,
        "trunk_cargo_opening_reference_mm_ASSUMED": trunk,
        "stroller_folded_footprint_reference_mm_ASSUMED": stroller,
        "comparison": {
            "fits_stated_trunk_opening": fits_trunk,
            "length_vs_stroller_reference_ratio": length_mm / stroller["length"],
            "width_vs_stroller_reference_ratio": width_mm / stroller["width"],
            "height_vs_stroller_reference_ratio": height_mm / stroller["height"],
        },
        "open_items": [
            "Corner suspension is not commanded to a retracted/parked stroke in this "
            "pose, and the seat pod has no modeled fold mechanism -- both would need "
            "real fold hardware and CAD poses before this is a production fold spec. "
            "Closes when those subsystems get sourced fold hardware.",
            "Trunk and stroller reference dimensions above are representative "
            "figures, not measurements of a specific target vehicle or competitor "
            "product. Closes when a specific target trunk/vehicle is sourced.",
        ],
        "result": "PASS" if fits_trunk else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "Terrain_Elevate_P1_V0_59_fold_envelope_audit.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
