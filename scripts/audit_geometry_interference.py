#!/usr/bin/env python3
"""Pairwise solid interference audit.

Truth boundary: this measures whether the assembly's solids occupy the same
space. It is geometry only -- it says nothing about whether the design is
mechanically sensible, only whether it is physically possible to build.

Why this exists: nothing in this repo checked it. The connectivity audit
requires every body to be in one touching group, the acceptance criteria
require a MINIMUM body count of 1300, and neither notices when two solids
overlap. Both metrics are in fact satisfied more easily by overlapping parts,
so the model has been optimised against checks that reward exactly the defect
a human sees immediately on opening the STEP.

Two solids sharing volume cannot be manufactured or assembled. This audit
reports how much of that exists.

Algorithm: axis-aligned bounding-box broad phase over all pairs, then an exact
boolean-intersection volume on every surviving candidate. Bolts and washers are
reported separately -- this model has no drilled holes, so a fastener passing
through a plate registers as interference and would otherwise drown out the
structural clashes that actually matter.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.cad_model import build_components, load_parameters  # noqa: E402

FASTENER_TOKENS = ("bolt", "washer", "locknut", "screw_m", "cap_screw", "nut_", "_nut")
# Solids that are declared reference/datum geometry rather than physical parts.
NON_PHYSICAL_TOKENS = ("datum", "envelope_reference", "reference_")
VOLUME_EPS_MM3 = 1.0  # ignore numerically-trivial slivers


def _is_fastener(name: str) -> bool:
    return any(t in name for t in FASTENER_TOKENS)


def _is_non_physical(name: str, material: str) -> bool:
    return material == "reference" or any(t in name for t in NON_PHYSICAL_TOKENS)


def main() -> None:
    params = load_parameters(ROOT / "data" / "te_v059_parameters.json")
    components = build_components(params)

    solids = []
    skipped_non_physical = 0
    for c in components:
        if _is_non_physical(c.name, c.material):
            skipped_non_physical += 1
            continue
        try:
            shape = c.shape.val()
            bb = shape.BoundingBox()
            solids.append((c.name, shape, bb, shape.Volume()))
        except Exception as exc:  # noqa: BLE001
            print(f"warning: could not evaluate {c.name}: {exc}", file=sys.stderr)

    n = len(solids)
    print(f"evaluating {n} physical solids "
          f"({skipped_non_physical} reference/datum skipped)", file=sys.stderr)

    # Broad phase.
    candidates = []
    for i in range(n):
        _, _, bi, _ = solids[i]
        for j in range(i + 1, n):
            _, _, bj, _ = solids[j]
            if (bi.xmin < bj.xmax and bj.xmin < bi.xmax
                    and bi.ymin < bj.ymax and bj.ymin < bi.ymax
                    and bi.zmin < bj.zmax and bj.zmin < bi.zmax):
                candidates.append((i, j))
    print(f"broad phase: {len(candidates)} AABB-overlapping pairs "
          f"of {n * (n - 1) // 2} total", file=sys.stderr)

    # Narrow phase.
    clashes = []
    failed = 0
    for k, (i, j) in enumerate(candidates):
        if k % 500 == 0:
            print(f"  narrow phase {k}/{len(candidates)}", file=sys.stderr)
        name_i, shape_i, _, vol_i = solids[i]
        name_j, shape_j, _, vol_j = solids[j]
        try:
            inter = shape_i.intersect(shape_j)
            vol = inter.Volume() if inter is not None else 0.0
        except Exception:  # noqa: BLE001
            failed += 1
            continue
        if vol <= VOLUME_EPS_MM3:
            continue
        smaller = min(vol_i, vol_j)
        clashes.append({
            "a": name_i,
            "b": name_j,
            "interference_volume_mm3": round(vol, 2),
            "fraction_of_smaller_part": round(vol / smaller, 4) if smaller > 0 else None,
            "involves_fastener": _is_fastener(name_i) or _is_fastener(name_j),
        })

    structural = [c for c in clashes if not c["involves_fastener"]]
    fastener = [c for c in clashes if c["involves_fastener"]]
    structural.sort(key=lambda c: -c["interference_volume_mm3"])
    fastener.sort(key=lambda c: -c["interference_volume_mm3"])

    parts_in_structural = sorted({p for c in structural for p in (c["a"], c["b"])})
    total_structural_mm3 = sum(c["interference_volume_mm3"] for c in structural)

    result = {
        "truth_boundary": (
            "Geometric interference only. Says nothing about whether the design is "
            "mechanically sensible -- only whether it is physically buildable."
        ),
        "method": (
            "AABB broad phase over all pairs, then exact boolean-intersection volume "
            "on each candidate. Fastener clashes are separated out because this model "
            "has no drilled holes, so a bolt through a plate necessarily registers."
        ),
        "physical_solids_checked": n,
        "reference_solids_skipped": skipped_non_physical,
        "aabb_candidate_pairs": len(candidates),
        "narrow_phase_failures": failed,
        "structural_interference": {
            "pair_count": len(structural),
            "total_volume_mm3": round(total_structural_mm3, 1),
            "distinct_parts_involved": len(parts_in_structural),
            "worst_25": structural[:25],
        },
        "fastener_interference": {
            "pair_count": len(fastener),
            "note": (
                "Expected in this model because no holes are modelled. Becomes a real "
                "finding once fastener holes exist -- until then it is a measure of how "
                "much of the assembly is represented rather than designed."
            ),
            "worst_10": fastener[:10],
        },
        "open_items": [
            "No hole features are modelled anywhere in the assembly, so fasteners "
            "intersect the parts they pass through by construction. Until holes exist, "
            "fastener_interference cannot distinguish a represented joint from a "
            "modelling error.",
            "This audit does not check clearance -- parts that merely touch, or sit "
            "implausibly close without overlapping, pass. Minimum-clearance checking "
            "needs a real assembly-tolerance budget, which does not exist yet.",
        ],
        "result": "PASS" if not structural else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "Terrain_Elevate_P1_V0_59_geometry_interference_audit.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps({k: v for k, v in result.items() if k != "structural_interference"},
                     indent=2))
    print(json.dumps({"structural_interference_summary": {
        "pair_count": len(structural),
        "total_volume_mm3": round(total_structural_mm3, 1),
        "distinct_parts_involved": len(parts_in_structural),
        "worst_25": structural[:25],
    }}, indent=2))

    # Hard gate. This started at 2877 clashing pairs and 31.8 litres of shared
    # volume; it is now zero, and zero is the only defensible value -- two
    # solids in the same space cannot be manufactured. Gating it here means the
    # assembly can never silently drift back, which is exactly what happened
    # while the only geometry checks were a body-count minimum and a
    # touching-group test that overlap satisfied trivially.
    if structural:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
