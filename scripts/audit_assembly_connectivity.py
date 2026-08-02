#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from terrain_elevate.cad_model import build_components, load_parameters



def _bbox(component):
    bb = component.shape.val().BoundingBox()
    return (bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax)


def _touches(a, b, tolerance: float) -> bool:
    return not (
        a[3] + tolerance < b[0]
        or b[3] + tolerance < a[0]
        or a[4] + tolerance < b[1]
        or b[4] + tolerance < a[1]
        or a[5] + tolerance < b[2]
        or b[5] + tolerance < a[2]
    )


def main() -> None:
    params = load_parameters(ROOT / "data" / "te_v059_parameters.json")
    components = [c for c in build_components(params) if c.material != "reference"]
    tolerance_mm = 8.0
    boxes = [_bbox(c) for c in components]
    neighbors: list[set[int]] = [set() for _ in components]

    for i in range(len(components)):
        for j in range(i + 1, len(components)):
            if _touches(boxes[i], boxes[j], tolerance_mm):
                neighbors[i].add(j)
                neighbors[j].add(i)

    seen: set[int] = set()
    groups: list[list[int]] = []
    for start in range(len(components)):
        if start in seen:
            continue
        stack = [start]
        group: list[int] = []
        seen.add(start)
        while stack:
            idx = stack.pop()
            group.append(idx)
            for neighbor in neighbors[idx]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        groups.append(group)

    groups.sort(key=len, reverse=True)
    largest = set(groups[0]) if groups else set()
    isolated = [i for i, links in enumerate(neighbors) if not links]
    outside_largest = [i for i in range(len(components)) if i not in largest]

    result = {
        "audit_method": "Axis-aligned bounding-box contact graph with 8 mm assembly tolerance; flags parts not geometrically connected to the main mechanism.",
        "component_count": len(components),
        "largest_connected_group_count": len(largest),
        "largest_connected_group_fraction": len(largest) / len(components) if components else 0,
        "connected_group_sizes": [len(g) for g in groups[:12]],
        "isolated_component_count": len(isolated),
        "isolated_components": [components[i].name for i in isolated[:80]],
        "outside_largest_group_count": len(outside_largest),
        "outside_largest_group_components": [components[i].name for i in outside_largest[:120]],
        "result": "PASS" if len(isolated) == 0 and len(outside_largest) == 0 else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "Terrain_Elevate_P1_V0_59_connectivity_audit.json"
    out_path.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if result["result"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
