# Terrain Elevate CAD

Parametric CAD source for the Terrain Elevate P1 stair-climbing stroller prototype.

The current production build uses CadQuery 2.8 on OpenCascade/OCCT 7.9 to generate a Fusion-ready STEP assembly, an STL mesh export, a top-view DXF package drawing, and a JSON manifest.

## Prototype Constraints

- Four 280 mm ground-contact wheels only
- 720 mm wheelbase
- 620 mm track width
- 750 mm outer packaging gate
- No tracks, belts, extra helper wheels, anti-tip rollers, canopy, or decorative geometry
- Reference stair: 203.2 mm rise by 279.4 mm going

## Build Locally

```powershell
python -m pip install -r requirements-cad.txt
python scripts/build_occt_cad.py
python validate_outputs.py
```

Outputs are written to `cad_out/`:

- `Terrain_Elevate_P1_V0_59_OCCT.step`
- `Terrain_Elevate_P1_V0_59_OCCT.stl`
- `Terrain_Elevate_P1_V0_59_package.dxf`
- `Terrain_Elevate_P1_V0_59_manifest.json`

The legacy FreeCAD script remains in the repository as a reference path, but the GitHub Actions build now targets the OCCT/CadQuery generator.
