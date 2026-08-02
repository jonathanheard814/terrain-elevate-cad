# Terrain Elevate CAD

Parametric CAD source for the Terrain Elevate P1 stair-climbing stroller prototype.

The current production build uses CadQuery 2.8 on OpenCascade/OCCT 7.9 to generate a Fusion-ready named STEP assembly, an STL mesh export, a top-view DXF package drawing, and a JSON manifest.

## Prototype Constraints

- Four 280 mm ground-contact wheels only
- 720 mm wheelbase
- 620 mm track width
- 750 mm outer packaging gate
- No tracks, belts, extra helper wheels, anti-tip rollers, canopy, or decorative geometry
- Reference stair: 203.2 mm rise by 279.4 mm going
- Mobility chassis follows the staircase slope while the occupant pod levels relative to the chassis
- Four independent corner modules use 300 mm selected suspension stroke

## Build Locally

```powershell
python -m pip install -r requirements-cad.txt
python scripts/analyze_vehicle_requirements.py
python scripts/build_occt_cad.py
python validate_outputs.py
python scripts/audit_assembly_connectivity.py
```

Outputs are written to `cad_out/`:

- `Terrain_Elevate_P1_V0_59_OCCT.step`
- `Terrain_Elevate_P1_V0_59_OCCT.stl`
- `Terrain_Elevate_P1_V0_59_package.dxf`
- `Terrain_Elevate_P1_V0_59_manifest.json`

The physics and sizing screen is written to `analysis_out/Terrain_Elevate_P1_V0_59_requirements_screen.json`.
The body-by-body connectivity audit is written to `analysis_out/Terrain_Elevate_P1_V0_59_connectivity_audit.json`.

The legacy FreeCAD script remains in the repository as a reference path, but the GitHub Actions build now targets the OCCT/CadQuery generator.

## Sourced Hardware Represented

- THK `BNK1404-3RRG2+430LC7Y` 300 mm stroke, 4 mm lead ball screw
- THK FK12/FF12-style screw support units
- THK HSR15C/HSR15 paired vertical linear guide hardware
- maxon EC-i 40 screened actuator motor family
- maxon AB 60 S normally-engaged holding brake
- Thomson Electrak MD pod pitch-leveling actuator family
- Dunkermotoren BG75/PLG75-class wheel drive family
- TE DEUTSCH DTP/DT sealed connector families and Littelfuse MIDI 498 fuse blocks
- igus iglidur G / igubal pivot bushing and spherical bearing families
- Aurora AM-M10T M10 PTFE-lined rod ends
- ISO 4762 socket-head cap screws and DIN 985 / ISO 10511 lock nuts
- S32K3, STM32G474, and DRV8353 screened control and drive families
- Explicit pivot pins, screw supports, guide rail fasteners, washers, tower lugs, clevises, and chassis/pod attachment hardware
