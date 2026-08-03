# Terrain Elevate CAD

Parametric CAD source for the Terrain Elevate P1 stair-climbing stroller prototype.

The current production build uses CadQuery 2.8 on OpenCascade/OCCT 7.9 to generate a portable named STEP assembly, an STL mesh export, a top-view DXF package drawing, and a JSON manifest.

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
python scripts/export_simulation_package.py
python scripts/generate_matlab_package.py
python validate_outputs.py
python scripts/audit_assembly_connectivity.py
python scripts/audit_physics_basis.py
python scripts/validate_simulation_package.py
python scripts/audit_design_acceptance.py
```

Outputs are written to `cad_out/`:

- `Terrain_Elevate_P1_V0_59_OCCT.step`
- `Terrain_Elevate_P1_V0_59_OCCT.stl`
- `Terrain_Elevate_P1_V0_59_stair_climb_pose.step`
- `Terrain_Elevate_P1_V0_59_stair_climb_pose.stl`
- `Terrain_Elevate_P1_V0_59_package.dxf`
- `Terrain_Elevate_P1_V0_59_manifest.json`

The physics and sizing screen is written to `analysis_out/Terrain_Elevate_P1_V0_59_requirements_screen.json`.
The 1001-sample stair-phase actuator sweep is written to `analysis_out/Terrain_Elevate_P1_V0_59_stair_phase_actuator_sweep.csv`.
The body-by-body connectivity audit is written to `analysis_out/Terrain_Elevate_P1_V0_59_connectivity_audit.json`.
The simulation physics-basis audit is written to `analysis_out/Terrain_Elevate_P1_V0_59_physics_basis_audit.json`.
The design acceptance audit is written to `analysis_out/Terrain_Elevate_P1_V0_59_design_acceptance_audit.json`.
The sourced part decision register is `data/te_v059_sourced_part_register.json`; each item states the requirement, reason, key specs, source status, and CAD representation.
The physical simulation parameter pack is `data/te_v059_physics_elements.json`; it contains tire/stair contact assumptions, actuator motor/gear/screw dynamics, sensor rates, local control timing, and the installed simulation-library status.

Simulation-ready outputs are written to `sim_out/`:

- `Terrain_Elevate_P1_V0_59_sim.urdf`
- `Terrain_Elevate_P1_V0_59_joint_graph.json`
- `load_terrain_elevate_matlab.m`
- `MATLAB_SOLIDWORKS_IMPORT_NOTES.json`
- `meshes/*.stl`

The simulation package exposes four independent prismatic corner actuators, four wheel-drive joints, one pod pitch-leveling joint, collision meshes, visual meshes, inertial properties, and a reference stair terrain mesh.

The intended stair-climbing claim is combined-system rampification: wheel drive advances the stroller, four independent corner actuators cancel local stair phase discontinuities into a smooth ramp-like chassis path, and the pod pitch joint keeps the child level.

The current simulation basis uses SciPy/CasADi for dynamics/control sizing and Trimesh for CAD-derived mesh sanity checks. PyBullet is treated as optional because it did not install on the current Windows host without Microsoft C++ Build Tools; the repo does not claim Bullet validation until that backend is actually available.

## Tool Targets

- SolidWorks: import `cad_out/Terrain_Elevate_P1_V0_59_OCCT.step` for the detailed 1300+ body CAD assembly, or the stair-climb pose STEP for architecture inspection.
- MATLAB: run `sim_out/load_terrain_elevate_matlab.m` from the repository root after generating `sim_out/`.
- Simscape/Robotics: use `sim_out/Terrain_Elevate_P1_V0_59_sim.urdf` plus the STL meshes and joint graph as the dynamics starting point.

The legacy FreeCAD script remains in the repository as a reference path, but the GitHub Actions build now targets the OCCT/CadQuery generator.

## Sourced Hardware Represented

- THK `BNK2010-2.5RRG2+499LC7Y` 300 mm stroke, 10 mm lead ball screw
- THK FK12/FF12-style screw support units
- THK HSR15C/HSR15 paired vertical linear guide hardware
- maxon EC-i 52 part 633919 screened actuator motor family
- maxon GPX52 UP 3.9:1 one-stage planetary gearhead
- maxon AB 44 24 VDC 2.5 Nm normally-engaged holding brake
- Coaxial motor-to-screw couplings, shaft collars, motor mount plates, and protective shrouds
- Thomson Electrak MD pod pitch-leveling actuator family
- Dunkermotoren BG75/PLG75-class wheel drive family
- TE DEUTSCH DTP/DT sealed connector families and Littelfuse MIDI 498 fuse blocks
- Sealed wheel encoders, suspension linear position scales, and load-sensing pivot pins for combined stair-phase control
- Molded tread-lug features on the four main 280 x 75 mm tires
- Individual connector contacts, wedgelocks, cable tie mounts, sensor screws, wheel flange bolts, washers, and locknuts
- igus iglidur G / igubal pivot bushing and spherical bearing families
- Aurora AM-M10T M10 PTFE-lined rod ends
- ISO 4762 socket-head cap screws and DIN 985 / ISO 10511 lock nuts
- S32K3, STM32G474, and DRV8353 screened control and drive families
- Explicit pivot pins, screw supports, guide rail fasteners, washers, tower lugs, clevises, and chassis/pod attachment hardware
