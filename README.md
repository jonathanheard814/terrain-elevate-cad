# Terrain Elevate CAD

Parametric CAD source for the Terrain Elevate P1 stair-climbing stroller prototype.

The current production build uses CadQuery 2.8 on OpenCascade/OCCT 7.9 to generate a portable named STEP assembly, an STL mesh export, a top-view DXF package drawing, and a JSON manifest.

## Read This First: What Is And Is Not Established

This repository is intended to become a real, buildable product, not a concept render. That commitment cuts both ways: **several screens in this build deliberately report FAIL, and those failures are the honest output of the design, not bugs to be suppressed.** A green CI run means the build ran without crashing — it does not mean every engineering gate passes.

Two classes of check exist, and the distinction is enforced in `scripts/audit_design_acceptance.py`:

- **Hard gates** — structural and safety properties (assembly connectivity, zero floating parts, wheel-riser clearance, the locked four-wheel/no-tracks/no-belts architecture, the ≥0.2 m/s no-break-stride floor). These block the build.
- **Honest findings** — sizing screens allowed to FAIL and be reported (wheel-only traction, corner-actuator speed, ride quality, and the power cross-check). Forcing these to pass by choosing convenient numbers is explicitly out of bounds.

### The open problem: ride quality is over-constrained

The corner actuators are meant to cancel the staircase into a ramp-like path. They do not currently achieve it, and this is **not** a component-selection gap that a better motor closes. Three constraints collide — forward speed ≥0.2 m/s (locked master requirement), chassis ramp error ≤25 mm, and the sourced 90 A / 314.5 Wh battery pack. Each could in principle be relieved by one of four routes; all four have now been measured and closed:

| Route | Result |
|---|---|
| Faster corner actuator | Needs 1455 W per corner. At a physically impossible 100% efficiency, ignoring every other load, four corners draw 90.95 A against a 90 A pack — a lower bound, so no efficiency argument recovers it. No compact BLDC family reaches the torque×speed product either (Dunkermotoren BG75, maxon EC 90 flat both checked against real datasheets). |
| Slower climb | Still fails (31.3 mm vs 25 mm) at 0.031 m/s — already 6.5× below the hard no-break-stride floor. Blocked twice. |
| Larger wheel | Ramp error falls 86.1 → 25.5 mm from 430 → 900 mm, and required power falls 8×, but 900 mm exceeds the 750 mm packaging gate. Helps genuinely; cannot close it within a stroller. |
| Relax the 25 mm budget | Checked against ISO 2631-1 rather than adjusted. 25 mm sits essentially on the "not uncomfortable" ceiling; the achieved 86.1 mm falls in the "uncomfortable" band. Not defensible. |

The measured sweeps behind this table are in `data/te_v059_ride_quality_trade_study.json`. Closing ride quality therefore requires an **architectural** change — the two identified but not-yet-analysed candidates are carrying the 1324 N static corner load on a spring so the actuator supplies only the dynamic delta, or moving vertical isolation to the ~35 kg occupant pod. Neither has been chosen.

### Traceability conventions

Every data file carries a `truth_boundary` string stating what its numbers are and are not. Field-name suffixes are load-bearing: `_SRC` (from a real datasheet or standard), `_CALC` (derived here), `_ASSUMED` (engineering placeholder), and `OPEN` (explicitly unresolved, with the condition that would close it). Sourced parts additionally record requirement, reason selected, key specs, and CAD representation. Where a source could not be verified, that is stated rather than papered over — see the source-confidence note on the ISO 2631-1 bands in `data/te_v059_load_cases.json`.

## Prototype Constraints

- Four 430 mm ground-contact wheels only
- 720 mm wheelbase
- 620 mm track width
- 750 mm outer packaging gate
- No tracks, belts, extra helper wheels, anti-tip rollers, canopy, or decorative geometry
- Reference stair: 203.2 mm rise by 279.4 mm going
- Mobility chassis follows the staircase slope while the occupant pod levels relative to the chassis
- Four independent corner modules use 300 mm selected suspension stroke

## Build Locally

This is the same order the CI workflow runs, and the order matters: `audit_power_budget.py` cross-checks against `simulate_smooth_stair_climb.py`'s output and will report `NOT_RUN` for that check if the climb screen has not been run first.

```powershell
python -m pip install -r requirements-cad.txt
python -m pip install -r requirements-sim.txt
python scripts/analyze_vehicle_requirements.py
python scripts/simulate_smooth_stair_climb.py
python scripts/build_occt_cad.py
python scripts/audit_fold_envelope.py
python scripts/audit_power_budget.py
python scripts/export_isometric_view.py
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
The smooth stair-climb / ride-quality screen is written to `analysis_out/Terrain_Elevate_P1_V0_59_smooth_climb_audit.json`; this is the screen that currently reports the ride-quality FAIL described above, along with the searched actuator speed that would be needed to close it.
The electrical power budget and its ride-quality cross-check are written to `analysis_out/Terrain_Elevate_P1_V0_59_power_budget_audit.json`. Note its two verdicts: `as_sourced_result` answers whether the pack can run the parts currently on the BOM, while the top-level `result` answers whether it can run the machine the design actually requires. These currently differ, and the difference is the point.
The fold-envelope audit is written to `analysis_out/Terrain_Elevate_P1_V0_59_fold_envelope_audit.json`.
The measured ride-quality trade study is `data/te_v059_ride_quality_trade_study.json`; it records the wheel-diameter and forward-speed parameter sweeps behind the table above, so the over-constraint conclusion rests on recorded measurements rather than intuition.
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

This remains the *intent*, not a validated result. The geometry and clearance parts of it hold — at 430 mm the wheel rolls over the 203.2 mm nosing unaided, and commanded extension never drops below the clearance floor. The smoothing part does not: the chassis currently tracks the ideal ramp line to 86.1 mm against a 25 mm budget. See the over-constraint table above before citing rampification as demonstrated.

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
- maxon EC-i 52 part 633919 screened actuator motor family (represented in CAD and screened for force, but known insufficient for ride quality — see the over-constraint table above)
- maxon GPX52 UP 3.9:1 one-stage planetary gearhead
- maxon AB 44 24 VDC 2.5 Nm normally-engaged holding brake
- Coaxial motor-to-screw couplings, shaft collars, motor mount plates, and protective shrouds
- Thomson Electrak MD pod pitch-leveling actuator family
- Dunkermotoren BG75/PLG75-class wheel drive family
- TE DEUTSCH DTP/DT sealed connector families and Littelfuse MIDI 498 fuse blocks
- Sealed wheel encoders, suspension linear position scales, and load-sensing pivot pins for combined stair-phase control
- Molded tread-lug features on the four main 430 x 75 mm tires
- Individual connector contacts, wedgelocks, cable tie mounts, sensor screws, wheel flange bolts, washers, and locknuts
- igus iglidur G / igubal pivot bushing and spherical bearing families
- Aurora AM-M10T M10 PTFE-lined rod ends
- ISO 4762 socket-head cap screws and DIN 985 / ISO 10511 lock nuts
- S32K3, STM32G474, and DRV8353 screened control and drive families
- Explicit pivot pins, screw supports, guide rail fasteners, washers, tower lugs, clevises, and chassis/pod attachment hardware
