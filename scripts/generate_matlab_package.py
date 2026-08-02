#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    sim_dir = ROOT / "sim_out"
    sim_dir.mkdir(exist_ok=True)
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    load = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    g = params["geometry"]
    actuator = load["actuator_assumptions"]

    matlab_script = f"""%% Terrain Elevate P1 V0.59 MATLAB/Simscape import helper
% Generated from the repository's neutral CAD/simulation package.
% Run from the repository root after building sim_out/.

clear; clc;

urdfPath = fullfile(pwd, "sim_out", "Terrain_Elevate_P1_V0_59_sim.urdf");
jointGraphPath = fullfile(pwd, "sim_out", "Terrain_Elevate_P1_V0_59_joint_graph.json");

robot = importrobot(urdfPath);
robot.DataFormat = "struct";
robot.Gravity = [0 0 -9.80665];

jointGraph = jsondecode(fileread(jointGraphPath));

terrainElevateParams = struct();
terrainElevateParams.wheelDiameter_m = {g["wheel_diameter_mm"] / 1000:.6f};
terrainElevateParams.wheelRadius_m = {g["wheel_diameter_mm"] / 2000:.6f};
terrainElevateParams.wheelbase_m = {g["wheelbase_mm"] / 1000:.6f};
terrainElevateParams.track_m = {g["track_mm"] / 1000:.6f};
terrainElevateParams.stairRise_m = {g["stair_rise_reference_mm"] / 1000:.6f};
terrainElevateParams.stairGoing_m = {g["stair_going_reference_mm"] / 1000:.6f};
terrainElevateParams.stairAngle_deg = {g["stair_angle_deg_CALC"]:.6f};
terrainElevateParams.cornerStroke_m = {g["corner_stroke_selected_mm_SRC"] / 1000:.6f};
terrainElevateParams.cornerActuatorVelocity_mps = 0.022;
terrainElevateParams.cornerActuatorEffort_N = 2511.0;
terrainElevateParams.wheelDriveEffort_Nm = 60.0;
terrainElevateParams.screwLead_m = 0.004;
terrainElevateParams.screwEfficiency = {actuator["screw_efficiency_ASSUMED"]:.6f};

homeConfig = homeConfiguration(robot);
show(robot, homeConfig, "Visuals", "on", "Collisions", "on");
title("Terrain Elevate P1 V0.59 simulation-ready URDF");

disp("Loaded Terrain Elevate URDF and joint graph.");
disp(jointGraph.degrees_of_freedom);
"""

    script_path = sim_dir / "load_terrain_elevate_matlab.m"
    script_path.write_text(matlab_script)

    simscape_notes = {
        "matlab_entrypoint": script_path.name,
        "recommended_import": "MATLAB importrobot for URDF; use Simscape Multibody Import XML only after SolidWorks/MATLAB plugin translation if native multibody constraints are required.",
        "solidworks_entrypoint": "../cad_out/Terrain_Elevate_P1_V0_59_OCCT.step",
        "simulation_entrypoint": "Terrain_Elevate_P1_V0_59_sim.urdf",
        "units": "URDF in meters; CAD STEP in millimeters",
        "intended_tools": ["MATLAB Robotics System Toolbox", "Simscape Multibody via URDF or translated XML", "SolidWorks STEP import"],
        "warning": "URDF contains simplified collision/visual meshes and explicit joints for dynamics setup; detailed 793-body STEP is the richer CAD reference for SolidWorks.",
    }
    notes_path = sim_dir / "MATLAB_SOLIDWORKS_IMPORT_NOTES.json"
    notes_path.write_text(json.dumps(simscape_notes, indent=2))
    print(json.dumps({"status": "PASS", "matlab_script": str(script_path), "notes": str(notes_path)}, indent=2))


if __name__ == "__main__":
    main()
