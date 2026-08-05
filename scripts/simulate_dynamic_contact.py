#!/usr/bin/env python3
"""Rigid-body dynamic-contact simulation of the exported URDF on real stairs.

Truth boundary: this is a rigid-body contact simulation of the CAD-derived
URDF, using the tire contact stiffness/damping/friction already recorded in
data/te_v059_physics_elements.json. It is NOT a validated dynamic model, not a
physical test, and not yet a ride-comfort evaluation. Read
`modelling_limitations` in the output before drawing any comfort conclusion
from it -- in particular the passive corner spring/damper is absent from the
URDF, so chassis accelerations here are stiffer than the real machine's.

Purpose of this stage: establish that the exported simulation package actually
loads and behaves physically (settles under gravity, contacts the stairs, no
interpenetration or explosion). The kinematic screens cannot tell us that, and
neither can a green CI run. Everything downstream -- a real ISO 2631-1
weighted-RMS comfort figure, and any quantitative comparison of the two
candidate corner architectures -- needs this to be trustworthy first.

Reported as an honest finding, not a hard gate: a physics result that comes
out badly is information, not a build error.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM_OUT = ROOT / "sim_out"
OUT_DIR = ROOT / "analysis_out"
OUT_PATH = OUT_DIR / "Terrain_Elevate_P1_V0_59_dynamic_contact_audit.json"

TRUTH_BOUNDARY = (
    "Rigid-body contact simulation of the CAD-derived URDF using the recorded "
    "tire contact parameters. Not a validated dynamic model, not a physical "
    "test, and not a ride-comfort evaluation -- see modelling_limitations."
)

MODELLING_LIMITATIONS = [
    "The passive per-corner spring/damper exists in the CAD as an envelope but "
    "is NOT a joint in the exported URDF, so the corner is rigid apart from "
    "tire compliance. Chassis accelerations here are therefore stiffer than the "
    "real machine's, and must not be quoted as a ride-comfort result.",
    "PyBullet convex-hulls non-convex collision meshes for dynamic bodies, so "
    "the chassis and wheel modules collide as their convex hulls rather than "
    "their true CAD outlines. Wheels are near-convex so this matters little for "
    "them; it matters more for any chassis-to-stair clearance conclusion.",
    "Tire contact stiffness/damping and friction come from "
    "te_v059_physics_elements.json, where they are ASSUMED screening values, "
    "not measured tire/stair data.",
    "No actuator control loop is applied at this stage: corner actuators are "
    "held at their spawn position. This screen answers whether the model is "
    "physically sound, not whether the vehicle can climb.",
]


def _fail(reason: str, detail: str = "") -> None:
    result = {
        "truth_boundary": TRUTH_BOUNDARY,
        "result": "NOT_RUN",
        "reason": reason,
        "detail": detail,
        "modelling_limitations": MODELLING_LIMITATIONS,
    }
    OUT_DIR.mkdir(exist_ok=True)
    OUT_PATH.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


def main() -> None:
    try:
        import pybullet as p
    except ImportError as exc:  # noqa: BLE001
        _fail(
            "pybullet is not installed on this host; install requirements-dynamics.txt",
            str(exc),
        )
        return

    urdf_path = SIM_OUT / "Terrain_Elevate_P1_V0_59_sim.urdf"
    terrain_path = SIM_OUT / "meshes" / "reference_stairs_203r_279g.stl"
    if not urdf_path.exists() or not terrain_path.exists():
        _fail(
            "simulation package missing; run scripts/export_simulation_package.py first",
            f"urdf={urdf_path.exists()} terrain={terrain_path.exists()}",
        )
        return

    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    physics = json.loads((ROOT / "data" / "te_v059_physics_elements.json").read_text())
    g = params["geometry"]
    tire = physics["contact_model"]["wheel_tire_on_dry_stair"]
    gravity = physics["environment"]["gravity_mps2"]

    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -gravity, physicsClientId=client)
    p.setAdditionalSearchPath(str(SIM_OUT), physicsClientId=client)
    # 0.5 ms matches the sourced local control loop period, so the contact
    # solver is not coarser than the loop the real vehicle runs.
    timestep_s = physics["sensor_and_control_timing"]["local_motor_suspension_command_loop_s"]
    p.setTimeStep(timestep_s, physicsClientId=client)

    # Static terrain must be a true concave trimesh; the convex-hull default
    # would fill in every stair and present a ramp instead of a staircase.
    terrain_collision = p.createCollisionShape(
        p.GEOM_MESH,
        fileName=str(terrain_path),
        meshScale=[0.001, 0.001, 0.001],
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        physicsClientId=client,
    )
    terrain_body = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=terrain_collision,
        basePosition=[0, 0, 0],
        physicsClientId=client,
    )
    p.changeDynamics(
        terrain_body, -1,
        lateralFriction=tire["static_friction_mu_ASSUMED"],
        restitution=0.0,
        physicsClientId=client,
    )

    # Spawn clear of the stairs and above the ground: the chassis origin sits a
    # slider drop plus a wheel radius above the contact patch, derived here
    # rather than hardcoded so it tracks the wheel-diameter parameter.
    wheel_radius_m = g["wheel_diameter_mm"] / 2000.0
    chassis_above_ground_m = 0.23 + wheel_radius_m
    spawn_z = chassis_above_ground_m + 0.05  # small drop so settling is visible
    spawn_x = -1.2  # behind the first riser, on flat ground

    robot = p.loadURDF(
        str(urdf_path),
        basePosition=[spawn_x, 0, spawn_z],
        useFixedBase=False,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=client,
    )

    joint_count = p.getNumJoints(robot, physicsClientId=client)
    joints = []
    wheel_link_indices = []
    for i in range(joint_count):
        info = p.getJointInfo(robot, i, physicsClientId=client)
        name = info[1].decode()
        jtype = info[2]
        joints.append({
            "name": name,
            "type": {p.JOINT_REVOLUTE: "revolute", p.JOINT_PRISMATIC: "prismatic",
                     p.JOINT_FIXED: "fixed"}.get(jtype, str(jtype)),
        })
        if name.endswith("_wheel_drive_joint"):
            wheel_link_indices.append(i)
        # Apply real tire contact properties to the wheel links only.
        if name.endswith("_wheel_drive_joint"):
            p.changeDynamics(
                robot, i,
                lateralFriction=tire["static_friction_mu_ASSUMED"],
                rollingFriction=tire["rolling_resistance_coefficient_ASSUMED"],
                spinningFriction=tire["torsional_friction_coefficient_ASSUMED"],
                contactStiffness=tire["normal_stiffness_N_per_m"],
                contactDamping=tire["normal_damping_Ns_per_m"],
                restitution=0.0,
                physicsClientId=client,
            )

    # Hold every actuated joint at its spawn position: this stage asks whether
    # the model is physically sound, not whether it can climb.
    for i in range(joint_count):
        info = p.getJointInfo(robot, i, physicsClientId=client)
        if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            p.setJointMotorControl2(
                robot, i, p.POSITION_CONTROL, targetPosition=0.0,
                force=info[10] if info[10] > 0 else 2000.0,
                physicsClientId=client,
            )

    settle_time_s = 2.0
    steps = int(settle_time_s / timestep_s)
    heights: list[float] = []
    max_speed = 0.0
    for step_i in range(steps):
        p.stepSimulation(physicsClientId=client)
        if step_i % 20 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot, physicsClientId=client)
            lin, _ = p.getBaseVelocity(robot, physicsClientId=client)
            heights.append(pos[2])
            max_speed = max(max_speed, math.sqrt(sum(v * v for v in lin)))

    final_pos, final_orn = p.getBasePositionAndOrientation(robot, physicsClientId=client)
    final_lin, _ = p.getBaseVelocity(robot, physicsClientId=client)
    euler = p.getEulerFromQuaternion(final_orn)
    contacts = p.getContactPoints(bodyA=robot, bodyB=terrain_body, physicsClientId=client)
    contact_links = sorted({c[3] for c in contacts})
    max_penetration_mm = 0.0
    if contacts:
        # contactDistance is negative when bodies interpenetrate.
        max_penetration_mm = -min(c[8] for c in contacts) * 1000.0

    final_speed = math.sqrt(sum(v * v for v in final_lin))
    settled = final_speed < 0.02
    exploded = (
        not all(math.isfinite(v) for v in final_pos)
        or abs(final_pos[2]) > 10.0
        or max_speed > 50.0
    )
    upright = abs(math.degrees(euler[1])) < 10.0 and abs(math.degrees(euler[0])) < 10.0
    resting_height_m = final_pos[2]
    expected_height_m = chassis_above_ground_m
    height_error_mm = (resting_height_m - expected_height_m) * 1000.0

    checks = {
        "model_did_not_explode": not exploded,
        "settled_to_rest": settled,
        "remained_upright": upright,
        "wheels_contacted_ground": len(contact_links) > 0,
        "resting_height_matches_geometry": abs(height_error_mm) < 60.0,
        "no_gross_interpenetration": max_penetration_mm < 20.0,
    }
    overall = all(checks.values())

    result = {
        "truth_boundary": TRUTH_BOUNDARY,
        "purpose": (
            "Establish that the exported simulation package loads and behaves "
            "physically before any comfort or architecture conclusion is drawn "
            "from it."
        ),
        "backend": {
            "engine": "PyBullet",
            "version": getattr(p, "__version__", "unknown"),
            "mode": "DIRECT (headless)",
            "timestep_s": timestep_s,
            "settle_time_s": settle_time_s,
            "gravity_mps2": gravity,
        },
        "model": {
            "urdf": urdf_path.name,
            "terrain_mesh": terrain_path.name,
            "terrain_collision": "GEOM_FORCE_CONCAVE_TRIMESH (true staircase, not a hull)",
            "joint_count": joint_count,
            "joints": joints,
            "wheel_joints_found": len(wheel_link_indices),
            "spawn_position_m": [spawn_x, 0, spawn_z],
        },
        "contact_parameters_applied_SRC": {
            "source": "data/te_v059_physics_elements.json contact_model.wheel_tire_on_dry_stair",
            "static_friction_mu": tire["static_friction_mu_ASSUMED"],
            "rolling_resistance": tire["rolling_resistance_coefficient_ASSUMED"],
            "normal_stiffness_N_per_m": tire["normal_stiffness_N_per_m"],
            "normal_damping_Ns_per_m": tire["normal_damping_Ns_per_m"],
        },
        "settling_CALC": {
            "final_position_m": list(final_pos),
            "final_roll_pitch_yaw_deg": [math.degrees(a) for a in euler],
            "final_speed_mps": final_speed,
            "max_speed_during_settle_mps": max_speed,
            "resting_chassis_height_m": resting_height_m,
            "expected_chassis_height_m": expected_height_m,
            "height_error_mm": height_error_mm,
            "contact_link_indices": contact_links,
            "contact_point_count": len(contacts),
            "max_interpenetration_mm": max_penetration_mm,
            "height_trace_m": heights[:40],
        },
        "checks": checks,
        "modelling_limitations": MODELLING_LIMITATIONS,
        "open_items": [
            "Add the passive corner spring/damper as a real URDF joint before any "
            "ride-comfort figure is computed here; without it the corner is rigid "
            "apart from tire compliance and accelerations are overstated.",
            "Drive the corner actuators from the stair-climb control state machine "
            "and command a real climb, rather than holding joints at spawn "
            "position, before claiming stair-climb validation.",
            "Once both of the above exist, compute an ISO 2631-1 frequency-weighted "
            "RMS acceleration from the real contact-simulated chassis motion. That "
            "is the figure the kinematic screen cannot produce, and it is what "
            "closes the ride-quality comfort question either way.",
        ],
        "result": "PASS" if overall else "FAIL",
    }

    p.disconnect(physicsClientId=client)
    OUT_DIR.mkdir(exist_ok=True)
    OUT_PATH.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
