#!/usr/bin/env python3
"""Electrical power budget: real load sum vs. sourced battery pack.

Sums actual electrical draw from every already-sourced powered subsystem
(corner actuators, wheel drives, pod pitch actuators, control/sensor
electronics) under two scenarios -- continuous climbing and worst-case
simultaneous peak -- and checks both against the sourced battery pack's
continuous discharge rating and against a target continuous-climb runtime.

Truth boundary: engineering sizing screen. Not a certified electrical design,
not a UL/IEC compliance review, not a thermal analysis.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    params = json.loads((ROOT / "data" / "te_v059_parameters.json").read_text())
    load_cases = json.loads((ROOT / "data" / "te_v059_load_cases.json").read_text())
    physics = json.loads((ROOT / "data" / "te_v059_physics_elements.json").read_text())
    electrical = json.loads((ROOT / "data" / "te_v059_electrical_system.json").read_text())

    g = params["geometry"]
    loads = electrical["load_assumptions"]
    battery = electrical["battery_pack"]
    system_voltage = electrical["system"]["nominal_voltage_V"]

    # Corner actuators.
    corner_current_A = loads["corner_actuator_current_A_SRC"]
    corner_peak_count = loads["corner_actuators_simultaneous_peak_ASSUMED"]
    corner_duty = loads["corner_actuator_average_duty_fraction_ASSUMED"]
    corner_peak_A = corner_current_A * corner_peak_count
    corner_continuous_A = corner_current_A * 4 * corner_duty

    # Wheel drives: real mechanical power from the already-screened climb
    # torque and forward speed, not the motor's "up to" peak rating.
    wheel_radius_m = g["wheel_diameter_mm"] / 2000.0
    forward_speed_mps = load_cases["combined_climb_assumptions"]["stair_climb_forward_speed_mps_ASSUMED"]
    wheel_angular_speed_rad_s = forward_speed_mps / wheel_radius_m
    per_wheel_torque_Nm = physics["actuator_models"]["wheel_drive"]["screened_per_wheel_torque_Nm"]
    per_wheel_mech_W = per_wheel_torque_Nm * wheel_angular_speed_rad_s
    wheel_efficiency = loads["wheel_drive_mechanical_efficiency_ASSUMED"]
    per_wheel_electrical_W = per_wheel_mech_W / wheel_efficiency
    wheel_continuous_A = (per_wheel_electrical_W * 4) / system_voltage
    # Peak: same four wheels drawing continuously is already the sustained
    # case; there is no separate "peak" wheel scenario beyond this in the
    # combined ramp-like climb (wheel-only stair lifting is not the
    # mechanism -- see requirements_screen.json wheel_propulsion_screen).
    wheel_peak_A = wheel_continuous_A

    # Pod pitch actuators.
    pod_power_W = loads["pod_pitch_actuator_power_W_ASSUMED"]
    pod_active_fraction = loads["pod_pitch_active_fraction_ASSUMED"]
    pod_peak_A = (pod_power_W * 2) / system_voltage
    pod_continuous_A = pod_peak_A * pod_active_fraction

    # Control/sensor electronics baseline.
    electronics_A = loads["electronics_baseline_continuous_W_ASSUMED"] / system_voltage

    continuous_A = corner_continuous_A + wheel_continuous_A + pod_continuous_A + electronics_A
    peak_A = corner_peak_A + wheel_peak_A + pod_peak_A + electronics_A

    continuous_W = continuous_A * system_voltage
    peak_W = peak_A * system_voltage

    pack_continuous_limit_A = battery["pack_max_continuous_discharge_A_CALC"]
    continuous_current_margin_A = pack_continuous_limit_A - peak_A
    battery_current_result = "PASS" if peak_A <= pack_continuous_limit_A else "FAIL"

    usable_energy_Wh = battery["usable_energy_Wh_CALC"]
    runtime_at_continuous_climb_minutes = (usable_energy_Wh / continuous_W) * 60.0 if continuous_W > 0 else math.inf
    target_runtime_minutes = loads["target_continuous_climb_runtime_minutes_ASSUMED"]
    runtime_result = "PASS" if runtime_at_continuous_climb_minutes >= target_runtime_minutes else "FAIL"

    overall_pass = battery_current_result == "PASS" and runtime_result == "PASS"

    result = {
        "truth_boundary": electrical["truth_boundary"],
        "method": (
            "Sums real electrical draw from every already-sourced powered subsystem "
            "under two scenarios: sustained continuous climbing (wheel drives at their "
            "screened climb torque/speed, corner actuators at an averaged duty cycle, "
            "pod actuators at an averaged active fraction) and worst-case simultaneous "
            "peak (multiple corner/pod actuators assumed active at once), then checks "
            "both against the sourced 13S2P battery pack's continuous discharge rating "
            "and a target continuous-climb runtime."
        ),
        "system_nominal_voltage_V": system_voltage,
        "loads": {
            "corner_actuators": {
                "continuous_A": corner_continuous_A,
                "peak_A": corner_peak_A,
                "basis": "maxon EC-i 52 nominal current (SRC) x simultaneity/duty assumptions",
            },
            "wheel_drives": {
                "continuous_A": wheel_continuous_A,
                "peak_A": wheel_peak_A,
                "per_wheel_mechanical_W_CALC": per_wheel_mech_W,
                "basis": "screened climb torque (requirements_screen.json) x forward speed, not motor peak rating",
            },
            "pod_pitch_actuators": {
                "continuous_A": pod_continuous_A,
                "peak_A": pod_peak_A,
                "basis": "ASSUMED Electrak MD-class power x active-fraction duty",
            },
            "electronics_baseline": {
                "continuous_A": electronics_A,
                "peak_A": electronics_A,
                "basis": "ASSUMED summed ECU/sensor/connector baseline",
            },
        },
        "totals": {
            "continuous_A": continuous_A,
            "continuous_W": continuous_W,
            "peak_A": peak_A,
            "peak_W": peak_W,
        },
        "battery_check": {
            "pack_configuration": battery["configuration_CALC"],
            "pack_nominal_voltage_V": battery["pack_nominal_voltage_V_CALC"],
            "pack_max_continuous_discharge_A": pack_continuous_limit_A,
            "screened_peak_draw_A": peak_A,
            "continuous_current_margin_A": continuous_current_margin_A,
            "result": battery_current_result,
        },
        "runtime_check": {
            "usable_energy_Wh": usable_energy_Wh,
            "continuous_climb_draw_W": continuous_W,
            "runtime_at_continuous_climb_minutes": runtime_at_continuous_climb_minutes,
            "target_runtime_minutes": target_runtime_minutes,
            "result": runtime_result,
        },
        "open_items": [
            "pod_pitch_actuator_power_W_ASSUMED and electronics_baseline_continuous_W_ASSUMED "
            "are engineering placeholders, not datasheet values; closes when the Thomson "
            "Electrak MD is configured to order and a per-PCB current budget exists for "
            "the control electronics.",
            "Precharge/contactor sequencing is assumed covered by the Orion Jr BMS's "
            "integrated contactor control (data/te_v059_electrical_system.json); closes "
            "when the actual harness/contactor wiring is drawn and verified against the "
            "BMS's rated switching current.",
            "No thermal analysis of the battery pack, motor drives, or wiring harness "
            "has been performed; closes with a thermal model once enclosure airflow is "
            "designed.",
        ],
        "result": "PASS" if overall_pass else "FAIL",
    }

    out_dir = ROOT / "analysis_out"
    out_dir.mkdir(exist_ok=True)
    (out_dir / "Terrain_Elevate_P1_V0_59_power_budget_audit.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
