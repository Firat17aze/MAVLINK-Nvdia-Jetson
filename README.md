# Industry-Grade UAV Edge AI (MAVLink + PX4 + Jetson)

This repository is a production-style UAV autonomy scaffold that separates flight-critical and AI decision layers:

- Flight-critical control: `PX4` on `Pixhawk 6X`
- Edge AI autonomy: `Jetson Orin NX` companion
- Control interface: `MAVLink` Offboard setpoints with policy enforcement

## Architecture

Data flow:

`sensor_ingest -> perception_inference -> sensor_fusion -> target_tracker -> mission_manager -> safety_guardian -> mavlink_bridge -> PX4`

Guardrail:

- AI never commands raw actuators.
- Safety guardian is the single command gate before MAVLink output.
- PX4 keeps deterministic failsafe ownership.

## Repository Layout

- `autonomy/`: decision pipeline, bridge, health/failsafe logic, tests
- `flight_stack/`: PX4 profile, firmware guard module, and MAVLink integration contracts
- `hardware/`: BOM, pin matrix, wiring topology, bench checklist
- `safety/`: hazard analysis and safety contract
- `ops/`: security baseline, observability, release/stage gates
- `configs/`: single-source operational limits and thresholds

## Quick Start

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install '.[dev]'
pytest -q
python scripts/run_local_sim.py
python flight_stack/px4/scripts/check_param_profile.py
python scripts/validate_pin_matrix.py
```

## System Baseline

- Companion OS: Ubuntu 22.04
- Companion middleware: ROS 2 Humble target profile
- Flight stack: PX4 stable branch
- Telemetry strategy: RF primary, LTE fallback

## Current Status

This repository includes:

- runnable autonomy modules with deterministic decision/failsafe behavior
- test coverage for mission states, safety policy, fault handling
- PX4 firmware-side companion guard module scaffold with policy enforcement
- hardware pin-level integration artifacts for Pixhawk baseboard ports
- release, security, and validation process templates
