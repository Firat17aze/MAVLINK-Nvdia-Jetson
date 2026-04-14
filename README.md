

This repository is a production-style UAV autonomy scaffold that separates flight-critical and AI decision layers:

- Flight-critical control: `PX4` on `Pixhawk 6X`
- Edge AI autonomy: `Jetson Orin NX` companion
- Control interface: `MAVLink` Offboard setpoints with policy enforcement


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


