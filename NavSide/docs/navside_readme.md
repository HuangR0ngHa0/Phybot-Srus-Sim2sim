# NavSide

This directory is the SRU-only navigation side of `sru_mujoco_sim`.

## What it contains
- `run_nav.py`: thin launcher entry.
- `navside/`: actual implementation package.
  - `navside/cli.py`: argument parsing and top-level entry.
  - `navside/app.py`: SRU app orchestration.
  - `navside/adapter.py`: depth -> feature -> obs -> ONNX -> action -> clamp.
  - `navside/bridge.py`: UDP packet parsing and command transport.
  - `navside/depth.py`: MuJoCo depth rendering helpers.
  - `navside/sim.py`: MuJoCo SRU runner for `--sim-dry-run` / `--sim-control`.
  - `navside/config.py`: config dataclass and loader.
  - `navside/state.py`: shared SRU robot-state dataclass.
- `sru_app.py`, `sru_nav_adapter.py`, `udp_bridge.py`, `mujoco_depth_source.py`, `mujoco_sim_runner.py`: compatibility shims that re-export the package modules.
- `config/nav.yaml`: local config with model paths and limits.
- `models/`: copied ONNX models used by NavSide.
- `smoke_test/`: minimal local checks.

## Current contract
- depth feature: 2560
- obs size: 2576
- policy tick: 5 Hz
- action output: 3 floats
- fail-safe: zero cmd on invalid inputs / goal reached

## Sim mode
`run_nav.py` also has a VIPlanner-compatible MuJoCo runner:

```bash
python3 -B run_nav.py --sim-dry-run
python3 -B run_nav.py --sim-control
```

`--sim-dry-run` opens the MuJoCo scene, renders depth, and runs SRU diagnostics.
`--sim-control` additionally sends `vx, vy, wz` to robot-side UDP port `8080`
and listens for the legacy `x, y, yaw` state packet on `8081`.

## Robot-side state packets
NavSide accepts both robot-side state formats:

- Legacy `3f`: `x`, `y`, `yaw`
- `NavStatePacketV2`: 84 bytes, parsed as `<IHHId16f`

`NavStatePacketV2` contains:

- `magic`: `0x32555253`
- `version`: `2`
- `seq`
- `timestamp_sec`
- `linear_vel_b[3]`
- `angular_vel_b[3]`
- `projected_gravity_b[3]`
- `robot_pos_w[3]`
- `robot_quat_wxyz[4]`

When V2 is received, NavSide uses it directly for SRU obs construction.
When only legacy `3f` is received, NavSide falls back to the MuJoCo mirror
state estimator.

## Runtime note
This is the navigation core only. Robot-side state publishing and command transport are expected to be provided by `robotside`.
