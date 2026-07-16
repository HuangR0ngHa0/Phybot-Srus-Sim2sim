# NavSide

This directory is the high-level navigation side of the current TensorRT-based
sim2sim stack.

## What it contains
- `run_nav.py`: thin launcher entry.
- `navside/`: actual implementation package.
  - `navside/runtime.py`: top-level orchestration and app entry for `--sim-control`.
  - `navside/adapter.py`: depth -> feature -> obs -> TensorRT -> action -> clamp.
  - `navside/bridge.py`: UDP packet parsing and command transport.
  - `navside/depth.py`: MuJoCo depth rendering helpers.
  - `navside/sim.py`: MuJoCo SRU runner for `--sim-control`.
  - `navside/state.py`: shared SRU robot-state dataclass.
  - `navside/timing.py`: optional timing / perf helpers.
- `config/nav.yaml`: local config with model paths and limits.
- `asset/`: models, TensorRT engines, robot XML, URDF, meshes, and textures.
- `cpp_trt/`: C++ TensorRT extension used by the default runtime path.

## Current contract
- depth feature: 2560
- obs size: 2576
- policy tick: 8 Hz
- action output: 3 floats
- fail-safe: zero cmd on invalid inputs / goal reached

## Sim mode
`run_nav.py` launches the MuJoCo SRU control loop:

```bash
PYTHONPATH=/path/to/NavSide:/path/to/NavSide/cpp_trt/build \
python3 -u -B scripts/run_nav.py --sim-control
```

`--sim-control` opens the MuJoCo scene, renders depth, runs SRU inference,
and sends `vx, vy, wz` to robot-side UDP port `8080`.

## Robot-side state packets
NavSide expects `NavStatePacketV2`: 84 bytes, parsed as `<IHHId16f`

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

NavSide uses it directly for SRU obs construction.

## Runtime note
This is the navigation core only. Robot-side state publishing and command
transport are expected to be provided by `PhybotSoftware_c2`.
