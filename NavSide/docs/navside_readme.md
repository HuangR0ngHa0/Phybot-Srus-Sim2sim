# NavSide

This directory is the SRU-only navigation side of `sru_mujoco_sim`.

## What it contains
- `run_nav.py`: thin launcher entry.
- `navside/`: actual implementation package.
  - `navside/mode.py`: human-interaction state machine and ANSI panel.
  - `navside/runtime.py`: top-level orchestration and app entry for `--sim-control`.
  - `navside/adapter.py`: depth -> feature -> obs -> ONNX -> action -> clamp.
  - `navside/bridge.py`: UDP packet parsing and command transport.
  - `navside/depth.py`: MuJoCo depth rendering helpers.
  - `navside/image.py`: local image resize / visualization helpers.
  - `navside/sim.py`: MuJoCo SRU runner for `--sim-control`.
  - `navside/state.py`: shared SRU robot-state dataclass.
- `config/nav.yaml`: local config with model paths and limits.
- `asset/`: copied ONNX models, robot XML, URDF, meshes, and textures.

## Current contract
- policy tick / control gate: 5 Hz
- action output: 3 floats (`vx, vy, wz`), with `vy` forced to 0 on NavSide output
- default mode: `STANDBY`
  - no depth rendering
  - no ONNX policy inference
  - continuous zero command output
- `LOW_SPEED` (`S + Enter`)
  - policy runs
  - `vx_max=0.6`, `wz_max=0.8`
- `MEDIUM_SPEED` (`D + Enter`)
  - policy runs
  - `vx_max=1.0`, `wz_max=1.3`
- `EMERGENCY` (`F + Enter`)
  - policy still runs
  - final UDP output is forced to `0,0,0`
  - `zero_reason=emergency` has priority in the panel
- `A + Enter`
  - return to `STANDBY`
  - reset recurrent state
  - send 3 zero packets
- `G + Enter`
  - return to `STANDBY`
  - does not exit the program
  - reset recurrent state
  - send 3 zero packets

## Sim mode
Recommended runtime environment:

```bash
/home/ubuntu/miniconda3/envs/env_sru_ort/bin/python -B scripts/run_nav.py --sim-control --config config/nav.yaml
```

`--sim-control` opens the MuJoCo viewer, renders depth only in `LOW_SPEED`,
`MEDIUM_SPEED`, and `EMERGENCY`, runs SRU inference, and sends `vx, vy, wz`
to robot-side UDP port `8080`.

NavSide no longer uses the old independent RGB/depth windows. The current UI is:
- MuJoCo viewer
- terminal ANSI panel from `navside/mode.py`

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
transport are expected to be provided by `robotside`.

Robot-side real consumption is still pending; the current validation covers
code paths, short interactive runs, and local UDP listening against
`127.0.0.1:8080`.
