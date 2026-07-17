# NavSide Baseline

## Frozen facts
- Encoder model: `models/vae_encoder.onnx`
- Policy model: `models/nav_policy.onnx`
- Depth embedding dim: 2560
- Obs dim: 2576
- Control gate: 5 Hz
- Walk threshold: 0.3
- LOW_SPEED clamp:
  - vx <= 0.6
  - wz <= 0.8
- MEDIUM_SPEED clamp:
  - vx <= 1.0
  - wz <= 1.3
- EMERGENCY:
  - policy still runs
  - final UDP command is forced to `0,0,0`
- Goal stop: enabled
- A / G:
  - reset recurrent state
  - send 3 zero packets
- No old independent RGB/depth window path in the current NavSide flow

## Intent
Keep only the SRU navigation core plus the human-interaction safety state
machine required for sim2sim.
Do not bring VIPlanner legacy paths into NavSide.

## Sim mode
- Recommended launch:
  - `/home/ubuntu/miniconda3/envs/env_sru_ort/bin/python -B scripts/run_nav.py --sim-control --config config/nav.yaml`
- `--sim-control`: MuJoCo viewer + terminal ANSI panel + SRU policy + UDP
  command send to robot-side `8080`.
- Keyboard mapping:
  - `A + Enter`: STANDBY
  - `S + Enter`: LOW_SPEED
  - `D + Enter`: MEDIUM_SPEED
  - `F + Enter`: EMERGENCY
  - `G + Enter`: return to STANDBY without exiting
- Preferred robot-side state is `NavStatePacketV2`, 84 bytes, parsed as `<IHHId16f`.

## Validation summary
- `py_compile`: passed
- XML check: passed, `camera_id=0`
- short interactive run: passed under `env_sru_ort`
- ANSI panel interaction: passed
- local UDP listener on `127.0.0.1:8080`: passed
- STANDBY periodic zero output: passed
- LOW_SPEED limit output: passed
- EMERGENCY immediate zero / sustained zero: passed
- G return to STANDBY without exit: passed
- `Ctrl+C` cleanup and final zero protection: passed
- Robot-side real consumption: pending
