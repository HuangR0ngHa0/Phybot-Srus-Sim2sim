# NavSide Baseline

## Frozen facts
- Backend: `tensorrt`
- Encoder model: `models/vae_pretrain_new.onnx`
- Encoder engine: `models/vae_pretrain_new.plan`
- Policy model: `models/policy_1.onnx`
- Policy engine: `models/policy_1.plan`
- Depth embedding dim: 2560
- Obs dim: 2576
- Control gate: 8 Hz
- Walk threshold: 0.3
- Goal stop: enabled

## Intent
Keep only the SRU navigation core required for the current TensorRT sim2sim path.

## Sim mode
- `--sim-control`: MuJoCo depth + SRU policy + UDP command send to robot-side `8080`.
- Preferred robot-side state is `NavStatePacketV2`, 84 bytes, parsed as `<IHHId16f`.
