# NavSide C++ TensorRT Extension

This directory is the first implementation step for the NavSide TensorRT
backend. It stays separate from the Python navigation flow until integration
is explicitly assigned.

The built extension is `navside_trt.so`.

## Fixed contract

- Encoder engine source: `../asset/models/vae_pretrain_new.onnx`
- Policy engine source: `../asset/models/policy_1.onnx`
- Encoder plan path: `/home/amov/nav_arm_mujoco/NavSide/asset/models/vae_pretrain_new.plan`
- Policy plan path: `/home/amov/nav_arm_mujoco/NavSide/asset/models/policy_1.plan`
- Plans are TensorRT deployment artifacts generated on Orin.
- No runtime engine build in Python.
- No CPU fallback in the target inference path.

## API target

The Python-facing API is intentionally small:

```python
runner = NavSideTRTRunner(encoder_engine_path, policy_engine_path)
depth_features = runner.encode_depth(depth_tensor)
actions, h_out, c_out = runner.run_policy(obs, h_in, c_in)
```

## Implementation

This extension does not use `pybind11`.

Python bindings are implemented with the CPython C API plus the Python buffer
protocol. The extension depends on:

- TensorRT
- CUDA runtime
- Python 3.10 headers and runtime
- NumPy runtime

It does not depend on:

- `pybind11`
- `onnxruntime`

## Contract rules

- Inputs must be `np.float32`
- Inputs must be C-contiguous
- Inputs must match the exact expected shape
- Any mismatch raises immediately
- No implicit cast.
- No implicit reshape.
- No CPU fallback.
- Output arrays are created by the extension itself.

## Expected tensor shapes

Encoder:

- input `depth`: `(1, 1, 40, 64)`
- output `depth_features`: `(1, 64, 5, 8)`

Policy:

- input `obs`: `(1, 2576)`
- input `h_in`: `(1, 1, 512)`
- input `c_in`: `(1, 1, 512)`
- output `actions`: `(1, 3)`
- output `h_out`: `(1, 1, 512)`
- output `c_out`: `(1, 1, 512)`

## Recheck

Minimal rebuild and verification:

```bash
cd /home/amov/nav_arm_mujoco/NavSide/cpp_trt
mkdir -p build
cd build
cmake ..
make -j4
ldd ./navside_trt.so | grep -E "not found|nvinfer|cudart|cuda|python" || true
cd ..
PYTHONPATH=/home/amov/nav_arm_mujoco/NavSide/cpp_trt/build \
python3 tests/smoke_test_runtime.py
```

## Notes

- This module is meant to be loaded from a built `.so` later.
- The existing Python NavSide adapter still owns depth preprocessing, obs
  construction, hidden-state bookkeeping, UDP, and logging.
- This module only owns TensorRT inference.
- The output path is intentionally explicit so later packaging can copy the
  `.plan` files as deployment assets without changing `NavSide/asset/` in this
  phase.
- This CPython binding path is simpler to deploy offline, but it has higher
  maintenance cost than a pybind11-based wrapper.
