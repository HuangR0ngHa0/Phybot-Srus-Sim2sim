# ONNX Runtime GPU Build Notes for Orin

This document only records a build plan and a read-only preflight checklist.
It does not execute the build.

> Superseded note: NavSide no longer uses GPU ONNXRuntime as the default
> backend. The current controlling document is
> `docs/development_workflow_and_requirements.md`, and the current NavSide
> deployment route is the C++ TensorRT extension under `NavSide/cpp_trt/`.
> Keep this file only as historical context for the earlier ONNXRuntime
> investigation.

## Target Platform

- JetPack `R36.4.3`
- CUDA `12.6`
- Python `3.10`
- Architecture `aarch64`

## Goal

Build an ONNX Runtime wheel that exposes `CUDAExecutionProvider` for NavSide.

## Version Recommendation

- Recommended ORT tag: `v1.23.2`

Reason:

- It matches the currently observed runtime version.
- It keeps the first build attempt on a known Python 3.10-compatible baseline.
- If this build fails, keep the failure logs and address the failure point before
  changing versions.

## Provider Scope

- Enable `CUDAExecutionProvider`
- Do not enable TensorRT provider in this phase

TensorRT provider can be evaluated later only after CUDA provider is proven to
work.

## Path Assumptions To Verify

The following paths are the current working assumptions for the Orin build
environment:

- `cuda_home=/usr/local/cuda`
- `cudnn_home=/usr`

Important:

- Path existence is not enough by itself.
- The first configure step must still prove that the build system recognizes the
  headers and libraries.
- If configure fails, keep the early logs and do not jump to a new version
  immediately.

## Build Command Draft

Run this from the ONNX Runtime source root, after manual confirmation:

```bash
./build.sh \
  --config Release \
  --build_wheel \
  --enable_pybind \
  --use_cuda \
  --cuda_home /usr/local/cuda \
  --cudnn_home /usr \
  --parallel 4
```

Notes:

- No TensorRT provider flag is included.
- `--parallel 4` is a conservative starting point for this 8-core Orin.
- Do not install missing build dependencies automatically.

## Expected Artifacts

- Wheel output under `build/Linux/Release/dist/*.whl`
- Wheel contents should include `libonnxruntime_providers_cuda.so`

After validation, the wheel can be copied into:

- `NavSide/third_party_wheels/`

## Failure Recovery Strategy

- Keep `build/Linux/Release` unless there is an explicit cleanup decision.
- Do not delete build directories by default.
- Preserve configure and build logs before any retry.
- Re-run only after identifying the failure point.
- Do not change versions on the first failure without evidence from logs.

## Preflight Expectations

Before any long build task, confirm:

- JetPack version
- Python version
- CUDA toolchain visibility
- cuDNN headers and libraries
- TensorRT headers and libraries
- disk, memory, and CPU availability
- `pip`, `wheel`, `setuptools`, `cmake`, `ninja`

If any critical item is missing, stop and report the warning instead of trying
to repair the environment.
