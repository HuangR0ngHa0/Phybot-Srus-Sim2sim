# ARM Offline GPU Deployment Plan

> Superseded note: the NavSide GPU backend plan has changed from GPU
> ONNXRuntime to a C++ TensorRT inference extension called from Python.
> Use `docs/development_workflow_and_requirements.md` as the current
> controlling document. This file is kept as historical context for the
> earlier ONNXRuntime investigation.

## Goal

Build an `arm` branch that can be cloned onto the same Orin class and run without downloading project Python or C++ dependencies from the network.

Platform assumptions:

- JetPack R36.4.3 is already installed.
- CUDA 12.6 is already installed.
- TensorRT is already installed or provided by the repository `ThirdParty` layout.
- Python 3.10 is available.
- The target machine is the same Orin class as the validated development device.

Runtime acceptance target:

- RobotSide and NavSide can run together for 30 seconds.
- RobotSide uses CPG with TensorRT/ONNX.
- NavSide uses GPU ONNXRuntime through `CUDAExecutionProvider`.
- NavSide must fail fast if no GPU provider is available.
- No Torch/libtorch dependency is required for the target `mujoco_sim_mini` or `realrobot_mini` CPG-only flows.

## Scope

In scope:

- `PhybotSoftware_c2` C++ RobotSide deployment.
- `NavSide` Python runtime deployment.
- `PhybotSoftware_c2/ThirdParty` native C++ dependencies.
- `NavSide/third_party_wheels` offline Python wheelhouse.
- CPG-only `mujoco_sim_mini`.
- CPG-only `realrobot_mini`.
- GPU ONNXRuntime for `NavSide`.
- Deployment scripts and documentation.

Out of scope for the current plan:

- Final 30-second cross-process acceptance test phase. This is intentionally postponed.
- MolmoSpaces or ProcTHOR cache packaging.
- Moving ONNX models into `ThirdParty`.
- Moving robot XML, mesh, or texture assets into `ThirdParty`.
- Reworking `policy.onnx` for TensorRT engine build.
- Reinstalling JetPack, CUDA, TensorRT, or GPU drivers.

## Dependency Boundaries

Keep native C++ dependencies in:

```text
PhybotSoftware_c2/ThirdParty/
```

Expected native dependency groups:

- `mujoco`
- `TensorRT`
- `glfw`
- `gl`
- `yaml`
- `eigen3`
- `boost`
- `urdfdom`
- `pinocchio`
- `qpOASES`
- `LCM`
- `spdlog`
- `nlohmann`

Keep Python wheels in:

```text
NavSide/third_party_wheels/
```

Expected Python wheel groups:

- `numpy`
- `scipy`
- `PyYAML`
- `mujoco`
- `glfw`
- GPU-capable `onnxruntime`

Keep runtime assets in their existing asset/model locations, not in `ThirdParty`.

Do not commit:

- `.venv_navside`
- build directories
- `__pycache__`
- `.pyc`
- temporary logs
- local machine cache directories

## Isolation Rules

NavSide must run in its own virtual environment.

RobotSide must not be built or run from inside `.venv_navside`.

Do not install NavSide Python packages into system Python.

Do not add permanent `PYTHONPATH` or `LD_LIBRARY_PATH` exports in shell startup files.

If runtime library paths are needed, set them inside the specific launch script process only.

## Known Current Facts

RobotSide `mujoco_sim_mini` has already been validated on Orin:

- CMake configure succeeded.
- `make -j4` succeeded.
- `ldd ./main` had no missing libraries.
- `ldd ./main` had no Torch.
- `RL_deploy_cpg` initialized TensorRT successfully.
- `Input dims: [1, 283]`.
- `Output dims: [1, 21]`.
- A local Orin 30-second run was stable.

NavSide current runtime state:

- Current ONNXRuntime version observed: `1.23.2`.
- Current providers observed: `AzureExecutionProvider`, `CPUExecutionProvider`.
- `CUDAExecutionProvider` is not currently available.
- `policy.onnx` fails direct TensorRT parsing because an `/If` node has incompatible branch output shapes.
- The chosen route for NavSide is GPU ONNXRuntime, not TensorRT engine build.

## Phase Plan

### Phase A: GPU ONNXRuntime Feasibility

Purpose:

- Find or validate an aarch64 Python 3.10 ONNXRuntime build that supports CUDA on JetPack R36.4.3 / CUDA 12.6.

Success criteria:

- `onnxruntime.get_available_providers()` includes `CUDAExecutionProvider`.
- Both `NavSide/asset/models/vae_encoder.onnx` and `NavSide/asset/models/policy.onnx` can create ONNXRuntime sessions with CUDA provider.

No code changes unless the only needed change is a diagnostic script or documentation update.

### Phase B: NavSide Offline Wheelhouse

Purpose:

- Make NavSide installable without network access.

Expected outputs:

- `NavSide/third_party_wheels/`
- `NavSide/requirements-arm.txt`
- a short offline install verification command.

### Phase C: NavSide GPU-Only Runtime

Purpose:

- Remove CPU fallback from NavSide.

Expected behavior:

- If `CUDAExecutionProvider` is missing, NavSide exits with a clear error.
- Startup logs print the actual providers used.

### Phase D: realrobot_mini CPG-Only No-Torch

Purpose:

- Make `realrobot_mini` match the already validated CPG-only direction used by `mujoco_sim_mini`.

Expected behavior:

- `realrobot_mini` does not compile `RL_deploy_amp`.
- `realrobot_mini` does not compile `RL_deploy_mimic`.
- `realrobot_mini` does not compile `StateMachine/src/RL_mimic.cpp`.
- `realrobot_mini` does not call `find_package(Torch REQUIRED)`.
- `realrobot_mini` does not link `${TORCH_LIBRARIES}`.
- Hardware-related logic such as motor, IMU, and UDP is preserved.

### Phase E: Deployment Scripts

Purpose:

- Provide repeatable commands for offline setup and launch.

Expected scripts:

- `scripts/bootstrap_orin_offline.sh`
- `scripts/build_robotside_mujoco.sh`
- `scripts/build_robotside_realrobot.sh`
- `scripts/run_robotside_mujoco.sh`
- `scripts/run_navside.sh`

Scripts must auto-detect the repository root and avoid machine-specific absolute paths.

### Phase F: Branch Packaging

Purpose:

- Clean the `arm` branch into a deployable package.

Expected checks:

- No virtual environment committed.
- No build artifacts committed.
- ThirdParty native libraries present.
- NavSide wheelhouse present.
- Documentation present.

### Phase G: Final Runtime Acceptance

This phase is intentionally deferred for now.

Acceptance later:

- Clone `arm` branch on same-class Orin.
- Run offline bootstrap.
- Start RobotSide and NavSide.
- Verify both run together for 30 seconds.

## Agent Execution Protocol

Each delegated agent task must include:

- Phase name.
- Exact working directory.
- Commands run.
- Files changed.
- What was intentionally not changed.
- Verification output summary.
- Remaining risks.
- Blocked or complete status.

Agents must not:

- Run final long acceptance unless explicitly assigned.
- Reinstall JetPack, CUDA, TensorRT, or drivers.
- Delete models, logs, source code, or build caches without explicit approval.
- Reintroduce Torch into target `mujoco_sim_mini` or `realrobot_mini`.
- Add CPU fallback to NavSide.
- Write machine-specific absolute paths into committed scripts.

## Review Rubric

Every agent result should be reviewed from two angles.

Project angle:

- Does it advance the stated phase goal?
- Does it preserve the already validated RobotSide `mujoco_sim_mini` behavior?
- Does it keep NavSide isolated in `.venv_navside`?
- Does it keep deployment offline-compatible?
- Does it avoid unrelated changes?

Process angle:

- Did the agent show evidence before modifying files?
- Did it keep changes minimal?
- Did it avoid guessing when blocked?
- Did it report exact commands and outcomes?
- Did it identify remaining risks honestly?

Scoring:

- 5: Correct, scoped, verified, no meaningful issues.
- 4: Correct with minor reporting or verification gaps.
- 3: Partially useful but needs follow-up before accepting.
- 2: Risky, incomplete, or mixed unrelated changes.
- 1: Incorrect direction or likely regression.

The review should include:

- Project score.
- Process score.
- Accepted or rejected status.
- Specific issues.
- Required correction.
- Reflection point the agent should carry forward.
