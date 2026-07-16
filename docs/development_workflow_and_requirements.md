# Development Workflow and Requirements

## Operating Principles

All future work follows three rules.

1. Safety, accuracy, and efficiency are required together.
   A task should not proceed if it is fast but unsafe, safe but based on
   unverified assumptions, or accurate but unnecessarily broad.

2. This planning role designs phases, sets verification criteria, and reviews
   results. Concrete code changes are delegated to another agent.

3. Every delegated result is reviewed from both the project angle and the
   process angle. The review must identify problems, require corrections when
   needed, and record a reflection point for the executing agent.

## Final Deployment Goal

Build an `arm` branch that can be cloned onto the same Orin class and run
without downloading project Python or C++ dependencies from the network.

Platform assumptions:

- JetPack R36.4.3 is already installed.
- CUDA 12.6 is already installed.
- TensorRT is already installed or available through the repository native
  dependency layout.
- Python 3.10 is available.
- Target hardware is the same Orin class as the validated development device.

Runtime acceptance target, deferred until the final phase:

- RobotSide and NavSide run together for 30 seconds.
- RobotSide CPG policy runs with TensorRT.
- NavSide encoder and policy run with TensorRT.
- Target flows do not depend on Torch/libtorch.
- No network access is required for project dependencies after cloning.

## Current Technical Direction

The NavSide GPU backend is C++ TensorRT called from Python.

Current target model set:

- RobotSide CPG: `PhybotSoftware_c2/RL_deploy_cpg/model/phybot_cpg_policy.onnx`
- NavSide encoder: `NavSide/asset/models/vae_pretrain_new.onnx`
- NavSide policy: `NavSide/asset/models/policy.onnx`

The previous GPU ONNXRuntime route is paused because no official PyPI
aarch64 GPU wheel was available for the target Orin combination. The old
`policy.onnx` TensorRT failure is no longer the active route because the
model set has changed.

## Scope

In scope:

- `mujoco_sim_mini` CPG-only TensorRT flow.
- `realrobot_mini` CPG-only TensorRT flow.
- NavSide Python main flow calling a C++ TensorRT inference extension.
- Offline Python wheelhouse for non-GPU Python runtime dependencies.
- Native C++ dependencies under `PhybotSoftware_c2/ThirdParty`.
- Build and launch scripts with repository-relative paths.

Out of scope for the current work:

- Final 30-second dual-process acceptance run.
- MolmoSpaces or ProcTHOR cache packaging.
- Moving ONNX models into `ThirdParty`.
- Moving robot XML, mesh, or texture assets into `ThirdParty`.
- Reinstalling JetPack, CUDA, TensorRT, or GPU drivers.
- Continuing ONNXRuntime GPU wheel source builds unless explicitly reopened.

## Dependency Boundaries

Native C++ dependencies stay in:

```text
PhybotSoftware_c2/ThirdParty/
```

Python wheels stay in:

```text
NavSide/third_party_wheels/
```

Runtime assets stay in existing asset/model locations. They are not moved into
`ThirdParty`.

Do not commit:

- `.venv_navside`
- build directories
- `__pycache__`
- `.pyc`
- temporary logs
- local machine cache directories

## Isolation Rules

NavSide must run in its own virtual environment.

RobotSide must not be built or run from inside the NavSide virtual
environment.

Do not install NavSide Python packages into system Python.

Do not add permanent `PYTHONPATH` or `LD_LIBRARY_PATH` exports in shell startup
files.

If runtime library paths are needed, set them only inside the specific launch
script process.

## Phase Plan

### Phase 0: Documentation Realignment

Purpose:

- Mark the old GPU ONNXRuntime route as paused.
- Record the C++ TensorRT extension route as the current controlling plan.

Success criteria:

- Current requirements and workflow are documented.
- Older ORT documents are clearly marked as historical context.

### Phase 1: NavSide TensorRT Model Interface Audit

Purpose:

- Record exact input and output contracts for `vae_pretrain_new.onnx` and
  `policy.onnx`.

Required evidence:

- TensorRT parse/build result for both models.
- Input names, shapes, and dtypes.
- Output names, shapes, and dtypes.
- Whether either model has dynamic shapes.
- Whether the policy model uses recurrent state inputs or outputs.

No code changes.

### Phase 2: NavSide C++ TensorRT Extension Design

Purpose:

- Design the Python-to-C++ API before implementation.

Expected result:

- Proposed C++ module boundaries.
- Proposed pybind11 API.
- Input/output shape checks.
- Engine loading strategy.
- Error reporting strategy.

No implementation yet unless explicitly assigned.

### Phase 3: NavSide C++ TensorRT Extension Implementation

Purpose:

- Implement the inference extension.

This phase is executed by the implementation agent only after the design phase
is accepted.

### Phase 4: NavSide Adapter Integration

Purpose:

- Replace ONNXRuntime session usage with the accepted C++ TensorRT extension.

Preserve:

- MuJoCo camera/depth flow.
- State and goal logic.
- UDP behavior.
- Control scaling and diagnostics.

### Phase 5: realrobot_mini CPG-Only No-Torch

Purpose:

- Make `realrobot_mini` match the CPG-only direction already validated for
  `mujoco_sim_mini`.

Expected behavior:

- No `RL_deploy_amp` in target realrobot build.
- No `RL_deploy_mimic` in target realrobot build.
- No `StateMachine/src/RL_mimic.cpp` in target realrobot build.
- No `find_package(Torch REQUIRED)` for the target realrobot flow.
- No `${TORCH_LIBRARIES}` for the target realrobot flow.
- Motor, IMU, UDP, and CPG logic are preserved.

### Phase 6: Offline Packaging and Scripts

Purpose:

- Prepare repeatable offline deployment commands.

Expected outputs:

- `NavSide/third_party_wheels/`
- `NavSide/requirements-arm.txt`
- `scripts/bootstrap_orin_offline.sh`
- `scripts/build_navside_trt.sh`
- `scripts/build_robotside_mujoco.sh`
- `scripts/build_robotside_realrobot.sh`
- `scripts/run_navside.sh`
- `scripts/run_robotside_mujoco.sh`

### Phase 7: Final Runtime Acceptance

This phase remains deferred.

Acceptance later:

- Clone the `arm` branch on the same Orin class.
- Run offline bootstrap.
- Start RobotSide and NavSide.
- Verify both run together for 30 seconds.

## Delegated Agent Protocol

Each delegated task must report:

- Phase.
- Commands.
- Files changed.
- What was intentionally not changed.
- Evidence collected.
- Verification result.
- Remaining risks.
- Blocked or complete status.

Delegated agents must not:

- Run final 30-second acceptance unless explicitly assigned.
- Reinstall JetPack, CUDA, TensorRT, or drivers.
- Delete models, logs, source code, or build caches without explicit approval.
- Reintroduce Torch into target `mujoco_sim_mini` or target `realrobot_mini`.
- Reintroduce ONNXRuntime as the target NavSide GPU backend.
- Add CPU fallback to the target NavSide inference path.
- Write machine-specific absolute paths into committed scripts.

## Review Rubric

Every delegated result receives two scores.

Project score:

- Did it advance the phase goal?
- Did it preserve validated RobotSide behavior?
- Did it avoid unrelated scope?
- Did it keep deployment compatible with offline packaging?
- Did it respect the current C++ TensorRT direction?

Process score:

- Did it show evidence before conclusions?
- Did it keep changes minimal?
- Did it avoid guessing when blocked?
- Did it report exact commands and outcomes?
- Did it identify risks honestly?

Score meanings:

- 5: Correct, scoped, verified, no meaningful issue.
- 4: Correct with minor verification or reporting gaps.
- 3: Partially useful but needs follow-up before acceptance.
- 2: Risky, incomplete, or mixed with unrelated changes.
- 1: Wrong direction or likely regression.

Each review must include:

- Project score.
- Process score.
- Accepted or rejected status.
- Specific issues.
- Required correction.
- Reflection point for the executing agent.
