# Orin 离线部署与运行说明

本文描述当前 `arm` 分支面向 Orin 的最小主线部署约定。它只覆盖当前已验证的 TensorRT 主路径，不再把旧的 ONNXRuntime GPU 或 legacy 模型路线作为默认选项。

## 目标平台

- JetPack `R36.4.3`
- CUDA `12.6`
- Python `3.10`
- aarch64 Orin

## 当前默认路线

- RobotSide CPG: TensorRT
- NavSide encoder: TensorRT
- NavSide policy: TensorRT

当前已验证的是 `mujoco_sim_mini` 闭环路线。`realrobot_mini` 当前只确认到 Orin 本体可编译，不作为“真实硬件已运行验证”结论。

## 必需模型与 engine

NavSide 当前 TensorRT 路线必需资产：

- `NavSide/asset/models/vae_pretrain_new.onnx`
- `NavSide/asset/models/vae_pretrain_new.plan`
- `NavSide/asset/models/policy_1.onnx`
- `NavSide/asset/models/policy_1.plan`

RobotSide 当前 CPG TensorRT 路线必需资产：

- `PhybotSoftware_c2/RL_deploy_cpg/model/phybot_cpg_policy.onnx`

## 运行命令

RobotSide:

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2/build
./main
```

NavSide:

```bash
cd /home/amov/nav_arm_mujoco/NavSide
PYTHONPATH=/home/amov/nav_arm_mujoco/NavSide:/home/amov/nav_arm_mujoco/NavSide/cpp_trt/build \
python3 -u -B scripts/run_nav.py --sim-control
```

## timing / perf 开关

默认关闭，打开后只用于观测，不改变控制逻辑。

- `NAVSIDE_TIMING=1`
- `ROBOTSIDE_TIMING=1`
- `SRU_TIMING=1`

## 成功日志

当前已确认的关键日志包括：

- `backend=tensorrt`
- `state_source=nav_state_v2`
- `TensorRT policy init complete`
- `Success transition: 1 → 2`

## 运行限制

- NavSide viewer 必须在本地 Orin 图形会话中运行，不能依赖 remote tty。
- 当前第三方库目录是混合架构资产集合，不能简单假设全部是 ARM。
- 本次验证主要覆盖 `mujoco_sim_mini` 路线，`realrobot_mini` 目前仅完成编译验证。
