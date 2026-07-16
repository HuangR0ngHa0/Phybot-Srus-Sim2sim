# Orin 离线部署与运行说明

本文描述当前 `arm` 分支面向 Orin 的离线部署约定。它对应的是当前已验证的 TensorRT 路线，不是旧的 ONNXRuntime GPU 路线。

## 目标平台

- JetPack `R36.4.3`
- CUDA `12.6`
- Python `3.10`
- aarch64 Orin

## 当前默认路线

- RobotSide CPG: TensorRT
- NavSide encoder: TensorRT
- NavSide policy: TensorRT

当前已验证的是 `mujoco_sim_mini` 路线。`realrobot_mini` 未在本次部署说明里作为验证结论。

## 必需模型与 engine

NavSide 当前 TensorRT 路线必需资产：

- `NavSide/asset/models/vae_pretrain_new.onnx`
- `NavSide/asset/models/vae_pretrain_new.plan`
- `NavSide/asset/models/policy_1.onnx`
- `NavSide/asset/models/policy_1.plan`

RobotSide 当前 CPG TensorRT 路线必需资产：

- `PhybotSoftware_c2/RL_deploy_cpg/model/phybot_cpg_policy.onnx`

## 旧模型说明

以下资产属于 legacy / baseline，不是当前默认 TensorRT 主路径必需项：

- `policy.onnx`
- `vae_encoder.onnx`
- `nav_policy.onnx`

这些资产可以保留用于历史对照或回退分析，但不应作为当前默认 TensorRT 路线的依赖前提。

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
- 本次验证主要覆盖 `mujoco_sim_mini` 路线，不代表 `realrobot_mini` 已完成同等级验证。

## 旧 ORT 方案状态

旧的 ONNXRuntime GPU 方案已被 TensorRT 路线 superseded。相关历史文档可以保留为背景资料，但不应被误解为当前默认部署路径。

