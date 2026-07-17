# Phybot-Srus-Sim2sim

这个仓库当前只保留最小主线交付内容：NavSide 高层导航 TensorRT、RobotSide CPG TensorRT、`mujoco_sim_mini` 闭环验证路径，以及 `realrobot_mini` 的 Orin 本体编译路径。

## 当前交付口径

- `mujoco_sim_mini`
  - 已实现 NavSide TensorRT + RobotSide TensorRT 的双端闭环。
  - 已验证 30 秒稳定运行。
  - 已接 timing / perf 统计。
- `realrobot_mini`
  - 已完成 no-Torch / no-Mocap / ARM MotorDrive SDK 对齐。
  - 已在 Orin 本体编译通过。
  - 尚未作为“真实硬件运行已验证”对外承诺。

## 主要工作链路

```text
NavSide depth + robot state
        ->
NavSide TensorRT encoder (vae_pretrain_new)
        ->
NavSide TensorRT policy (policy_1)
        ->
vx / vy / wz over UDP
        ->
RobotSide CPG TensorRT policy
        ->
MuJoCo or real robot low-level execution
        ->
robot state over UDP
        ->
NavSide state_source=nav_state_v2 closed loop
```

## 仓库结构

```text
NavSide/
  navside/            NavSide 运行时、adapter、sim、timing
  cpp_trt/            NavSide C++ TensorRT extension
  config/             NavSide 配置
  asset/models/       NavSide 当前默认模型与 engine
  scripts/run_nav.py  NavSide 启动入口

PhybotSoftware_c2/
  RobotStart/mujoco_sim/   MuJoCo RobotSide 入口
  RobotStart/realrobot/    realrobot 入口与 set_zero
  RL_deploy_cpg/           当前低层 CPG TensorRT 主路径
  StateMachine/            状态机
  MotorList/               电机接口与 ARM MotorDrive 适配
  MujocoInterface/         MuJoCo 接口
  ThirdParty/TensorRT/     RobotSide TensorRT 依赖

docs/
  arm_orin_deployment.md   Orin 主线部署说明
```

保留但不属于主运行入口的辅助内容：

- `.learnings/`
- `.vscode/`
- `NavSide/docs/`
- `NavSide/tests/`

## 当前默认模型与 backend

NavSide 默认配置见 [NavSide/config/nav.yaml](NavSide/config/nav.yaml)。

- backend: `tensorrt`
- encoder:
  - `NavSide/asset/models/vae_pretrain_new.onnx`
  - `NavSide/asset/models/vae_pretrain_new.plan`
- policy:
  - `NavSide/asset/models/policy_1.onnx`
  - `NavSide/asset/models/policy_1.plan`

RobotSide 默认低层策略：

- `PhybotSoftware_c2/RL_deploy_cpg/model/phybot_cpg_policy.onnx`
- runtime backend: TensorRT

## 获取工作空间后的部署

目标平台：

- JetPack `R36.4.3`
- CUDA `12.6`
- Python `3.10`
- aarch64 Orin

建议目录：

```bash
cd /home/amov
git clone <your-repo-url> nav_arm_mujoco
cd nav_arm_mujoco
git checkout arm
```

### 1. 编译 NavSide C++ TensorRT extension

```bash
cd /home/amov/nav_arm_mujoco/NavSide/cpp_trt
mkdir -p build
cd build
cmake ..
make -j4
```

预期产物：

- `NavSide/cpp_trt/build/navside_trt*.so`

### 2. 编译 RobotSide

RobotSide 推荐使用仓库自带脚本：

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2
./autobuild.sh
```

`autobuild.sh` 会进入交互菜单：

```text
1.realrobot_mini
2.webots_sim
3.mujoco_sim_mini
4.gazebo_sim
```

当前主线只使用：

- 输入 `3` 编译 `mujoco_sim_mini`
- 输入 `1` 编译 `realrobot_mini`

脚本默认使用 `PhybotSoftware_c2/build/` 作为构建目录。切换 `mujoco_sim_mini` 和 `realrobot_mini` 之前，建议先清理同一个 build 目录：

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2
./autoclean.sh
./autobuild.sh
```

预期产物：

- `mujoco_sim_mini`: `PhybotSoftware_c2/build/main`
- `realrobot_mini`: `PhybotSoftware_c2/build/main` and `PhybotSoftware_c2/build/set_zero`

`autobuild.sh` 默认使用 `/usr/local/cuda`。如果 CUDA 安装路径不同，可以显式覆盖：

```bash
CUDA_TOOLKIT_ROOT_DIR=/path/to/cuda ./autobuild.sh
```

`/dev/ttyUSB4` 只属于真实机器人硬件链路。脚本只在选择 `realrobot_mini` 时尝试设置串口权限；如果真实设备路径不同，可以覆盖：

```bash
TTY_DEVICE=/dev/ttyUSB0 ./autobuild.sh
```

手动 CMake 只作为调试方式使用。需要定位 configure/link 错误时可以直接指定 `WHICH_ENV`：

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2
./autoclean.sh
cd build
cmake \
  -DWHICH_ENV=mujoco_sim_mini \
  -DCMAKE_BUILD_TYPE=Release \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  ..
make -j4
```

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2
./autoclean.sh
cd build
cmake \
  -DWHICH_ENV=realrobot_mini \
  -DCMAKE_BUILD_TYPE=Release \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda \
  ..
make -j4
```

## 启动方法

### `mujoco_sim_mini`

先起 RobotSide：

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2/build
./main
```

再起 NavSide：

```bash
cd /home/amov/nav_arm_mujoco/NavSide
PYTHONPATH=/home/amov/nav_arm_mujoco/NavSide:/home/amov/nav_arm_mujoco/NavSide/cpp_trt/build \
python3 -u -B scripts/run_nav.py --sim-control
```

### `realrobot_mini`

`set_zero` 只用于零位相关流程：

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2/build
./set_zero
```

`main` 是真实机器人主程序入口，但当前仓库只承诺“可编译”，不承诺“已完成真实硬件运行验证”：

```bash
cd /home/amov/nav_arm_mujoco/PhybotSoftware_c2/build
./main
```

## timing / perf

默认关闭，打开后只用于观测：

```bash
NAVSIDE_TIMING=1
ROBOTSIDE_TIMING=1
SRU_TIMING=1
```

关键输出：

- `[NavSide TIMING] ...`
- `[NavSide PERF] encoder_trt`
- `[NavSide PERF] policy_trt`
- `[RobotSide PERF] cpg_trt_policy`

## 成功判据

`mujoco_sim_mini` 主线成功时，常见关键日志包括：

- `backend=tensorrt`
- `state_source=nav_state_v2`
- `TensorRT policy init complete`
- `Success transition: 1 → 2`

## 当前限制

- NavSide viewer 必须在 Orin 本地图形会话运行，不能依赖 remote tty。
- 当前主线验证覆盖的是 `mujoco_sim_mini`，不是 `realrobot_mini` 真实硬件运行闭环。
- `ThirdParty` 目录是混合资产集合，不应简单假设全部可跨平台复用。

说明当前系统已经实现：

```text
视觉观测
→ 导航推理
→ 速度控制
→ 低层行走
→ 机器人运动
→ 状态反馈
→ 下一周期导航推理
```

### 总结

本阶段完成了 IsaacLab/SRU 导航策略向 MuJoCo 的 sim2sim 迁移。系统采用 NavSide 与 RobotSide 双侧架构：NavSide 负责 MuJoCo 场景视觉渲染、深度预处理、SRU ONNX 推理和高层速度输出；RobotSide 负责机器人动力学仿真、状态机、CPG/RL 低层行走控制和机器人状态回传。

同时接入了 ProcTHOR 多房间真实室内场景，对齐了 IsaacLab 训练侧的深度相机参数和图像处理流程，补充了出生点、目标点、相机、可视化和 UDP 通信接口，最终实现了从导航感知、策略推理、高层速度输出到低层运动执行和状态反馈的完整闭环。

一句话概括：

> 将 IsaacLab 中训练得到的 SRU 导航策略，通过 ONNX 部署到 MuJoCo 的 NavSide 中，并与 Phybot RobotSide 的 CPG/RL 行走控制器通过 UDP 连接，在 ProcTHOR 多房间场景中实现了深度感知、导航推理、速度控制、物理执行和状态回传的完整 sim2sim 闭环。
