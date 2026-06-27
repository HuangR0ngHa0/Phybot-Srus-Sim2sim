## IsaacLab 到 MuJoCo 的 sim2sim 工作总结

本阶段工作的核心不是重新训练导航策略，而是将 IsaacLab/SRU 中训练得到的高层导航模型迁移到 MuJoCo 环境中，接入 Phybot 机器人低层 CPG/RL 行走控制器，构建完整的 sim2sim 闭环导航系统，用于验证策略在不同仿真器、不同机器人动力学和更真实室内场景中的运行效果。

### 1. 搭建 MuJoCo 侧导航闭环架构

完成了从 IsaacLab 训练模型到 MuJoCo 机器人执行的完整链路：

```text
IsaacLab/SRU 训练模型
        ↓
ONNX 导航模型
        ↓
NavSide 深度渲染与状态构造
        ↓
SRU 导航策略推理
        ↓
输出 vx、vy、wz
        ↓
UDP 发送速度命令
        ↓
RobotSide CPG/RL locomotion policy
        ↓
MuJoCo 机器人动力学执行
        ↓
机器人状态通过 UDP 回传 NavSide
```

其中，NavSide 负责高层视觉导航推理，RobotSide 负责机器人动力学仿真、状态机和低层行走控制，二者通过 UDP 形成状态反馈与速度控制闭环。

### 2. 完成导航模型 ONNX 推理接入

将训练侧导航模型部署到 NavSide，当前使用的主要模型包括：

* `vae_encoder.onnx`
* `nav_policy.onnx`

NavSide 将 MuJoCo 深度图和机器人状态转换为训练模型所需的观测格式：

```text
深度图
→ 裁剪与缩放
→ VAE encoder
→ 2560 维视觉特征

视觉特征 + 16 维机器人状态
→ 2576 维导航观测
→ SRU policy
→ vx、vy、wz
```

同时保留循环网络的隐藏状态更新，使策略能够连续利用历史观测，而不是仅依赖单帧深度图进行决策。

### 3. 接入 MolmoSpaces/ProcTHOR 真实室内场景

为避免只在空地或简单障碍物环境中测试，接入了 MolmoSpaces 场景资源。

前期尝试了 iTHOR `FloorPlan1`，但该场景面积较小，更偏向视觉操作和 VLA 任务，不适合长距离导航测试。后续改用 ProcTHOR-10k 的 `val_0` 多房间场景，并生成机器人与环境合并后的 MuJoCo XML：

```text
NavSide/asset/merged/
phybot_molmospaces_procthor_val0.xml
```

该场景包含 Phybot 机器人、墙体、房间结构和室内障碍物，可用于测试跨房间导航、转角避障和长距离目标跟踪。

场景合并工具还完成了：

* 场景资源路径处理；
* XML 资产合并；
* 场景对象名称前缀化；
* 名称冲突避免；
* 原有测试障碍物清理；
* 机器人出生位置配置；
* MuJoCo XML 可加载性检查。

### 4. 对齐 IsaacLab 训练侧深度相机输入

针对训练侧和 MuJoCo 侧深度图分布不一致的问题，对相机输入链路进行了严格适配。

训练侧等效处理流程为：

```text
原始分辨率：1920 × 1200
中心裁剪：1728 × 1080
最终输入：64 × 40
深度范围：0.10 ～ 10.0 m
垂直视场角：57°
导航频率：约 5 Hz
```

MuJoCo 侧最终复现为：

```text
MuJoCo head_camera
→ fovy = 57°
→ 渲染 1920 × 1200
→ 中心裁剪 1728 × 1080
→ resize 到 64 × 40
→ 输入 shape = [1, 1, 40, 64]
→ SRU depth encoder
```

相比早期直接采用 `848 × 480 → 64 × 40` 的简化方式，该方案更接近 IsaacLab 训练时的相机视场和图像处理分布，从而减少 sim2sim 中的视觉输入偏差。

同时增加了启动阶段的分辨率硬校验，避免配置错误时继续运行：

```text
renderer size must be at least 1920 × 1200
```

GUI 中的 RGB 和 depth 窗口采用单独缩放显示，该缩放仅影响可视化，不改变模型输入。

### 5. 打通 NavSide 与 RobotSide 的 UDP 状态和控制接口

NavSide 与 RobotSide 之间实现了双向 UDP 通信。

NavSide 向 RobotSide 发送：

```text
vx
vy
wz
```

RobotSide 向 NavSide 回传 `NavStatePacketV2`，主要包含：

```text
序列号
时间戳
本体坐标系线速度
本体坐标系角速度
投影重力方向
机器人世界坐标位置
机器人世界坐标四元数
```

RobotSide 采用非阻塞 UDP 接收方式，避免网络通信阻塞底层控制循环。

接收到导航速度后，RobotSide 将其写入低层控制命令：

```text
js_vx_desire
js_vy_desire
js_OmegaZ_desire
```

经过速度缩放后拼接到底层 locomotion policy 的观测中，由 CPG/RL 行走策略生成机器人关节控制量。

RobotSide 同时从 MuJoCo 和 `DataPackage` 中读取机器人位置、姿态、角速度和重力方向，并回传给 NavSide。NavSide 使用 `nav_state_v2` 作为真实状态源，从而形成真正的闭环，而不是仅依赖本地 MuJoCo 镜像状态。

### 6. 实现出生点、朝向和目标点统一配置

在 NavSide 配置中增加了：

```yaml
sim:
  spawn_w: [x, y, z]
  spawn_yaw: yaw

goal:
  default_goal_w: [x, y, z]
```

当前可以通过：

```text
NavSide/config/nav_molmospaces_procthor.yaml
```

统一配置：

* 机器人出生位置；
* 机器人初始朝向；
* 导航目标点。

早期仅修改 NavSide 出生点时，RobotSide 的状态会通过 UDP 将其覆盖，导致两侧初始位置不一致。

为解决该问题，修改了 RobotSide MuJoCo 启动程序，使其启动时读取同一份 NavSide 配置文件，并将 `spawn_w` 和 `spawn_yaw` 写入 MuJoCo 初始 `qpos`。由此保证：

```text
NavSide 视觉场景初始位姿
=
RobotSide 物理仿真初始位姿
```

### 7. 增加可视化和调试工具

为了便于检查导航链路，增加了以下可视化功能：

* NavSide RGB 图像窗口；
* NavSide depth 图像窗口；
* MuJoCo Viewer；
* 出生点和目标点标记；
* 状态来源、速度命令、机器人位置和目标距离日志。

标记定义为：

```text
蓝色：机器人出生点
红色：导航目标点
```

这些 marker：

* 不参与物理碰撞；
* 不进入机器人 RGB/depth 相机；
* 不影响导航策略输入；
* 只用于上帝视角调试。

同时保留相机外参调整接口，可通过修改 `head_camera` 的位置参数调整相机安装高度和偏移。

### 8. 当前 sim2sim 验证结果

目前已完成并验证：

* MuJoCo 合并场景 XML 能够正常加载；
* ProcTHOR 多房间场景能够显示；
* `head_camera` 能够被正确识别；
* RGB 和 depth 渲染正常；
* 相机分辨率、视场角和深度范围已与训练侧对齐；
* ONNX encoder 和 navigation policy 能够正常初始化；
* SRU 导航推理能够连续运行；
* NavSide 能够输出 `vx、vy、wz`；
* RobotSide 能够接收并执行速度命令；
* RobotSide 状态能够通过 `nav_state_v2` 回传；
* NavSide 能够使用回传状态更新机器人位姿；
* `robot_xy` 和 `goal_dist` 能够随机器人运动变化；
* NavSide 与 RobotSide 出生点保持一致；
* 整体系统已形成完整 sim2sim 闭环。

日志中出现：

```text
state_source=nav_state_v2
cmd=[...]
robot_xy=(...)
goal_dist=...
```

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
