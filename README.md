# sru_mujoco_sim

## IsaacLab/SRU 到 MuJoCo 的 sim2sim 导航闭环

本项目当前目标是把 IsaacLab/SRU 中训练得到的高层导航策略部署到
MuJoCo 中，并接入 Phybot 的 RobotSide CPG/RL 低层行走控制器，形成
一个可以在 MuJoCo 室内场景中运行的端到端 sim2sim 导航闭环。

当前系统不是训练工程，而是部署和联调工程：

```text
ProcTHOR / MuJoCo 场景
        ↓
NavSide 渲染 head_camera 深度图
        ↓
VAE encoder ONNX
        ↓
SRU navigation policy ONNX
        ↓
NavSide 状态机限速 / 急停 / 待机决策
        ↓
UDP 发送 vx, vy, wz
        ↓
RobotSide CPG/RL locomotion policy
        ↓
MuJoCo 机器人动力学执行
        ↓
RobotSide 回传 NavStatePacketV2
        ↓
NavSide 下一周期导航推理
```

NavSide 负责视觉导航、ONNX 推理、人机交互状态机和高层速度输出。
RobotSide 负责 MuJoCo 物理仿真、状态机、CPG/RL 行走控制和机器人状态回传。

---

## 当前工作空间架构

```text
sru_mujoco_sim/
├── README.md
├── NavSide/
│   ├── scripts/run_nav.py
│   ├── navside/
│   │   ├── mode.py        # 人机交互状态机 + ANSI 面板
│   │   ├── sim.py         # MuJoCo viewer + NavSide 主循环
│   │   ├── runtime.py     # NavSideApp，单次导航推理封装
│   │   ├── adapter.py     # depth -> ONNX encoder -> obs -> ONNX policy -> cmd
│   │   ├── bridge.py      # UDP 状态包解析和速度命令发送
│   │   ├── depth.py       # MuJoCo depth 渲染
│   │   ├── image.py       # 无 cv2 的本地图像 resize 工具
│   │   └── state.py       # SRU 机器人状态 dataclass
│   ├── config/
│   │   ├── nav.yaml
│   │   ├── nav_molmospaces.yaml
│   │   └── nav_molmospaces_procthor.yaml
│   ├── asset/
│   │   ├── models/
│   │   │   ├── vae_encoder.onnx
│   │   │   └── nav_policy.onnx
│   │   ├── robot/
│   │   └── merged/
│   └── docs/
│       ├── navside_readme.md
│       └── navside_baseline.md
└── PhybotSofware/
    ├── build/main      # 当前 RobotSide 启动入口
    ├── RL_deploy_cpg/
    ├── RL_deploy_mimic/
    ├── ZeroState/
    ├── StateMachine/
    ├── MujocoInterface/
    └── ...
```

---

## 运行环境

### NavSide 推荐 Python

不要使用系统 `/usr/bin/python3` 直接运行 NavSide。推荐在目标设备上准备一个
Conda 环境，环境名建议为：

```bash
env_sru_ort
```

该环境需要至少能导入：

```text
numpy
yaml / PyYAML
onnxruntime
mujoco
```

在当前开发设备上，这个环境的 Python 路径是：

```bash
/home/ubuntu/miniconda3/envs/env_sru_ort/bin/python
```

其他设备不应依赖这个绝对路径；优先使用：

```bash
conda activate env_sru_ort
python -c "import numpy, yaml, onnxruntime, mujoco; print('NavSide env ok')"
```

### RobotSide 运行环境

RobotSide 当前使用 `PhybotSofware/build/main` 作为运行入口。
它是已经构建好的 MuJoCo/RobotSide 程序，不通过 NavSide 的 Python 环境启动。

---

## 部署和启动顺序

推荐使用两个终端。

### 1. 启动 RobotSide

终端 1：

```bash
cd /home/ubuntu/sru_mujoco_sim/PhybotSofware/build
./main
```

启动后程序会询问：

```text
Start in RL_walk state? [y/N]:
```

输入：

```text
y
```

正常启动日志中应看到类似信息：

```text
MuJoCo version 3.1.1
Start in RL_walk state? [y/N]: y
UDP Socket Initialized on port 8080
...
interface load success!
model load success!
Success transition: 1 → 2
```

RobotSide 会监听 UDP `8080`，接收 NavSide 发送的 `vx, vy, wz`。

日志中可能出现：

```text
[RobotSide] NavSide spawn config unavailable at ../NavSide/config/nav_molmospaces_procthor.yaml: bad file: ../NavSide/config/nav_molmospaces_procthor.yaml
[RobotSide] spawn source=../../NavSide/config/nav_molmospaces_procthor.yaml
[RobotSide] applied spawn_w=[15, 6.5, 0.695] spawn_yaw=1.5708
```

这里第一行是相对路径尝试失败；如果后面出现 `spawn source=...` 和
`applied spawn_w=...`，说明 fallback 路径已成功读取并应用出生点配置。

如果出现：

```text
xbox init fail
```

在当前 sim2sim 链路中不一定是致命错误。RobotSide 主要通过 UDP 接收 NavSide
速度命令，不依赖 Xbox 手柄完成本次导航闭环验证。

### 2. 启动 NavSide

终端 2：

```bash
cd /home/ubuntu/sru_mujoco_sim/NavSide
conda activate env_sru_ort
python -B scripts/run_nav.py --sim-control --config config/nav.yaml
```

启动后 NavSide 会：

- 打开 MuJoCo viewer；
- 启动终端 ANSI 状态面板；
- 在默认 `STANDBY` 下持续发送零速；
- 接收 RobotSide 回传的 `NavStatePacketV2`；
- 根据用户输入切换导航模式。

#如果需要相机的RGB以及DEEP视角，可以用--show-camera启动gui

```bash
cd /home/ubuntu/sru_mujoco_sim/NavSide
conda activate env_sru_ort
python3 -B scripts/run_nav.py \
    --sim-control \
    --config config/nav.yaml \
    --show-camera
```

启动后 NavSide 会：

- 打开 MuJoCo viewer；
- 打开 RGB、DEEP GUI；
- 启动终端 ANSI 状态面板；
- 在默认 `STANDBY` 下持续发送零速；
- 接收 RobotSide 回传的 `NavStatePacketV2`；
- 根据用户输入切换导航模式。


---

## NavSide 人机交互状态机

启动后默认进入 `STANDBY`。

按键需要在 NavSide 终端输入字母后按 Enter。

| 功能 | 按键 | 状态 | 网络推理 | UDP 输出 |
|---|---:|---|---|---|
| 待机 | `A + Enter` | `STANDBY` | 停止 | `0,0,0` |
| 低速运行 | `S + Enter` | `LOW_SPEED` | 运行 | 限速输出 |
| 中速运行 | `D + Enter` | `MEDIUM_SPEED` | 运行 | 限速输出 |
| 紧急制动 | `F + Enter` | `EMERGENCY` | 继续运行 | 强制 `0,0,0` |
| 退出到待机 | `G + Enter` | `STANDBY` | 停止 | `0,0,0` |

注意：

- `G` 是“Q/退出模式”的实际按键，但它不退出程序，只回到待机。
- `F` 急停后可以直接按 `S` 或 `D` 恢复运行。
- `A/G` 会重置 SRU policy 的 recurrent state，并发送 3 个零速包。
- `F` 不重置 recurrent state，因为 Emergency 期间网络仍继续推理。
- `vy` 在 NavSide 输出端保持强制为 `0`。

限速规则：

```text
LOW_SPEED:
  vx <= 0.6
  |wz| <= 0.8

MEDIUM_SPEED:
  vx <= 1.0
  |wz| <= 1.3

EMERGENCY:
  final UDP command = 0,0,0
  policy_cmd 仍会在面板中更新，用于确认网络继续运行
```

---

## NavSide 模型输入和推理链路

当前 ONNX 模型：

```text
NavSide/asset/models/vae_encoder.onnx
NavSide/asset/models/nav_policy.onnx
```

深度图处理流程：

```text
MuJoCo head_camera
→ 渲染 1920 × 1200
→ 中心裁剪 1728 × 1080
→ resize 到 64 × 40
→ 输入 shape = [1, 1, 40, 64]
→ VAE encoder
→ 2560 维 depth feature
```

导航观测：

```text
linear_vel_b[3]
angular_vel_b[3]
projected_gravity_b[3]
last_action[3]
target_position[4]
depth_feature[2560]
```

总维度：

```text
obs dim = 2576
```

策略输出：

```text
raw_action[3]
→ tanh + scale
→ cmd_vel = vx, vy, wz
→ 状态机限速或强制零速
→ UDP 发送到 RobotSide
```

控制频率：

```text
5 Hz
```

---

## UDP 通信契约

### NavSide → RobotSide

NavSide 发送到：

```text
127.0.0.1:8080
```

数据格式：

```text
struct 3f
vx, vy, wz
```

RobotSide 接收到后写入低层控制命令：

```text
js_vx_desire
js_vy_desire
js_OmegaZ_desire
```

### RobotSide → NavSide

RobotSide 回传到：

```text
127.0.0.1:8081
```

NavSide 期望状态包：

```text
NavStatePacketV2
84 bytes
struct: <IHHId16f
magic: 0x32555253
version: 2
```

包含：

```text
seq
timestamp_sec
linear_vel_b[3]
angular_vel_b[3]
projected_gravity_b[3]
robot_pos_w[3]
robot_quat_wxyz[4]
```

NavSide 面板中如果看到：

```text
state_source: nav_state_v2
```

说明 NavSide 正在使用 RobotSide 回传状态。  
如果看到：

```text
state_source: mujoco_mirror
```

说明 NavSide 当前没有收到 RobotSide 状态包，只能使用本地 MuJoCo 镜像状态。

---

## ProcTHOR / MolmoSpaces 场景

当前已接入 MolmoSpaces/ProcTHOR 多房间场景。主要合并 XML 位于：

```text
NavSide/asset/merged/phybot_molmospaces_procthor_val0.xml
```

ProcTHOR 配置位于：

```text
NavSide/config/nav_molmospaces_procthor.yaml
```

其中包含：

```yaml
sim:
  spawn_w: [15, 6.5, 0.695]
  spawn_yaw: 1.57079632679

goal:
  default_goal_w: [5.0, 3, 0.0]
```

RobotSide 启动时会尝试读取 NavSide 配置中的出生点，使 RobotSide 物理仿真初始位姿与
NavSide 视觉侧一致。

---

## 常用检查命令

### XML 检查

```bash
cd /home/ubuntu/sru_mujoco_sim/NavSide
conda activate env_sru_ort
python tools/check_mujoco_xml.py asset/robot/phybot_mini_mark2/xml/phybot_real.xml --camera-name head_camera
```

成功时应看到：

```text
camera_name=head_camera camera_id=0
```

### Python 编译检查

```bash
cd /home/ubuntu/sru_mujoco_sim/NavSide
conda activate env_sru_ort
python -m py_compile navside/mode.py navside/sim.py navside/runtime.py navside/adapter.py navside/depth.py navside/image.py scripts/run_nav.py
```

成功时无输出。

### 确认没有 OpenCV 依赖

```bash
cd /home/ubuntu/sru_mujoco_sim/NavSide
rg -n "\bcv2\b|opencv|OpenCV" .
```

预期无输出。

### UDP 监听 NavSide 输出

如果暂时不启动 RobotSide，可以用本地 UDP 监听器占用 `8080` 验证 NavSide 发包。
注意：这个监听器不能和 RobotSide 同时监听同一个端口。

```bash
cd /home/ubuntu/sru_mujoco_sim/NavSide
conda activate env_sru_ort
python -c "
import socket, struct, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 8080))
sock.settimeout(30.0)
t0 = time.time()
count = 0
try:
    while time.time() - t0 < 60:
        data, addr = sock.recvfrom(1024)
        if len(data) == 12:
            vx, vy, wz = struct.unpack('3f', data)
            count += 1
            print(f'{time.time()-t0:8.3f}s #{count:03d} cmd=({vx:.4f},{vy:.4f},{wz:.4f}) from={addr}', flush=True)
        else:
            print(f'bad packet len={len(data)} from={addr}', flush=True)
except socket.timeout:
    print('timeout')
finally:
    print(f'total_packets={count}')
"
```

---

## 当前验证状态

已完成：

- `py_compile` 通过；
- MuJoCo XML 检查通过，`head_camera` 的 `camera_id=0`；
- `env_sru_ort` 下 NavSide 可启动；
- MuJoCo viewer 可进入主循环；
- 终端 ANSI 面板可原地刷新；
- A/S/D/F/G 交互验证通过；
- STANDBY 周期零速通过；
- LOW_SPEED 限速输出通过；
- EMERGENCY 即时零速和持续零速通过；
- EMERGENCY 下 `policy_cmd` 仍变化，说明网络继续推理；
- G 回 STANDBY 且不退出程序；
- UDP 本地监听验证通过；
- RobotSide 联调已确认程序可以正常运行。

仍需按具体实验继续观察：

- 真实长时间导航成功率；
- 不同 ProcTHOR 场景和目标点下的避障表现；
- RobotSide 对 A/G 三连零速的下游消费节拍；
- Emergency 长时间保持时的机器人侧响应。

---

## 一句话概括

本项目当前已经形成了 `NavSide 视觉导航 + 人机交互安全状态机 + UDP 高层速度命令`
与 `RobotSide CPG/RL 低层行走控制 + MuJoCo 物理执行 + 状态回传` 的完整 sim2sim 闭环，
可以在 MuJoCo 室内场景中通过 A/S/D/F/G 进行待机、低速、中速、急停和回待机控制。
