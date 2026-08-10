# ROS 2 六轴机械臂 RCM 控制工作区

- 本项目旨在基于 `ros2_control` 框架，为六轴机械臂（如 AgileX / Piper 系列）开发并实现具备柔顺交互能力的 RCM（Remote Center of Motion，远程运动中心）控制算法。
- Piper 快速入门可参考：[GitHub - kkmjj0721/Quick_Start_Piper](https://github.com/kkmjj0721/Quick_Start_Piper)。
- 更完整的二次开发路线、RCM 数学、安全门控和 PLANNED 模块说明见 [`docs/PIPER_RCM_DEVELOPMENT_GUIDE.md`](docs/PIPER_RCM_DEVELOPMENT_GUIDE.md)。

## 项目介绍

- **核心目标**：基于 `ros2_control` 硬件抽象层开发，实现高实时性的 RCM 约束控制算法。
- **拖动示教与标定（重力补偿）**：规划引入重力补偿模式，抵消机械臂自重，使用户可以拖动机械臂标记 RCM 不动点。
- **遥操作 RCM 控制**：当前仓库已经具备手柄/键盘输入 dry-run 链路；后续控制器将把操作者意图转换为满足 RCM 约束的运动。
- **虚实结合（Sim2Real）**：当前仓库包含 Piper URDF、mesh 和 MuJoCo MJCF 资产；完整 MuJoCo bridge、硬件接口和真机写控制仍属于待实现内容。

> 安全说明：当前 README 中的可运行教程默认只做模型、话题和 dry-run 验证。当前仓库尚未实现真实机械臂写控制、Piper SDK adapter、CAN adapter、RCM solver 或重力补偿控制器，不要把 `/rcm_cmd` 直接接入真实硬件命令。

## 当前状态

| 模块 | 当前状态 | 说明 |
| --- | --- | --- |
| `rcm_teleop` | CURRENT | 可通过手柄或键盘发布 `/joy`，并生成 dry-run 的 `/rcm_mode` 和 `/rcm_cmd`。 |
| `agx_arm_description` | PARTIAL | 已有 Piper URDF、mesh、MuJoCo MJCF；`display_piper.launch.py` 当前为空，URDF 中 mesh package 路径仍需校正后才能稳定用于 RViz。 |
| `agx_arm_controller` | SKELETON | 仅有 ROS 2 包骨架，尚无 RCM 控制器、求解器、重力补偿或测试。 |
| `agx_arm_bringup` | SKELETON | 仅有 ROS 2 包骨架，尚无完整 bringup launch。 |
| `rcm_msgs` / MoveIt / hardware interface | PLANNED | 开发指南中已有推荐路线，当前仓库尚未落地。 |

## 目录结构

```text
src/
├── agx_arm_bringup/          # 后续组合 display、dry-run、sim、readonly、live launch
├── agx_arm_controller/       # 后续放 RCM solver、状态机和 ros2_control controller
├── agx_arm_description/      # Piper URDF、mesh、MuJoCo MJCF 和模型显示资源
└── rcm_teleop/               # 当前可运行的手柄/键盘 dry-run 输入包
docs/
└── PIPER_RCM_DEVELOPMENT_GUIDE.md
```

## 开发环境与依赖

已按以下环境整理教程：

- **OS**：Ubuntu 22.04
- **ROS 2**：Humble
- **Python**：3.10
- **可选仿真**：MuJoCo 3.x
- **后续规划**：MoveIt 2、Pinocchio、Piper SDK / SocketCAN

基础依赖建议：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-joy \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2
```

如需检查 MuJoCo MJCF：

```bash
python3 -m pip install mujoco
```

## 编译与安装

从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建当前可运行的遥操作输入包：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rcm_teleop --symlink-install
source install/setup.bash
ros2 pkg executables rcm_teleop
```

预期至少能看到：

```text
rcm_teleop keyboard_to_joy.py
rcm_teleop rcm_teleop.py
rcm_teleop joystick_test.py
```

## 使用教程

### 1. 手柄输入 dry-run

该教程用于验证手柄输入、模式切换和 dry-run 指令输出，不会控制真实机械臂。

终端 1：启动手柄输入链路。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rcm_teleop rcm_teleop.launch.py input_source:=gamepad
```

终端 2：观察话题。

```bash
source install/setup.bash
ros2 topic list
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
```

当前手柄映射：

| 输入 | 行为 | 输出影响 |
| --- | --- | --- |
| `X` | 进入 `GRAVITY_COMP` 标签模式 | `/rcm_mode` 发布 `GRAVITY_COMP`，当前只代表 dry-run 标签。 |
| `Y` | 在 `GRAVITY_COMP` 下请求标定 | `/rcm_mode` 短暂发布 `CALIBRATE_RCM`。 |
| `B` | 进入 `RCM_CONTROL` 标签模式 | 允许摇杆累计更新 `/rcm_cmd`。 |
| `A` | 指令清零 | `/rcm_cmd.x/y/z` 回到 0。 |
| 左摇杆 Y | pitch intent | 累计到 `/rcm_cmd.x`。 |
| 左摇杆 X | yaw intent | 累计到 `/rcm_cmd.y`。 |
| 右摇杆 Y | insertion intent | 累计到 `/rcm_cmd.z`。 |

默认参数来自 `src/rcm_teleop/launch/rcm_teleop.launch.py`：

```text
joy_node.deadzone = 0.1
joy_node.autorepeat_rate = 20.0 Hz
rcm_input_controller.deadzone = 0.15
pitch_step = 0.02
yaw_step = 0.02
insertion_step = 0.005
```

可以在 launch 命令中切换输入源或后续改 launch 参数。当前 `/rcm_cmd` 使用 `geometry_msgs/msg/Vector3` 表示操作者意图，不带时间戳、工具 ID、标定版本、deadman 或安全有效期，因此只能用于 dry-run 观察。

### 2. 键盘输入 dry-run

没有手柄时可以用键盘模拟 `/joy`，并复用同一个 `rcm_teleop.py` 解析节点。

终端 1：启动键盘输入链路。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rcm_teleop rcm_teleop.launch.py input_source:=keyboard
```

键盘映射：

| 按键 | 行为 |
| --- | --- |
| `W` / `S` | pitch 正/负方向 |
| `A` / `D` | yaw 负/正方向 |
| `Q` / `E` | insertion 正/负方向 |
| `G` | 进入 `GRAVITY_COMP` 标签模式 |
| `C` | 请求 `CALIBRATE_RCM` |
| `B` | 进入 `RCM_CONTROL` 标签模式 |
| `R` | 清零累计指令 |
| `Ctrl+C` | 退出键盘输入节点 |

终端 2：观察输出。

```bash
source install/setup.bash
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
```

### 3. 单独测试手柄数据

如果不确定手柄按钮编号是否和代码默认映射一致，先运行只读测试节点。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rcm_teleop joystick_test.launch.py
```

按下每个按钮和摇杆，观察终端打印的 `axes` 和 `buttons`。如果手柄型号不同，应先根据 `/joy` 输出调整 `src/rcm_teleop/src/rcm_teleop.py` 中的按钮索引，再进入 RCM dry-run。

### 4. MuJoCo 模型检查

仓库当前包含 MJCF 文件：

```text
src/agx_arm_description/models/xml/piper.xml
```

只检查模型能否被 MuJoCo 加载：

```bash
python3 - <<'PY'
import mujoco
model = mujoco.MjModel.from_xml_path('src/agx_arm_description/models/xml/piper.xml')
print('nq=', model.nq, 'nv=', model.nv, 'nu=', model.nu)
PY
```

打开 MuJoCo viewer：

```bash
python3 - <<'PY'
import mujoco
import mujoco.viewer
model = mujoco.MjModel.from_xml_path('src/agx_arm_description/models/xml/piper.xml')
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
PY
```

注意：当前仓库还没有 `agx_arm_sim` 包，也没有 ROS 2 与 MuJoCo 的 runtime bridge。上面的命令只用于模型资产检查，不代表已经完成仿真闭环或 Sim2Real。

### 5. RViz 模型显示（待补齐）

开发指南中规划的目标命令是：

```bash
ros2 launch agx_arm_description display_piper.launch.py
```

但当前仓库还不能把它当成稳定教程直接使用，原因如下：

- `src/agx_arm_description/launch/display_piper.launch.py` 当前为空文件。
- `src/agx_arm_description/CMakeLists.txt` 当前没有安装 `launch/`、`models/`、RViz 配置等资源。
- `src/agx_arm_description/models/urdf/piper/urdf/piper_description.urdf` 中的 mesh 路径仍写成 `package://agx_arm_description/agx_arm_urdf/...`，和当前 `models/meshes/...` 目录不一致。

补齐 RViz 教程前应先完成：

```text
1. 在 agx_arm_description/CMakeLists.txt 中安装 launch/ 和 models/。
2. 修正 URDF mesh 的 package:// 路径，使其指向当前安装后的 mesh 目录。
3. 实现 display_piper.launch.py，启动 robot_state_publisher、joint_state_publisher_gui 和 RViz。
4. 构建后运行 check_urdf，并确认 RViz 中 mesh、TF tree、joint1~joint6 都正常。
```

### 6. RCM 控制器和真机流程（待实现）

开发路线建议按以下顺序推进，不要跳过 dry-run、只读和低能量门控：

```text
1. 完成 agx_arm_description：模型安装、URDF 路径、RViz display launch、tool frame。
2. 保持 rcm_teleop 为 dry-run 输入包，不直接写硬件。
3. 新增 rcm_msgs：用 RCMCommand / RCMStatus 替代长期使用 Vector3。
4. 在 agx_arm_controller 中实现 RCM 数学、求解器、状态机和单元测试。
5. 新增 mock 或 sim 链路，先验证 RCM 残差、限位、奇异和 timeout。
6. 新增 hardware interface：先做 CAN/SDK 只读，再做唯一 command owner。
7. 通过 H0-H9 验收后，才允许低能量真机 RCM 控制实验。
```

真实硬件写控制前必须具备：

- 唯一 command owner。
- deadman / enable / watchdog / timestamp / valid-for 语义。
- joint name、joint order、方向、单位、零位、限位对表。
- RCM 点、工具轴、tool ID、calibration version 固化记录。
- solver infeasible、SDK/CAN error、feedback timeout 能进入 HOLD 或 FAULT。

## 常见问题

### `ros2 launch rcm_teleop rcm_teleop.launch.py` 找不到包

确认已经从工作区根目录构建并 source：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rcm_teleop --symlink-install
source install/setup.bash
ros2 pkg list | grep rcm_teleop
```

### `/joy` 没有数据

先确认系统能看到手柄设备：

```bash
ls /dev/input/js*
ros2 run joy joy_enumerate_devices
```

如果没有手柄，改用键盘模式：

```bash
ros2 launch rcm_teleop rcm_teleop.launch.py input_source:=keyboard
```

### `/rcm_cmd` 一直为 0

当前代码只有在模式为 `RCM_CONTROL` 时才累计 pitch、yaw、insertion。先按 `B` 进入 `RCM_CONTROL`，再移动摇杆或键盘方向键。

### `GRAVITY_COMP` 是否已经是真重力补偿

不是。当前 `GRAVITY_COMP` 是 `/rcm_mode` 上的字符串标签，用于输入流程 dry-run。真实重力补偿需要动力学模型、控制器、状态机、限幅、只读硬件验证和低能量验收。

## 文档对比后补齐的教程范围

本 README 已根据 `docs/PIPER_RCM_DEVELOPMENT_GUIDE.md` 和当前代码补齐以下入口级教程：

- 当前可运行的 `rcm_teleop` 手柄 dry-run 教程。
- 无手柄环境下的键盘 dry-run 教程。
- `/joy`、`/rcm_mode`、`/rcm_cmd` 的观察和按钮/轴映射说明。
- MuJoCo MJCF 资产加载与 viewer 检查教程。
- RViz display 当前缺口和补齐步骤，避免把空 launch 误写成可运行命令。
- RCM controller、重力补偿、真机写控制的待实现边界和安全前置条件。

## 审查清单

提交文档或代码前建议执行：

```bash
git diff --check -- README.md docs/PIPER_RCM_DEVELOPMENT_GUIDE.md
rg -n "CURRENT|PARTIAL|PLANNED|真机|写控制|GRAVITY_COMP|RCM_CONTROL" README.md docs/PIPER_RCM_DEVELOPMENT_GUIDE.md
```

如果本地有 ROS 2 Humble 环境，再执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rcm_teleop --symlink-install
source install/setup.bash
ros2 launch rcm_teleop rcm_teleop.launch.py input_source:=keyboard
```

审查重点：README 只能把已经有源码、launch 或模型资产支撑的内容标为 CURRENT；未实现的 RViz launch、RCM solver、重力补偿、MuJoCo bridge、Piper SDK/CAN 写控制必须继续标为 PARTIAL 或 PLANNED。

## 演示视频

- 暂未补充。建议后续按 `rcm_teleop` dry-run、RViz display、MuJoCo asset check、mock RCM controller、低能量真机验收分阶段录制。

## 参考项目与致谢

- [AgileX Arm URDF](https://github.com/agilexrobotics/agx_arm_urdf) - 松灵六轴机械臂 URDF 模型文件。
- [AgileX pyAgxArm SDK](https://github.com/agilexrobotics/pyAgxArm) - 松灵机械臂 Python SDK。
