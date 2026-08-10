# Piper 六轴机械臂 RCM 控制项目实施手册

- **状态**：中文重构版
- **日期**：2026-08-09
- **适用仓库**：`ros2_control_6_dof_arm_rcm_control`
- **改动范围**：本文只描述文档、接口、功能包搭建、RCM 原理和验收流程；不代表本文中的 `PLANNED` 代码已经在仓库中实现。
- **执行原则**：从上到下做。先模型和接口，再 RCM 数学，再 mock/sim，最后只读真机和低能量真机。

---

## 目录

- [0. 项目目标、架构和安全边界](#0-项目目标架构和安全边界)
- [1. 开始前：环境、仓库基线和当前可运行内容](#1-开始前环境仓库基线和当前可运行内容)
- [2. 功能包：`agx_arm_description`](#2-功能包agx_arm_description)
- [3. 功能包：`rcm_teleop`](#3-功能包rcm_teleop)
- [4. 功能包：`rcm_msgs`](#4-功能包rcm_msgs)
- [5. RCM 纯理论：远心点约束、工具轴线和求解器](#5-rcm-纯理论远心点约束工具轴线和求解器)
- [6. 功能包：`agx_arm_controller`](#6-功能包agx_arm_controller)
- [7. 功能包：`agx_arm_moveit_config`](#7-功能包agx_arm_moveit_config)
- [8. 功能包：`agx_arm_sim`](#8-功能包agx_arm_sim)
- [9. 功能包：`agx_arm_hw_interface`](#9-功能包agx_arm_hw_interface)
- [10. 功能包：`agx_arm_bringup`](#10-功能包agx_arm_bringup)
- [11. 从 mock 到真机的逐步验收](#11-从-mock-到真机的逐步验收)
- [12. 故障处理、接口契约和文档维护](#12-故障处理接口契约和文档维护)
- [附录 A：命令速查](#附录-a命令速查)
- [附录 B：功能包状态表](#附录-b功能包状态表)
- [附录 C：术语和禁止事项](#附录-c术语和禁止事项)

---

## 0. 项目目标、架构和安全边界

本项目的目标是把 Piper 六轴机械臂接入 ROS 2 工作区，逐步形成一条可验证、可回放、可停机的 RCM 控制链路。文档按真实开发顺序组织：先把机器人模型和输入链路做清楚，再定义消息接口，再理解 RCM 约束，再写 C++ 控制器，然后进入仿真、只读硬件、低能量真机和验收。

### 0.1 最终系统链路

推荐最终链路如下：

```text
操作者 / 手柄 / 上层任务 / 视觉目标
  -> RCMCommand 或上层 intent
  -> SafetySupervisor / 状态机 / watchdog
  -> RCMController C++ 控制器
  -> ros2_control controller 或 command owner
  -> Piper SDK / CAN hardware adapter
  -> 真实 Piper 机械臂
  -> joint feedback / diagnostics / rosbag evidence
```

这条链路的核心是“输入意图”和“硬件写命令”必须分开：

- `rcm_teleop` 只表达操作者意图，不接触 SDK/CAN 写接口。
- `rcm_msgs` 固化跨包接口，避免用普通 `Vector3` 长期承载安全语义。
- `agx_arm_controller` 负责 RCM 几何、雅可比、求解器、状态机和状态发布。
- `agx_arm_hw_interface` 是唯一接触 Piper SDK/CAN 的硬件边界。
- `agx_arm_bringup` 只组合 launch 和参数，不写控制算法。
- MoveIt 2、MuJoCo、RViz、rosbag 都是辅助工具，不能成为真机 command owner。

### 0.2 当前仓库事实和规划包

当前仓库已经存在以下 ROS 2 package：

```text
src/
├── rcm_teleop/
├── agx_arm_description/
├── agx_arm_controller/
└── agx_arm_bringup/
```

后续建议新增或完善：

```text
src/
├── rcm_msgs/                  # PLANNED：RCM 强类型消息、状态、reason code
├── agx_arm_moveit_config/     # PLANNED：MoveIt 2 配置和 fake execution
├── agx_arm_sim/               # PLANNED：MuJoCo / replay / sim namespace
└── agx_arm_hw_interface/      # PLANNED：Piper SDK/CAN 只读适配和 command owner
scripts/                       # PLANNED：CAN、SDK 只读检查、证据收集脚本
```

本文使用三个状态标签：

| 标签 | 含义 | 写入条件 |
| --- | --- | --- |
| `CURRENT` | 当前仓库中已经存在，并且可以静态检查或运行 | 有路径、命令、输出或测试证据 |
| `PLANNED` | 建议实现，但当前不能当作已完成能力 | 有接口草案、目录建议、验收条件 |
| `RESEARCH` | 理论、算法、仿真或安全研究内容 | 有公式、风险说明、实验要求 |

不能因为目录存在就声明功能可用；也不能因为文档里有代码片段就认为仓库已经实现。所有 `PLANNED` 内容都需要后续提交源码、配置、launch、测试和验收证据。

### 0.3 技术路线：ROS 2 + C++ 核心 + Python 工具

本项目不建议做成纯 Python 单体程序，也不建议做成纯 C++ 单体程序。推荐路线是：

- ROS 2 负责通信、参数、launch、TF、RViz、rosbag、MoveIt 2、diagnostics、ros2_control 接入。
- C++ 负责实时性和安全风险高的链路，例如 RCM 控制器、求解器、状态机、hardware interface、command owner、高频 sim bridge。
- Python 负责输入 dry-run、标定脚本、日志分析、只读 probe、运维脚本和离线数学验证。

语言选择建议：

| 模块 | 推荐技术 | 原因 |
| --- | --- | --- |
| `rcm_teleop` 当前 dry-run | Python / `rclpy` | 人机输入低频，便于调试，不直接写硬件 |
| `rcm_msgs` | ROS 2 IDL / `ament_cmake` | C++ 和 Python 都可生成接口 |
| `agx_arm_controller` | C++ / `rclcpp` / Eigen | 控制周期、矩阵计算、异常路径更可控 |
| `agx_arm_hw_interface` | C++ 优先 | SDK/CAN、watchdog、command owner 必须 fail-closed |
| `agx_arm_bringup` | Python launch | 只组合节点和参数，不做实时控制 |
| `agx_arm_moveit_config` | 配置为主，C++ 插件按需 | MoveIt 规划不是 RCM 闭环本体 |
| `agx_arm_sim` | C++/Python 分工 | 高频同步用 C++，离线 replay/画图用 Python |
| `scripts/` | bash/Python | 一次性只读工具，不承担持续控制 |

判断准则：需要稳定周期、低延迟、硬件写控制、watchdog、限幅和状态机的部分，优先用 C++；需要快速验证、离线分析、一次性检查的部分，保留 Python 更合适。

### 0.4 控制权和安全边界

真实机械臂写控制必须坚持 single command owner：

```text
同一时刻只能有一个节点拥有真实硬件写命令权。
```

禁止多个节点同时 import SDK 或同时发布真实写命令。允许多个节点产生目标、建议、规划或仿真状态，但最终写硬件的节点必须唯一，并且受状态机、watchdog、timestamp、operator enable、joint limit 和 fault latch 管控。

写控制前必须满足：

- joint name、joint order、方向、单位、零位、限位已经核对。
- RCM 工具轴线、有效直杆段、tool ID、calibration version 已冻结。
- `RCMCommand` 或等价命令带 timestamp、有效期、enable、speed scale、tool/calibration 字段。
- command owner 默认关闭，必须显式授权才允许写控制。
- SDK/CAN error、feedback timeout、watchdog timeout、solver infeasible 都能进入 `HOLD` 或 `FAULT`。

---

## 1. 开始前：环境、仓库基线和当前可运行内容

本章只确认当前仓库事实，不启动真机，不发送硬件命令。后续所有功能包都应在本章能稳定执行的基础上继续做。

### 1.1 环境准备

建议以 ROS 2 Humble 为基线：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果构建失败，先不要新增功能包。按以下顺序排查：

1. `source /opt/ros/humble/setup.bash` 是否执行。
2. `colcon` 是否安装。
3. `src/*/package.xml` 中依赖是否缺失。
4. 当前工作区是否有未提交的大规模移动、删除或 mesh 路径变化。
5. `build/`、`install/`、`log/` 是否来自旧环境；必要时清理这些生成目录后重建。

### 1.2 当前包基线检查

列出当前 package：

```bash
find src -maxdepth 2 -name package.xml -print
```

应能看到：

```text
src/rcm_teleop/package.xml
src/agx_arm_description/package.xml
src/agx_arm_controller/package.xml
src/agx_arm_bringup/package.xml
```

单独构建当前可运行输入包：

```bash
colcon build --packages-select rcm_teleop --symlink-install
source install/setup.bash
ros2 pkg executables rcm_teleop
```

### 1.3 当前 `rcm_teleop` dry-run 验收

当前仓库真正可优先验证的是手柄 dry-run 链路：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rcm_teleop --symlink-install
source install/setup.bash
ros2 launch rcm_teleop rcm_teleop.launch.py
```

另开终端观察：

```bash
source install/setup.bash
ros2 topic list
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
```

通过标准：

- launch 能启动，不报找不到 package 或 executable。
- `/joy` 有手柄输入。
- `/rcm_mode` 能随按钮变化。
- `/rcm_cmd` 能随摇杆或按键产生 dry-run intent。

这一步只证明输入链路可用，不证明 RCM 控制、MoveIt、MuJoCo、SDK、CAN 或真机控制可用。

### 1.4 建议的实际推进顺序

从这里开始按顺序做，不要跳到真机写控制：

```text
1. 补齐 agx_arm_description：模型、mesh、TF、显示 launch
2. 整理 rcm_teleop：保留 dry-run，明确 deadman、mode、intent
3. 新增 rcm_msgs：冻结 RCMCommand / RCMStatus / reason code
4. 学清 RCM 理论：工具轴线、RCM 点、残差、雅可比、求解器
5. 实现 agx_arm_controller：先 C++ 数学库，再 ROS wrapper，再状态机
6. 新增 agx_arm_moveit_config：只做 fake plan / approach / retract
7. 新增 agx_arm_sim：只读 replay / MuJoCo / sim namespace
8. 新增 agx_arm_hw_interface：先 CAN/SDK 只读，再 command owner
9. 完善 agx_arm_bringup：按 display、dry-run、mock、sim、readonly、live_low_energy 分开 launch
10. 按 H0-H9 验收进入低能量真机；启用重力补偿前必须额外通过 H8.5
```

这个顺序的原因很简单：RCM 控制依赖几何，几何依赖 description；控制器依赖强类型消息；真机写控制依赖只读反馈和 command owner；所有 live 操作都依赖前面的证据。

---

## 2. 功能包：`agx_arm_description`

### 2.1 作用和边界

`agx_arm_description` 是模型包，负责 URDF/Xacro、mesh、joint、link、TF、工具 frame、RViz 显示资源和后续 MoveIt/MuJoCo 的几何基础。

它应该放：

| 内容 | 说明 |
| --- | --- |
| URDF/Xacro | 六轴机械臂 link、joint、limit、inertial、visual、collision |
| mesh | STL/DAE 或其他模型文件，路径必须可安装和可解析 |
| tool frames | `tool_mount`、`tool_axis`、`tool_tip`、`tool_collision` |
| display launch | 只启动模型显示、TF、RViz，不接硬件 |
| RViz 配置 | 模型显示、TF tree、joint state 可视化 |
| 模型测试 | URDF 解析、mesh 路径、joint name、关键 frame 检查 |

它不应该放：

- SDK/CAN 代码。
- RCM solver。
- rcm_teleop 输入解析。
- command owner。
- 真机写控制 launch。

### 2.2 当前状态

当前仓库已经有 `src/agx_arm_description`，但需要继续确认资源安装、display launch、RViz 配置和工具 frame 是否完整。description 包必须先稳定，因为后续 RCM 理论和 controller 都引用同一套 joint name、frame name 和 tool axis。

检查当前文件：

```bash
find src/agx_arm_description -maxdepth 4 -type f | sort
```

重点看：

- `package.xml` 是否声明模型显示运行依赖。
- `CMakeLists.txt` 是否安装 URDF、mesh、launch、rviz/config。
- URDF 中 mesh 是否使用 `package://` 路径。
- 是否存在绝对路径，例如 `/home/.../mesh.stl`。
- `joint1` 到 `joint6` 名称、顺序、方向是否和 SDK/厂商资料可对应。

### 2.3 一步步补齐模型显示

第 1 步：补资源安装规则。

```cmake
# PLANNED：示例安装规则。实际目录名以仓库为准。
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
```

第 2 步：新增或完善 `display_piper.launch.py`。

该 launch 只允许做模型显示：

```python
# PLANNED：模型显示 launch 结构示意，不连接 SDK/CAN。
robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{"robot_description": robot_description}],
)

joint_state_publisher_gui = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
)
```

第 3 步：构建并检查安装结果。

```bash
colcon build --packages-select agx_arm_description --symlink-install
source install/setup.bash
ros2 pkg prefix agx_arm_description
find install/agx_arm_description/share/agx_arm_description -maxdepth 3 -type f | sort
```

第 4 步：检查 URDF。

```bash
check_urdf install/agx_arm_description/share/agx_arm_description/urdf/piper/urdf/piper_description.urdf
```

如果 URDF 路径不同，以实际 `find install/... -name '*.urdf'` 的输出为准。

第 5 步：启动显示。

```bash
ros2 launch agx_arm_description display_piper.launch.py
```

RViz 中应检查：

- 模型不缺 mesh。
- link 尺寸和方向看起来合理。
- `joint1` 到 `joint6` 移动方向可解释。
- TF tree 没有断链。
- tool frame 不和 collision mesh 混用。

### 2.4 工具 frame 怎么设计

RCM 不是控制“末端点到某个点”，而是控制“工具功能轴线穿过 RCM 点”。因此 description 包必须提供清晰的工具几何：

```text
base_link
 └── ... joint1 ~ joint6 ...
      └── flange / link6
           └── tool_mount
                ├── tool_axis       # +Z 表示工具功能轴线
                ├── tool_tip        # 工具工作尖端
                └── tool_collision  # 工具外形/碰撞模型
```

原则：

- `tool_axis` 表示功能轴线，不一定等于外形几何中心线。
- `tool_collision` 表示碰撞外形，可以复杂，但不能替代功能轴线。
- `tool_tip` 是工作点，用于插入深度或末端可视化。
- 有效直杆段要在 tool manifest 中定义，例如 `lambda_min_m`、`lambda_max_m`。
- 如果工具是弯曲或柔性结构，不要硬套单一直线 RCM，需要另做模型。

### 2.5 验收标准和禁止事项

通过标准：

- package 能单独构建。
- URDF 能被 `check_urdf` 解析。
- 安装目录中能找到 URDF、mesh、launch、rviz/config。
- RViz 能显示模型和 TF。
- joint name/order/limit 有对表记录。
- tool frame 的设计和 RCM 理论一致。

禁止事项：

- 不要在 controller 中用符号翻转临时修正模型方向错误。
- 不要把 SDK 的 joint order 写死在 description 之外又不留对表。
- 不要让 MoveIt、MuJoCo、controller 分别维护不同 joint name。
- 不要用绝对路径引用 mesh。

---

## 3. 功能包：`rcm_teleop`

### 3.1 作用和边界

`rcm_teleop` 负责把手柄输入转成操作者意图。当前它是 dry-run 输入包，不是真机控制包。

它应该放：

| 内容 | 说明 |
| --- | --- |
| 手柄输入解析 | `/joy` axes/buttons 到内部 intent |
| mode 切换 | 例如 pivot、insert、roll、reset |
| deadman / enable intent | 表示操作者是否持续允许发送意图 |
| dry-run topic | 当前 `/rcm_mode`、`/rcm_cmd`，未来 typed intent |
| 参数 | deadzone、step、speed scale、button mapping |
| 测试 | 模拟 Joy 消息验证 mode、deadzone、reset、timeout |

它不应该放：

- SDK/CAN import。
- joint command。
- MoveIt Execute。
- RCM solver。
- 真机 command owner。

### 3.2 当前 dry-run 怎么跑

```bash
colcon build --packages-select rcm_teleop --symlink-install
source install/setup.bash
ros2 launch rcm_teleop rcm_teleop.launch.py
```

观察 topic：

```bash
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
```

当前 `/rcm_cmd` 使用普通消息表达累计 intent。它可以验证输入链路，但不能作为 live arm 命令接口长期使用。

### 3.3 一步步整理输入逻辑

第 1 步：记录手柄映射。

```text
button A: reset 或归零 dry-run intent
button B/X/Y: mode 切换
axes: pivot / insertion / roll intent
deadman: 后续必须指定一个持续按压按钮
```

实际按钮编号必须通过 `/joy` 输出确认，不要直接照搬别的手柄。

第 2 步：参数化 deadzone 和步长。

建议参数：

```yaml
rcm_teleop:
  ros__parameters:
    deadzone: 0.08
    translational_step_m: 0.001
    angular_step_rad: 0.01
    max_speed_scale: 0.2
    command_valid_for_sec: 0.1
```

第 3 步：把输入解析和消息发布分开。

推荐内部结构：

```text
Joy callback
  -> parse axes/buttons
  -> GamepadIntent 内部结构
  -> apply deadzone / mode / reset
  -> publish dry-run message 或 typed intent
```

这样后续迁移到 `rcm_msgs` 时，不需要把整个节点重写。

第 4 步：增加命令有效期。

手柄断开、节点卡顿或 topic 过期时，后续 controller 必须能拒绝旧命令。typed intent 中应包含 timestamp 和 `valid_for_sec`。

第 5 步：保留 dry-run 输出一段时间。

迁移到 `rcm_msgs` 后，可以短期保留旧 `/rcm_cmd` 作为观察输出，但不能让 live command owner 订阅旧 dry-run topic。

### 3.4 未来 typed intent 示例

```python
# PLANNED：字段组织示意。这里只用于文档说明，不表示当前已经实现。
cmd.header.stamp = node.get_clock().now().to_msg()
cmd.header.frame_id = "base_link"
cmd.enable = deadman_pressed
cmd.mode = RCM_MODE_PIVOT
cmd.max_speed_scale = configured_speed_scale
cmd.valid_for_sec = 0.1
cmd.tool_id = current_tool_id
cmd.calibration_version = current_calibration_version
```

注意：`enable=true` 只是操作者意图，不等于硬件允许运动。真正是否输出写命令，由 `agx_arm_controller` 和 `agx_arm_hw_interface` 的状态机判断。

### 3.5 Python 和 C++ 取舍

当前 `rcm_teleop` 用 Python 是合理的，因为手柄输入低频、便于调试、不直接写硬件。真正影响机械臂流畅性的链路是 controller、solver、watchdog、hardware interface 和 command owner，这些应优先用 C++。

只有在以下情况才考虑把 rcm_teleop 核心迁移到 C++：

- 同时融合多个高频输入源。
- 输入滤波和状态机复杂到 Python 难以稳定维护。
- 需要和 C++ controller 在同一进程内做低延迟通信。
- 安全审查要求所有 live-adjacent 输入链路使用 C++。

### 3.6 验收标准

- 手柄静止时 `/rcm_cmd` 不漂移。
- mode 切换可复查，日志中能看到状态变化。
- deadman 未按下时，未来 typed intent 必须表达 `enable=false`。
- 手柄断开后，controller 不会继续使用旧命令。
- 单元测试覆盖 deadzone、reset、mode、timeout。

---

## 4. 功能包：`rcm_msgs`

### 4.1 作用和边界

`rcm_msgs` 是建议新增的强类型接口包。它只放 `.msg`、`.srv`、`.action` 和接口枚举说明，不放节点实现、算法、SDK 或 launch。

为什么必须有这个包：RCM 命令不是普通三维向量。它需要包含 frame、timestamp、有效期、工具 ID、标定版本、enable、mode、速度缩放、状态码和 reason code。长期用 `Vector3` 会让安全语义散落在代码注释里。

### 4.2 推荐目录

```text
src/rcm_msgs/
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── RCMCommand.msg
│   ├── RCMStatus.msg
│   └── RCMDiagnostics.msg
├── srv/
│   └── SetRCMMode.srv
└── action/
    └── CalibrateToolAxis.action   # 可选，后续标定流程再加
```

### 4.3 一步步创建接口包

第 1 步：创建包。

```bash
cd src
ros2 pkg create rcm_msgs --build-type ament_cmake
mkdir -p rcm_msgs/msg rcm_msgs/srv rcm_msgs/action
```

第 2 步：在 `package.xml` 中声明接口依赖。

```xml
<!-- PLANNED：接口包依赖示意。 -->
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
```

第 3 步：在 `CMakeLists.txt` 中生成接口。

```cmake
# PLANNED：接口生成示意。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RCMCommand.msg"
  "msg/RCMStatus.msg"
  "msg/RCMDiagnostics.msg"
  "srv/SetRCMMode.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

第 4 步：构建并检查接口。

```bash
colcon build --packages-select rcm_msgs --symlink-install
source install/setup.bash
ros2 interface list | grep rcm_msgs
ros2 interface show rcm_msgs/msg/RCMCommand
```

### 4.4 推荐 `RCMCommand`

```text
# PLANNED: rcm_msgs/msg/RCMCommand.msg
std_msgs/Header header

# RCM 固定点，推荐先约定在 base_link 下表达。
geometry_msgs/Point pivot_base

# 工具轴目标方向或当前 shaft direction intent，必须说明是否单位向量。
geometry_msgs/Vector3 shaft_direction_base

# RCM 兼容任务速度。
float64 insertion_velocity_mps
float64 roll_velocity_radps
float64 pivot_velocity_scale

# 安全字段。
float64 max_speed_scale
float64 valid_for_sec
bool enable
uint8 mode

# 工具和标定一致性。
string tool_id
string calibration_version
```

字段说明：

- `header.stamp` 是命令生成时间，controller 必须检查是否过期。
- `header.frame_id` 第一版建议固定为 `base_link`，后续如支持其他 frame，必须显式 TF 转换和时间戳检查。
- `pivot_base` 是 RCM 点，不是 TCP 目标。
- `shaft_direction_base` 必须说明是否单位向量；controller 应拒绝 NaN 和零向量。
- `max_speed_scale` 只能缩小速度，不允许绕过硬件限速。
- `tool_id` 和 `calibration_version` 必须和当前加载 tool manifest 一致。

### 4.5 推荐 `RCMStatus`

```text
# PLANNED: rcm_msgs/msg/RCMStatus.msg
std_msgs/Header header

float64 rcm_distance_m
float64 axis_error_norm
float64 insertion_m
float64 sigma_min
float64 condition_number
float64 task_scale

uint8 controller_state
uint8 solver_status
uint32 active_constraints

bool rcm_within_tolerance
bool command_owner_active
bool feedback_fresh
bool command_fresh

string reason_code
string tool_id
string calibration_version
```

状态消息必须能回答三个问题：现在是否允许控制、为什么停、RCM 误差是多少。没有这些字段，调试时只能靠日志猜。

### 4.6 迁移顺序

1. 先新增 `rcm_msgs`，只构建接口，不改控制逻辑。
2. 让 `rcm_teleop` 增加对 `rcm_msgs` 的依赖，新增 typed intent topic。
3. 让 `agx_arm_controller` 订阅 typed command，旧 dry-run topic 只做调试。
4. 让 `agx_arm_hw_interface` 只接收 controller/command owner 授权后的命令，不订阅 rcm_teleop。
5. 文档和测试中固定 message 字段、单位、frame、timeout、reason code。

---

## 5. RCM 纯理论：远心点约束、工具轴线和求解器

本章只讲理论、几何关系和求解思路，不绑定某个 ROS 节点。它的目标是让读者先知道 RCM 为什么不是普通末端位姿控制、工具轴线应该怎么定义、雅可比从哪里来、求解器为什么不能简单裁剪关节速度。理解本章后，再进入第 6 章的 `agx_arm_controller` C++ 实现。

本章建议按下面顺序读：

```text
物理约束是什么
  -> 坐标系和符号怎么定义
  -> 点到线残差怎么计算
  -> 工具轴线和有效直杆段怎么标定
  -> 六轴机械臂自由度够不够
  -> 关节速度如何影响 RCM 残差
  -> DLS/QP/SNS 怎么求 q_dot
  -> 求解失败时怎么停
```

### 5.1 RCM 到底约束什么

RCM（Remote Center of Motion，远程运动中心）要求工具的功能轴线始终穿过空间中的固定点。这个固定点通常是套管、孔、穿刺入口或机械导向中心。

工具可以：

- 绕 RCM 点摆动。
- 沿工具轴线插入或退出。
- 绕工具轴线滚转。

工具不应该：

- 在入口点产生横向拉扯。
- 把普通 TCP pose goal 当作 RCM 约束。
- 在 RCM 残差增大时继续插入。

一个更直观的理解是：RCM 点像一个固定小孔，工具像一根穿过小孔的直杆。机械臂末端可以在小孔外侧改变工具方向，也可以让工具沿杆方向进出，但不能让工具在小孔处横向扫动。横向扫动会把入口当成支点强行撬动，这正是 RCM 要避免的风险。

RCM 和普通控制目标的区别：

| 控制目标 | 控制对象 | 能否天然保证 RCM | 说明 |
| --- | --- | --- | --- |
| TCP 位置控制 | 工具尖端点 | 不能 | TCP 到位时，工具轴线可能没有穿过入口点 |
| TCP 位姿控制 | 工具尖端 6D pose | 不能 | 姿态正确也不等于入口横向误差为 0 |
| 关节轨迹跟踪 | 六个关节角 | 不能 | 轨迹可执行不等于工具轴线过 RCM 点 |
| RCM 控制 | 工具轴线和固定点 | 可以 | 直接约束入口点到工具轴线的横向距离 |

因此，RCM 控制的第一原则是：**控制对象不是某一个末端点，而是一条工具功能轴线和一个固定空间点之间的几何关系**。

实际项目中要先回答四个问题：

1. 哪个点是 RCM 点？它在 `base_link`、相机坐标系还是外部定位坐标系下？
2. 哪条线是工具功能轴线？它是针体中心线、镜管中心线、视觉光轴，还是夹具定义的偏置轴线？
3. RCM 点是否落在工具真实有效直杆段内，而不只是落在数学无限延长线上？
4. 当约束不满足时，系统应该降速、保持、撤出，还是进入 fault？

如果这四个问题没有写清楚，后续控制器即使能动，也不能证明是在做 RCM 控制。

### 5.2 坐标系、变量和符号约定

RCM 理论里最容易出错的不是公式，而是坐标系混用。建议先统一以下符号：

| 符号 | 含义 | 推荐坐标系 |
| --- | --- | --- |
| `B` | 机器人基座坐标系，例如 `base_link` | 主控制坐标系 |
| `F` | 法兰或第六轴末端坐标系，例如 `link6` / `flange` | 工具安装基准 |
| `T` | 工具坐标系，例如 `tool_axis`、`tool_tip` | 工具几何定义 |
| `q` | 六个关节角 | rad |
| `q_dot` | 六个关节速度 | rad/s |
| `p_R_B` | RCM 点在基座坐标系下的位置 | m |
| `p0_F` | 工具轴线上一点，在法兰坐标系下定义 | m |
| `u_F` | 工具轴线方向，在法兰坐标系下定义 | 单位向量 |
| `p0_B` | 工具轴线上一点，变换到基座坐标系 | m |
| `u_B` | 工具轴线方向，变换到基座坐标系 | 单位向量 |
| `e_RCM` | RCM 横向误差向量 | m |
| `d_RCM` | RCM 横向误差距离 | m |

推荐所有控制计算先统一到 `base_link` 下完成。比如相机给出一个入口点 `p_R_C`，也必须先通过带时间戳的 TF 或外参变换成 `p_R_B`，再进入 RCM 公式。不要在一个公式里同时混用 `base_link` 点、法兰方向和相机坐标下的 pivot。

从法兰到基座的正运动学记为：

```text
T_BF(q) = [ R_BF(q)   p_F_B(q) ]
          [   0            1   ]
```

其中：

- `R_BF(q)` 是法兰相对基座的旋转矩阵。
- `p_F_B(q)` 是法兰原点在基座坐标系下的位置。

工具轴线从法兰坐标系变换到基座坐标系：

```text
p0_B(q) = R_BF(q) * p0_F + p_F_B(q)
u_B(q)  = R_BF(q) * u_F
```

注意 `u_B` 是方向向量，只受旋转影响，不应该加平移。实现中如果把方向向量当成点做齐次变换，会引入错误平移分量。

### 5.3 点到线残差

工具轴线由一点 `p0` 和单位方向 `u` 表示：

```text
line(lambda) = p0 + lambda * u
||u|| = 1
```

RCM 点 `p_R` 到该直线的横向误差为：

```text
e = (I - u u^T) (p_R - p0)
d_RCM = ||e||
```

`I - u u^T` 是投影矩阵，它去掉沿工具轴方向的分量，只保留垂直于工具轴的误差。工具沿轴向插入时，轴向分量可以变化；真正要控制的是横向误差 `e`。

也可以用叉乘形式：

```text
c = u x (p_R - p0)
```

但 `c` 虽然有三个分量，只有两个独立约束。实现时必须做秩判断，不能把三维叉乘误差当成满秩 3D 约束直接求逆。

点到线残差的几何分解可以写得更直观：

```text
r       = p_R - p0
s       = u^T r                    # RCM 点沿工具轴方向的投影长度
p_close = p0 + s * u                # 工具轴线上离 RCM 点最近的点
e       = p_R - p_close             # RCM 横向残差
d_RCM   = ||e||
```

其中 `s` 是 RCM 点投影到工具轴线上的轴向位置。这个值本身不是横向误差，但它可以用来判断 RCM 点是否落在有效直杆段内：

```text
lambda_min <= s <= lambda_max
```

如果 `d_RCM` 很小但 `s` 不在有效段内，说明 RCM 点落在工具轴线的无限延长线上，而不是落在真实工具直杆段上。这种情况不能判定为安全满足 RCM。

数值例子：假设工具轴线为 z 轴：

```text
p0 = [0.0, 0.0, 0.0]
u  = [0.0, 0.0, 1.0]
```

若 RCM 点为：

```text
p_R = [0.003, -0.004, 0.120]
```

则轴向投影：

```text
s = u^T (p_R - p0) = 0.120
```

横向残差：

```text
e = [0.003, -0.004, 0.0]
d_RCM = sqrt(0.003^2 + 0.004^2) = 0.005 m
```

这个例子说明：RCM 点在工具轴方向上的深度是 120 mm，但真正危险的是横向偏差 5 mm。控制器应该优先让 `[0.003, -0.004]` 收敛到 0，而不是把 120 mm 当成误差消掉。

实际计算时必须检查：

- `u` 是否为单位向量，容差例如 `abs(||u|| - 1) < 1e-6` 或项目设定阈值。
- `p0`、`u`、`p_R` 是否包含 NaN/Inf。
- `d_RCM` 是否超过进入阈值或 fault 阈值。
- `s` 是否在有效直杆段范围内。
- `e` 的方向是否和有限差分或可视化结果一致。

推荐把 `d_RCM` 至少分成三个区间：

| 区间 | 含义 | 控制策略 |
| --- | --- | --- |
| `d_RCM <= entry_threshold` | 可进入或保持 RCM 控制 | 允许低速 active |
| `entry_threshold < d_RCM <= fault_threshold` | 偏差偏大但未进入硬故障 | 降速、纠偏、禁止插入 |
| `d_RCM > fault_threshold` | 入口横向误差不可接受 | HOLD/FAULT，不继续执行任务 |

阈值不能凭感觉写死。应根据工具直径、入口机构间隙、传感器误差、机械臂重复定位精度、标定 RMS 和现场风险共同确定。

### 5.4 工具几何和标定

在法兰坐标系 `F` 中定义工具轴线：

```text
p0_F = 工具轴线上一个已知点
u_F  = 工具轴方向单位向量
```

变换到基座坐标系 `B`：

```text
p0_B = R_BF * p0_F + p_F_B
u_B  = R_BF * u_F
```

tool manifest 至少应记录：

```yaml
tool_id: piper_tool_v1
calibration_version: 2026-08-09_axis_a
base_frame: base_link
flange_frame: link6
axis_frame: tool_axis
tip_frame: tool_tip
axis_point_flange_m: [0.0, 0.0, 0.12]
axis_direction_flange: [0.0, 0.0, 1.0]
lambda_min_m: 0.02
lambda_max_m: 0.18
fit_rms_m: REPLACE_WITH_MEASURED_VALUE
source_bag: REPLACE_WITH_ROSBAG_OR_LOG
operator: REPLACE_WITH_NAME
```

`lambda_min_m` 和 `lambda_max_m` 用来表示真实有效直杆段。RCM 点只落在数学无限延长线上不够，还必须落在真实工具有效段范围内。

工具几何建议拆成三个对象，不要混在一个 frame 里：

| 对象 | 作用 | 例子 |
| --- | --- | --- |
| `tool_axis` | RCM 约束使用的功能轴线 | 针体中心线、镜管轴线、视觉光轴 |
| `tool_tip` | 插入深度、工作点或可视化终点 | 针尖、镜头端点、探针端点 |
| `tool_collision` | MoveIt/仿真碰撞外形 | 工具外壳、夹具、线缆保护件 |

不规则工具也可以使用单一直线 RCM，只要真正受入口约束的是一条清晰的功能直线。例如工具外壳偏心、夹具形状复杂，但穿过入口的是一根直针或直镜管，就应把 `tool_axis` 定义为这根直线，而不是用 mesh 几何中心替代。

不适合单一直线 RCM 的情况：

- 入口接触的是弯曲段。
- 工具是柔性导管，轴线随受力弯曲。
- 工具有效约束不是线，而是面、锥、槽或多点机构。
- 工具在夹具中有明显松动，法兰到工具轴线不是固定变换。

标定工具轴线的常见思路：

1. 固定工具，使工具功能轴尽量清晰可测。
2. 采集多个姿态下工具上两个或多个可测点。
3. 把测得点转换到法兰坐标系或基座坐标系。
4. 用最小二乘拟合轴线方向 `u_F` 和轴上一点 `p0_F`。
5. 计算拟合 RMS 和最大残差。
6. 写入 tool manifest，并生成 calibration version。
7. 用不同姿态复测，确认姿态变化不会引入系统偏差。

轴线拟合的最小二乘直觉：给定一组工具轴上的测量点 `x_i`，先求中心点：

```text
x_bar = mean(x_i)
```

再对点云协方差做主方向分析，最大特征值对应的特征向量可作为轴线方向 `u`。拟合残差为每个点到拟合轴线的距离：

```text
d_i = ||(I - u u^T) (x_i - x_bar)||
fit_rms = sqrt(mean(d_i^2))
```

工程上不一定必须在文档里实现这段算法，但必须保存：采样点来源、坐标系、拟合结果、RMS、最大误差、操作者、日期和原始日志。没有标定证据的工具轴线不能进入 live RCM。

标定验收建议：

| 项 | 要求 |
| --- | --- |
| `axis_direction_flange` | 必须归一化，方向定义清楚 |
| `axis_point_flange_m` | 必须在工具轴线上，单位为 m |
| `lambda_min_m/max_m` | 覆盖真实有效直杆段，不使用无限线替代 |
| `fit_rms_m` | 小于项目 RCM 偏差预算的一部分 |
| 复测姿态 | 至少覆盖中性位、偏转位、接近工作区边界 |
| 标定版本 | 写入 command 和 status，防止工具换了但控制器不知道 |

### 5.5 六轴机械臂的自由度预算

单一直杆 RCM 中，轴线穿过固定点提供两个独立横向约束。完整工具任务通常还包括两个摆动自由度、一个插入自由度、一个滚转自由度。

```text
2 个 RCM 横向约束 + 4 个工具任务自由度 = 6
```

对六轴 Piper 来说，这通常意味着没有额外冗余可以随意叠加肘部姿态、强避障、任意 TCP pose 等硬约束。工程上必须明确优先级：

1. 硬件安全和停止。
2. RCM 横向误差。
3. 关节限位和奇异性。
4. 插入、摆动、滚转任务。
5. 姿态偏好、平滑性、避障等二级目标。

当目标不可同时满足时，应缩放、hold、replan 或 fault，而不是继续输出不可解释命令。

为什么 RCM 横向约束是两个自由度：一条三维直线由方向 `u` 和轴上一点 `p0` 决定。一个空间点到这条线的误差有三维写法，但沿工具轴方向的分量不影响是否穿过入口。真正需要压住的是垂直于工具轴的两个方向，所以是两个独立约束。

六轴机械臂常见任务分解：

| 任务 | 自由度 | 说明 |
| --- | ---: | --- |
| RCM 横向约束 | 2 | 保证工具轴线穿过入口 |
| pivot 摆动 | 2 | 改变工具方向，相当于绕 RCM 点摆动 |
| insertion 插入 | 1 | 沿工具轴线进出 |
| roll 滚转 | 1 | 绕工具轴线自转 |
| 合计 | 6 | 刚好用满六轴能力 |

这意味着六轴机械臂在严格 RCM 下通常没有冗余去同时满足很多额外目标。比如“工具轴过 RCM 点”“TCP 到某个任意 3D 点”“姿态完全对齐某个 3D 姿态”“肘部远离某个区域”“远离所有关节限位”这些要求同时出现时，系统可能不可行。

可行性判断应在文档、UI 和代码里显式暴露：

- 若 RCM 约束和插入任务冲突，优先保持 RCM，停止插入。
- 若接近关节限位，降低 task scale 或要求重新规划 approach。
- 若接近奇异，降低摆动速度或退出到安全姿态。
- 若碰撞风险和 RCM 任务冲突，不要强行求解，应 hold/replan。

不要把不可行问题伪装成“调大增益就能解决”。增益只能影响收敛速度，不能创造额外自由度。

### 5.6 点雅可比和方向雅可比

法兰速度：

```text
v_F     = J_v(q) * q_dot
omega_F = J_w(q) * q_dot
```

工具轴线上点 `p = p_F + R_BF * r_F` 的速度：

```text
v_p = v_F + omega_F x (R_BF * r_F)
J_p = J_v - skew(R_BF * r_F) * J_w
```

工具轴方向 `u = R_BF * u_F` 的变化：

```text
u_dot = omega_F x u
J_u   = -skew(u) * J_w
```

工程实现时建议：

- 先用有限差分做数值雅可比作为测试 oracle。
- C++ 中实现解析或半解析雅可比。
- 用至少三个姿态验证：中性位、接近限位、接近奇异。
- 对低秩、NaN、Inf、单位向量不合法做显式失败。

这里的核心问题是：`q_dot` 是关节速度，而 RCM 误差是工具轴线相对入口点的横向偏差。中间需要雅可比把“关节怎么动”映射成“工具轴线怎么动”。

法兰 twist 可拆成线速度和角速度：

```text
twist_F = [ v_F ] = [ J_v(q) ] q_dot
          [ w_F ]   [ J_w(q) ]
```

工具轴线上一点相对法兰有偏置 `r_B = R_BF * r_F`。当法兰转动时，这个偏置点不仅跟随法兰平移，还会因为角速度产生绕法兰的线速度：

```text
v_p = v_F + w_F x r_B
```

利用叉乘矩阵 `skew(a)`：

```text
skew(a) b = a x b
```

可以得到：

```text
w_F x r_B = -skew(r_B) w_F
J_p = J_v - skew(r_B) J_w
```

工具轴方向只受角速度影响，不受法兰平移影响：

```text
u_dot = w_F x u = -skew(u) w_F
J_u = -skew(u) J_w
```

有限差分验证方法：

```text
for each joint i:
  q_plus  = q; q_plus[i]  += eps
  q_minus = q; q_minus[i] -= eps

  numeric_position_column = (p(q_plus) - p(q_minus)) / (2 * eps)
  numeric_axis_column     = (u(q_plus) - u(q_minus)) / (2 * eps)

  compare numeric_position_column with J_p.col(i)
  compare numeric_axis_column with J_u.col(i)
```

测试要点：

- `eps` 不能太大，否则非线性误差明显；也不能太小，否则浮点误差明显。
- 对方向向量 `u` 做差分后，结果应接近垂直于 `u`，因为单位向量变化不应改变长度。
- 接近奇异位姿时误差可能变大，但必须能检测并记录，不应静默通过。
- 数值雅可比可以作为测试 oracle，不建议作为 live 高频控制的唯一实现。

### 5.7 RCM 约束雅可比

RCM 残差 `e` 是 `p0` 和 `u` 的函数，因此 `e_dot` 可以写成：

```text
e_dot = J_e(q) * q_dot
```

速度级 RCM 控制目标：

```text
J_e(q) * q_dot = -K * e
```

其中 `K` 是误差收敛增益。`K` 太大可能导致速度突变，太小会导致入口误差收敛慢。第一版建议在 mock 和仿真中保守选择，并把 `d_RCM`、`e_dot`、`task_scale`、`sigma_min` 输出到状态消息。

从残差公式出发：

```text
e = P r
P = I - u u^T
r = p_R - p0
```

如果 RCM 点 `p_R` 在控制周期内固定，则：

```text
r_dot = -p0_dot
P_dot = -(u_dot u^T + u u_dot^T)
e_dot = P_dot r + P r_dot
```

代入 `p0_dot = J_p q_dot`、`u_dot = J_u q_dot` 后，可以得到 `J_e`。工程上可以先不手推完整解析式，而是按以下步骤推进：

1. 用有限差分计算 `J_e_numeric`，验证方向和量级。
2. 在 C++ 中实现解析或半解析 `J_e`。
3. 单元测试比较 `J_e` 和 `J_e_numeric`。
4. 检查 `rank(J_e)`，正常情况下 RCM 横向约束秩应为 2。
5. 低秩、奇异或数值病态时返回 reason code，不继续求普通逆。

RCM 约束速度目标通常是让误差指数收敛：

```text
e_dot_desired = -K * e
```

`K` 的单位接近 `1/s`。如果控制周期是 `dt`，粗略理解为每个周期把误差向 0 拉回一小部分。第一版应小增益、低速度，先证明方向正确，再提高响应。

约束行选择也很重要。因为 `e` 在三维里只有两个独立方向，工程上常见做法包括：

- 使用局部横向基底 `a`、`b`，其中 `a`、`b` 都垂直于 `u`，只控制 `[a^T e, b^T e]`。
- 使用三维残差但通过 SVD/秩判断处理低秩，不对三维矩阵直接求普通逆。
- 使用叉乘残差 `u x r`，同时明确它也是秩 2 约束。

推荐第一版优先把状态输出做完整：

```text
d_RCM
e_RCM[3]
lambda_on_axis
rank_J_e
sigma_min
condition_number
task_scale
solver_status
reason_code
```

这些量比“机械臂动了”更重要，因为它们能证明动的是 RCM 约束，而不是普通末端控制。

### 5.8 RCM 兼容任务：pivot、insertion、roll

RCM 控制不应让上层直接发送任意 TCP 6D pose。更合适的是发送 RCM 兼容任务：

| 任务 | 含义 | 对 RCM 的影响 |
| --- | --- | --- |
| `pivot` | 绕 RCM 点改变工具方向 | 需要保持工具轴线穿过 RCM 点 |
| `insertion` | 沿工具轴线插入或退出 | 不应增加横向残差 |
| `roll` | 绕工具轴线自转 | 理想情况下不改变 RCM 横向误差 |

#### pivot 摆动

pivot 不是让 TCP 在空间里任意移动，而是改变工具方向 `u_B`，同时让工具轴线仍穿过 `p_R_B`。可以理解为工具绕入口点转动。

工程上常见输入可以是：

```text
pivot_velocity_x
pivot_velocity_y
```

这两个速度应定义在一个和工具轴垂直的局部横向基底上，而不是随便用世界坐标 x/y。原因是工具方向变化始终发生在垂直于工具轴的两个自由度上。

#### insertion 插入

insertion 是沿工具轴线移动：

```text
v_insert = insertion_velocity * u_B
```

如果 `d_RCM` 已经超过进入阈值，应该先纠偏或 hold，不要继续插入。插入会增加入口接触风险，不能在 RCM 横向误差不可控时执行。

#### roll 滚转

roll 是绕工具轴线自转，角速度方向为 `u_B`：

```text
w_roll = roll_velocity * u_B
```

理论上 roll 不改变工具轴线位置，因此不应增加 RCM 横向误差。但真实机械臂中，如果工具轴线标定偏、夹具有偏心或 wrist 近奇异，roll 也可能引入横向扰动，所以仍需要 `d_RCM` 监控。

任务优先级建议：

1. RCM 横向误差约束。
2. 关节限位、速度限制、奇异性和硬件安全。
3. insertion 和 pivot 任务。
4. roll 任务。
5. 平滑性、姿态偏好等二级目标。

不要把所有任务都当成同权重目标。六轴机械臂自由度有限，权重和优先级必须明确。

### 5.9 DLS、QP 和约束求解

低速原型可以先用阻尼最小二乘 DLS：

```text
q_dot = J^T (J J^T + lambda^2 I)^-1 v_task
```

DLS 适合验证公式和接口，不适合作为最终安全求解器。原因：它很难自然表达关节位置预测限制、速度限制、碰撞、入口禁区、任务缩放和故障原因。

生产方向更适合 QP/LP/SNS 或等价约束求解器：

```text
minimize    ||J_task q_dot - v_task||^2 + w ||q_dot||^2
subject to  q_dot_min <= q_dot <= q_dot_max
            q_next_min <= q + q_dot * dt <= q_next_max
            d_RCM_next <= threshold
```

关键点：不要先求出 `q_dot` 再逐关节硬裁剪。逐关节裁剪可能破坏 RCM 等式，让入口误差反而增大。正确做法是求解器内部约束或整体 task scale。

#### DLS 适合做什么

DLS 的优势是简单、稳定、容易写测试。它适合第一版 mock controller：

- 验证 `J_task` 方向是否正确。
- 验证 `d_RCM` 是否能下降。
- 验证 message、状态机和日志字段是否完整。
- 验证 singular 时阻尼是否避免速度爆炸。

DLS 的基本输入输出：

```text
input:
  J_task      # 任务雅可比
  v_task      # 目标任务空间速度
  lambda      # 阻尼
  q_dot_limit # 关节速度限制

output:
  q_dot
  task_scale
  sigma_min
  condition_number
  solver_status
  reason_code
```

DLS 里的 `lambda` 不是越大越好：

| `lambda` | 现象 | 风险 |
| --- | --- | --- |
| 太小 | 接近奇异时速度很大 | 机械臂抖动或超过限制 |
| 适中 | 稳定且能跟踪任务 | 需要测试调参 |
| 太大 | 输出过于保守 | 响应慢，RCM 误差收敛慢 |

#### 为什么生产方向更偏 QP

真实机械臂控制不只是求一个数学上接近目标的 `q_dot`，还要满足许多硬约束：

- 每个关节速度不能超限。
- 下一周期预测关节位置不能越限。
- RCM 残差不能超过阈值。
- 接近奇异时要降速。
- command 过期不能继续运动。
- 碰撞或工作空间边界要阻止任务。

这些约束更适合写进优化问题，而不是求解后补丁式处理。QP 可以把目标和约束分开：

```text
minimize:
  ||J_rcm q_dot - v_rcm||^2
  + w_insert ||J_insert q_dot - v_insert||^2
  + w_roll ||J_roll q_dot - v_roll||^2
  + w_smooth ||q_dot - q_dot_prev||^2

subject to:
  q_dot_min <= q_dot <= q_dot_max
  q_min + margin <= q + q_dot * dt <= q_max - margin
  task_scale_min <= task_scale <= 1
```

如果引入 `task_scale`，可以在任务不可完全满足时整体降速，而不是逐关节裁剪：

```text
J_task q_dot = task_scale * v_task
0 <= task_scale <= 1
```

#### SNS / LP 思路

SNS（Saturation in the Null Space）或 LP/QP 类方法的目标是处理关节速度饱和：当某个关节达到限制时，把它固定在允许边界，再在剩余自由度中继续求解任务。它比简单裁剪更可靠，因为它知道哪些关节已经饱和，并重新计算剩余任务。

对本项目来说，第一版可以按以下路线：

```text
离线 Python 验证 DLS
  -> C++ DLS mock controller
  -> 加入速度/位置预测限制和 task_scale
  -> 替换或扩展为 QP/SNS
  -> HIL 只读验证状态和失败路径
  -> 低能量 live
```

不要一开始就追求复杂求解器。先用 DLS 把几何、雅可比和接口打通，再用同一套测试集验证 QP/SNS 的改进。

### 5.10 速度、滤波和流畅性

机械臂运行是否流畅，不只取决于求解器，还取决于速度连续性、限幅方式和控制周期。理论上建议关注以下量：

| 量 | 含义 | 建议 |
| --- | --- | --- |
| `q_dot` | 关节速度 | 有上限，不能突变 |
| `q_ddot` | 关节加速度 | 用速度斜率限制间接控制 |
| `task_scale` | 任务整体缩放 | 接近限制时平滑降低 |
| `d_RCM` | RCM 横向误差 | 超阈值禁止插入 |
| `sigma_min` | 最小奇异值 | 接近 0 时降速或 hold |
| `condition_number` | 条件数 | 过大说明数值病态 |

比逐关节硬裁剪更好的处理方式：

```text
1. 先在求解器中加入速度上下界。
2. 若任务不可行，整体降低 task_scale。
3. 对输出速度做斜率限制，避免周期间跳变。
4. 接近奇异或限位时提前降速，而不是到边界才停。
5. 输出 reason code，让用户知道是限位、奇异还是 RCM 超差。
```

理论上，速度连续比位置点连续更重要。很多机械臂抖动不是因为路径点错，而是因为相邻周期速度方向和大小变化太突然。

### 5.11 奇异、限位和不可行任务

奇异位姿下，小的任务空间速度可能需要很大的关节速度才能实现。DLS、QP 和 task scale 都是在处理这个问题，但它们不能改变机械臂本身的几何限制。

奇异风险指标：

```text
sigma_min(J_task) 很小
condition_number(J_task) 很大
q_dot_norm 远大于 v_task_norm
task_scale 被压得很低
```

限位风险不仅看当前 `q`，还要看下一周期预测：

```text
q_next = q + q_dot * dt
```

如果 `q_next` 会进入限位 margin，求解器应该降低或禁止该方向速度。不要等关节已经越限后再 fault。

不可行任务例子：

- 用户要求继续插入，但 RCM 横向误差已经超过 fault threshold。
- 工具接近奇异位姿，pivot 方向需要极大 wrist 速度。
- 关节接近限位，任务要求继续往限位方向运动。
- MoveIt 给出的 approach pose 和当前 RCM 点几何上不兼容。

不可行不是 bug。真正的 bug 是系统明明不可行，却继续输出看似正常的命令。

### 5.12 求解失败怎么处理

必须区分两类失败：

| 类型 | 例子 | 处理 |
| --- | --- | --- |
| 任务不可行 | 目标速度太大、接近限位、奇异、约束冲突 | task scale、hold、replan、retract |
| 系统不可信 | feedback 过期、TF 缺失、SDK error、tool mismatch、watchdog timeout | 进入 `FAULT` 或 fail-closed |

禁止做法：

- solver 失败后沿用上一条非零命令。
- RCM 残差增长时继续插入。
- 用默认零位或全零反馈替代读取失败。
- 用普通 TCP pose 控制假装满足 RCM。

正确日志至少包含：`q`、`q_dot`、`d_RCM`、`sigma_min`、`condition_number`、`solver_status`、`reason_code`、输入 command timestamp、feedback timestamp 和 active constraints。

失败处理建议按优先级执行：

```text
1. 输入不可信：直接 HOLD/FAULT
   - command stale
   - feedback stale
   - TF missing
   - tool/calibration mismatch
   - SDK/CAN error

2. 几何不安全：禁止继续任务
   - d_RCM > fault_threshold
   - RCM 点不在有效直杆段
   - 工具轴线无效或未标定

3. 求解不可行：降速或 HOLD
   - singularity near
   - joint limit near
   - QP infeasible

4. 任务可行但风险升高：缩放任务
   - task_scale < 1
   - 降低 pivot/insertion/roll 速度
```

状态转换建议：

| 条件 | 状态动作 | 输出 |
| --- | --- | --- |
| command 过期 | `HOLD` | 零速度或保持目标，reason=`COMMAND_STALE` |
| feedback 过期 | `FAULT` 或 fail-closed | 不继续估计，reason=`FEEDBACK_STALE` |
| `d_RCM` 超进入阈值 | 不进入 `RCM_ACTIVE` | reason=`RCM_ERROR_HIGH` |
| `d_RCM` 超 fault 阈值 | `FAULT` | stop action，保存日志 |
| solver infeasible | `HOLD` | reason=`SOLVER_INFEASIBLE` |
| 接近奇异 | 降速或 `HOLD` | reason=`SINGULARITY_NEAR` |
| 接近关节限位 | task scale 或 `HOLD` | reason=`JOINT_LIMIT_NEAR` |

### 5.13 理论到实现前的最小检查题

进入第 6 章写 C++ controller 前，应能回答下面问题：

- `p_R_B` 从哪里来？坐标系、时间戳、标定版本是什么？
- `p0_F` 和 `u_F` 怎么得到？是否有 RMS 和有效直杆段？
- `d_RCM` 的进入阈值和 fault 阈值是多少？依据是什么？
- `J_p`、`J_u`、`J_e` 如何用有限差分验证？
- 六轴机械臂当前任务是否可行？哪些目标是硬约束，哪些是软目标？
- DLS 输出被限速时，是否仍能解释 RCM 等式是否满足？
- QP/SNS 不可行时，状态机进入 HOLD 还是 FAULT？
- 哪些 reason code 会阻止进入 `RCM_ACTIVE`？

如果这些问题答不上来，就先不要写 live 控制代码。可以继续做离线数学、mock、可视化和测试，但不能接真机 command owner。

### 5.14 推荐理论测试集

理论进入代码前，至少准备以下测试：

| 测试 | 输入 | 期望 |
| --- | --- | --- |
| 点在线上 | `p_R = p0 + lambda * u` | `d_RCM = 0` |
| 点横向偏移 | `u=[0,0,1]`，`p_R=[x,y,z]` | `d_RCM=sqrt(x^2+y^2)` |
| 非单位方向 | `u=[0,0,2]` | 拒绝或归一化并记录 |
| 有效段外 | `lambda < lambda_min` | 不允许进入 active |
| 雅可比有限差分 | 多个 `q` 姿态 | 解析和数值误差小于阈值 |
| 接近奇异 | `sigma_min` 很小 | 降速或 reason=`SINGULARITY_NEAR` |
| 接近限位 | `q_next` 越过 margin | task scale 或 HOLD |
| solver 不可行 | 约束冲突 | reason=`SOLVER_INFEASIBLE` |

这些测试不需要真机。它们应该先在离线数学和 C++ 单元测试中通过，再进入 mock ROS，再进入仿真和只读硬件。

---

## 6. 功能包：`agx_arm_controller`

### 6.1 作用和边界

`agx_arm_controller` 是 RCM 控制核心包。它应优先用 C++ 实现，原因是它位于控制闭环中，需要稳定周期、明确异常路径、较低延迟和可单元测试的数学库。

它应该放：

| 内容 | 说明 |
| --- | --- |
| C++ RCM 数学库 | FK/Jacobian 接口、工具轴、RCM 残差、求解器 |
| 重力补偿模型 | 机器人自身重力、工具载荷、输出限幅和 mock/sim 验证 |
| 状态机 | `DISABLED`、`READY`、`RCM_ACTIVE`、`HOLD`、`FAULT` |
| ROS wrapper | 订阅 command/joint state，发布 status/candidate command |
| 参数 | frame、阈值、速度限制、solver 参数、tool manifest 路径 |
| 测试 | 数学单元测试、状态机测试、ROS contract 测试 |

它不应该放：

- Piper SDK/CAN 直接调用。
- rcm_teleop 低层按钮解析。
- MoveIt 配置文件。
- MuJoCo 模型。
- 真机写命令最终下发逻辑。

### 6.2 推荐目录

```text
src/agx_arm_controller/
├── include/agx_arm_controller/
│   ├── kinematics_model.hpp
│   ├── tool_model.hpp
│   ├── gravity_compensator.hpp
│   ├── rcm_constraint_model.hpp
│   ├── velocity_optimizer.hpp
│   ├── safety_supervisor.hpp
│   └── rcm_controller_node.hpp
├── src/
│   ├── kinematics_model.cpp
│   ├── tool_model.cpp
│   ├── gravity_compensator.cpp
│   ├── rcm_constraint_model.cpp
│   ├── velocity_optimizer.cpp
│   ├── safety_supervisor.cpp
│   └── rcm_controller_node.cpp
├── config/
│   └── rcm_controller.yaml
├── launch/
│   └── rcm_controller_mock.launch.py
└── test/
    ├── test_rcm_error.cpp
    ├── test_gravity_compensator.cpp
    ├── test_jacobian.cpp
    ├── test_velocity_optimizer.cpp
    └── test_state_machine.cpp
```

### 6.3 一步步编写 C++ 控制器

第 1 步：先写纯 C++ library，不启动 ROS。

优先实现：

```text
ToolModel
  - 加载 tool manifest
  - 检查单位向量、有效轴段、tool_id、calibration_version

RCMConstraintModel
  - computeError(p0, u, pivot)
  - computeDistance(error)
  - computeConstraintJacobian(state, tool)

VelocityOptimizer
  - solve(task, bounds)
  - 返回 q_dot、task_scale、sigma_min、status、reason_code

SafetySupervisor
  - 检查 command/feedback/TF/tool/watchdog
  - 管理状态机和 fault latch
```

第 2 步：给数学库写测试。

先测不依赖 ROS 的函数：

| 测试 | 目的 | 失败路径 |
| --- | --- | --- |
| `test_rcm_error.cpp` | 点到线残差正确 | 非单位向量、NaN、pivot 不合法 |
| `test_jacobian.cpp` | 点/方向雅可比正确 | 有限差分误差超限 |
| `test_velocity_optimizer.cpp` | DLS/QP 输出可解释 | singular、limit、infeasible |
| `test_state_machine.cpp` | 状态转换正确 | timeout、tool mismatch、watchdog timeout |

第 3 步：再写 ROS node wrapper。

ROS wrapper 只做：

```text
参数读取
订阅 RCMCommand
订阅 joint state 或 mock feedback
调用 C++ 数学库
发布 RCMStatus
发布 candidate command 给 command owner 或 mock sink
```

不要把 RCM 公式直接写在 callback 里。callback 应只收集输入，控制 tick 中读取缓存并执行固定顺序。

第 4 步：加状态机。

推荐状态：

```text
DISABLED
  -> SELF_CHECK
  -> READY
  -> APPROACH_RCM
  -> RCM_ACTIVE
  -> HOLD / RETRACT / FAULT
```

进入 `RCM_ACTIVE` 前必须检查：

- joint feedback 新鲜。
- command 新鲜。
- TF 完整。
- tool ID 和 calibration version 匹配。
- `d_RCM` 小于进入阈值。
- solver 可行。
- operator enable 和 watchdog 有效。

第 5 步：只在 mock 中跑通。

第一版 controller 不接真实 SDK。用 mock joint state、固定 tool manifest、固定 RCM 点和低速 command 验证状态消息、残差趋势和失败路径。

### 6.4 推荐参数

```yaml
rcm_controller:
  ros__parameters:
    base_frame: base_link
    flange_frame: link6
    command_timeout_sec: 0.10
    feedback_timeout_sec: 0.05
    gravity_compensation_enabled: false
    gravity_vector_base_mps2: [0.0, 0.0, -9.80665]
    tool_payload_mass_kg: 0.0
    tool_payload_com_tool_m: [0.0, 0.0, 0.0]
    max_gravity_torque_nm: [2.0, 2.0, 2.0, 1.0, 1.0, 1.0]
    max_gravity_torque_rate_nmps: [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
    rcm_entry_threshold_m: 0.002
    rcm_fault_threshold_m: 0.005
    max_insertion_velocity_mps: 0.005
    max_roll_velocity_radps: 0.05
    max_joint_velocity_radps: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
    damping_lambda: 0.01
    min_sigma: 0.02
    tool_manifest_path: "REPLACE_WITH_TOOL_YAML"
    hardware_write_enabled: false
```

默认 `hardware_write_enabled` 必须是 `false`。controller 本身也不应该直接写 SDK；该字段只用于防止 launch 参数误解和状态显示。

默认 `gravity_compensation_enabled` 也必须是 `false`。只有模型、动力学参数、工具载荷、输出限幅、mock/sim 验证和 H gate 都满足时，才能把重力补偿从计算结果推进到真实候选命令。

### 6.5 C++ 接口示意

```cpp
// PLANNED：文档示意，不代表当前仓库已经实现。
struct RCMError {
  Eigen::Vector3d vector_m;
  double distance_m;
  bool valid;
  std::string reason_code;
};

struct VelocitySolution {
  Eigen::VectorXd q_dot_radps;
  double task_scale;
  double sigma_min;
  double condition_number;
  SolverStatus status;
  std::string reason_code;
};

class RCMConstraintModel {
 public:
  RCMError computeError(
      const Eigen::Vector3d& axis_point_base,
      const Eigen::Vector3d& axis_direction_base_unit,
      const Eigen::Vector3d& pivot_base) const;

  Eigen::MatrixXd computeJacobian(
      const RobotState& state,
      const ToolModel& tool) const;
};
```

### 6.6 重力补偿教程

重力补偿不是 `rcm_teleop` 中的 `GRAVITY_COMP` 字符串本身。当前输入包只能表达“操作者请求进入重力补偿相关模式”。真正的重力补偿属于 controller、动力学模型、command owner 和硬件接口共同约束下的 `PLANNED` 能力。

本项目中建议把重力补偿分成四层推进：

| 层级 | 名称 | 允许做什么 | 禁止做什么 |
| --- | --- | --- | --- |
| L0 | dry-run mode label | `rcm_teleop` 发布 `GRAVITY_COMP`，用于观察状态流 | 宣称机械臂已被重力补偿 |
| L1 | 离线 / mock 计算 | 在 C++ library 中计算 `tau_g(q)`，写单元测试和日志 | 接真机写控制 |
| L2 | 仿真 / replay 验证 | 在 `/sim` 或 rosbag replay 中比较力矩趋势和限幅行为 | 把仿真成功当作真机安全证据 |
| L3 | 低能量真机候选命令 | 只在 H0-H8 通过后，由 command owner 限幅、限速、watchdog 后输出 | controller 直接调用 SDK/CAN |

#### 目标和非目标

重力补偿的目标是在已知关节角和工具载荷的情况下，估计抵消重力所需的关节力矩：

```text
tau_g(q) = inverse_dynamics(q, q_dot=0, q_ddot=0, gravity)
```

如果第一版暂时没有完整动力学库，也可以先只做 mock 输出和接口验证，但不能把“保持当前位置”“低速位置伺服”“厂商 teach mode”混称为自研重力补偿。

它不是：

- 不是 RCM 约束求解器。
- 不是阻抗控制。
- 不是碰撞检测。
- 不是硬件急停。
- 不是让机械臂自由拖动的完整拖拽示教系统。

#### 前置条件

进入 L1 之前必须有：

- URDF 或独立动力学参数中有可信的 link mass、center of mass、inertia。
- joint order、joint direction、单位和 feedback timestamp 已经和 description 对齐。
- 工具载荷 `tool_payload_mass_kg` 和 `tool_payload_com_tool_m` 有默认值，并能在 tool manifest 中覆盖。
- 重力方向 `gravity_vector_base_mps2` 和 `base_link` 姿态约定清楚。
- 输出单位明确是 Nm、current、vendor normalized value，三者不能混用。

进入 L3 之前还必须确认 Piper SDK/CAN 是否真的提供可控的 torque/current/gravity/teach 接口。如果厂商接口只支持位置或速度命令，第一版不能输出自研 `tau_g` 到硬件，只能使用厂商明确支持的安全模式，或者停留在 mock/sim。

#### 推荐内部接口

```cpp
// PLANNED：文档示意，不代表当前仓库已经实现。
struct GravityCompInput {
  RobotState state;
  ToolPayload payload;
  Eigen::Vector3d gravity_base_mps2;
  rclcpp::Time stamp;
};

struct GravityCompOutput {
  Eigen::VectorXd tau_g_nm;
  Eigen::VectorXd tau_limited_nm;
  bool valid;
  std::string reason_code;
};

class GravityCompensator {
 public:
  GravityCompOutput compute(const GravityCompInput& input) const;
};
```

第一版只要求 `compute()` 是纯函数：输入相同，输出相同；不读 ROS 参数、不访问文件、不调用 SDK。ROS wrapper 在控制 tick 外加载参数，在控制 tick 内只传入缓存好的 `RobotState` 和 `ToolPayload`。

#### 一步步实现

第 1 步：确认动力学来源。

优先顺序：

1. 厂商提供的 link mass / COM / inertia 对表。
2. 已验证 URDF inertial 字段。
3. 仿真模型中的 inertial 字段。
4. 暂时 mock 参数。

如果只能用 mock 参数，文档和状态必须标为 `PLANNED/mock`，不能作为真机依据。

第 2 步：写纯 C++ 重力补偿库。

推荐输出 `tau_g_nm` 和 `tau_limited_nm` 两组值。前者用于诊断，后者才允许进入候选命令链路。限幅和限速必须逐关节配置，不能用一个全局魔法数覆盖全部关节。

第 3 步：补单元测试。

最小测试集：

| 测试 | 目的 | 通过标准 |
| --- | --- | --- |
| `zero_payload` | 无工具载荷时只计算机械臂自身重力 | 输出有限、维度等于 joint 数 |
| `payload_scaling` | 工具质量增加后力矩趋势合理 | 相关关节力矩随质量单调变化 |
| `gravity_direction_flip` | 重力方向反转时符号变化可解释 | 主要力矩项符号反转或接近反转 |
| `limit_and_rate` | 限幅、限速生效 | 输出不超过配置边界 |
| `invalid_state` | NaN、维度错误、过期反馈 fail-closed | `valid=false` 且有 reason code |

第 4 步：接入 controller 状态机。

建议状态流：

```text
DISABLED
  -> SELF_CHECK
  -> READY
  -> GRAVITY_COMP_READY
  -> GRAVITY_COMP_ACTIVE
  -> HOLD / FAULT
```

进入 `GRAVITY_COMP_ACTIVE` 前必须检查：

- `gravity_compensation_enabled=true`。
- feedback 新鲜。
- tool payload 和 calibration version 匹配。
- 关节位置在软限位内。
- operator enable 有效。
- command owner 已进入低能量授权状态。
- 输出 `tau_limited_nm` 没有 NaN、没有超限、没有过快跳变。

第 5 步：只发布候选命令，不直接写硬件。

controller 只能发布 candidate gravity command 或 status。真正写硬件的节点仍然是 command owner。command owner 必须重新检查 timestamp、enable、限幅、限速和 fault latch。

第 6 步：先 mock，再 sim/replay，再只读硬件，再低能量。

顺序不能反：

```text
unit test
  -> mock joint state
  -> /sim replay
  -> SDK/CAN read-only 对比反馈
  -> 单关节低能量
  -> 多关节低能量
  -> 与 RCM_ACTIVE 状态联调
```

#### 和 RCM 的关系

RCM 控制解决“运动方向和约束”的问题；重力补偿解决“静态重力负载”的问题。两者可以在 candidate command 层组合，但必须先分别验证。

推荐组合顺序：

```text
RCM solver 产生 q_dot 或小步目标
gravity compensator 产生 tau_g
safety supervisor 检查状态、限幅和优先级
command owner 根据当前硬件模式选择实际下发接口
```

如果硬件当前只运行位置控制，`tau_g` 只能作为 diagnostics 或 feedforward 计划值，不应强行转换成位置偏移。只有硬件接口明确支持力矩、电流或厂商重力模式时，才允许进入写控制评审。

#### 故障和停止条件

以下情况必须进入 `HOLD` 或 `FAULT`：

- feedback timeout。
- joint order 或 joint count 不匹配。
- `tau_g_nm` 出现 NaN/Inf。
- 力矩超出逐关节限幅。
- 力矩变化率超出逐关节限速。
- 工具载荷未知或 calibration version 不匹配。
- SDK/CAN 返回模式不支持或写入失败。
- 操作者松开 enable，或急停/安全监督触发。

#### 最小验收标准

- 单元测试覆盖维度、符号、payload、限幅、限速和异常输入。
- mock launch 中能看到 `GRAVITY_COMP_READY`、`GRAVITY_COMP_ACTIVE`、`HOLD/FAULT` 转换。
- diagnostics 同时记录 `tau_g_nm`、`tau_limited_nm`、payload、gravity vector 和 reason code。
- 未通过 H8 前，没有任何真实 SDK/CAN 写控制。
- 低能量真机测试必须有人看护，有急停方案，有 rosbag 和现场记录。

### 6.7 控制 tick 顺序

```cpp
// PLANNED：控制周期伪代码。
void RCMController::update(const rclcpp::Time& now) {
  const auto command = command_buffer_.latest();
  const auto feedback = feedback_buffer_.latest();

  const auto input_status = safety_.validateInputs(now, command, feedback, tool_model_);
  if (!input_status.ok()) {
    publishHoldStatus(input_status.reason_code);
    return;
  }

  const auto robot_state = kinematics_.update(feedback.joint_state);

  if (command.mode == Mode::GRAVITY_COMP) {
    const auto gravity = gravity_compensator_.compute(
        {robot_state, tool_model_.payload(), gravity_base_, now});
    if (!gravity.valid) {
      publishHoldStatus(gravity.reason_code);
      return;
    }
    publishCandidateGravityCommand(gravity.tau_limited_nm);
    publishGravityStatus(gravity);
    return;
  }

  const auto axis = tool_model_.axisInBase(robot_state);
  const auto rcm_error = rcm_model_.computeError(axis.point, axis.direction, command.pivot_base);

  const auto guard = safety_.evaluateRcmGuards(rcm_error, robot_state);
  if (!guard.ok()) {
    publishHoldStatus(guard.reason_code);
    return;
  }

  const auto task = task_adapter_.toVelocityTask(command, axis, rcm_error);
  const auto solution = optimizer_.solve(robot_state, task, bounds_);

  if (!solution.ok()) {
    publishHoldStatus(solution.reason_code);
    return;
  }

  publishCandidateCommand(solution);
  publishStatus(rcm_error, solution);
}
```

控制周期内不要读 YAML、不要做文件 I/O、不要阻塞等待 TF、不要调用 SDK、不要打印大量日志。大量日志交给低频 diagnostics 或 rosbag。

### 6.8 验收标准

- `computeError()` 正负例通过。
- `GravityCompensator::compute()` 对 payload、重力方向、限幅、异常输入有测试。
- 雅可比有限差分测试通过。
- solver 对 singular、limit、infeasible 有 reason code。
- 状态机对 command timeout、feedback timeout、tool mismatch、watchdog timeout、gravity output invalid 有确定转换。
- mock launch 不接硬件也能发布 `RCMStatus`。
- controller 源码中没有 SDK/CAN include。

---

## 7. 功能包：`agx_arm_moveit_config`

### 7.1 作用和边界

`agx_arm_moveit_config` 用于 MoveIt 2 配置、规划、碰撞场景、fake execution 和 RViz MotionPlanning。它不是 RCM 闭环控制器，也不能绕过 command owner 执行真机写控制。

适合做：

- 进入 RCM 前的 approach pose。
- 退出 RCM 后的 retract pose。
- 碰撞模型和 planning scene 检查。
- fake controller 可视化。
- 离线路径审查。

不适合直接做：

- `RCM_ACTIVE` 内严格入口约束。
- 真机 command owner。
- SDK/CAN 写控制。

### 7.2 一步步生成 MoveIt 配置

前置条件：`agx_arm_description` 已经能安装和显示，joint limit、tool collision、tool frame 初版稳定。

第 1 步：使用 MoveIt Setup Assistant 生成配置。

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

选择内容：

- URDF 来自 `agx_arm_description` 安装路径。
- planning group 选择六轴 arm。
- end effector / tip frame 和 description 中一致。
- joint limits 先保守设置。
- controller 先用 fake execution。

第 2 步：生成目录。

```text
src/agx_arm_moveit_config/
├── config/
│   ├── agx_arm.srdf
│   ├── kinematics.yaml
│   ├── joint_limits.yaml
│   ├── moveit_controllers.yaml
│   └── planning_scene.yaml
├── launch/
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   └── fake_execution.launch.py
└── rviz/
    └── moveit.rviz
```

第 3 步：只跑 fake plan。

```bash
colcon build --packages-select agx_arm_moveit_config --symlink-install
source install/setup.bash
ros2 launch agx_arm_moveit_config demo.launch.py
```

第 4 步：检查 planning scene。

- 工具 collision 是否显示。
- 入口禁区是否可视化。
- joint limits 是否与 URDF 和硬件资料一致。
- fake execution 是否只在 RViz 中动，不接 hardware interface。

### 7.3 和 RCM controller 的关系

MoveIt 规划出的普通 TCP pose 不自动满足 RCM。推荐边界：

```text
MoveIt: approach / retract / collision scene / visualization
RCM controller: RCM_ACTIVE 内的局部约束和工具运动
command owner: 真机写命令唯一出口
```

如果后续要做 RCM-aware planning，可以研究 C++ constraint sampler、MoveIt Servo 插件或自定义规划约束，但仍必须输出 `d_RCM`、状态机、reason code 和验收证据。不能因为 MoveIt plan 成功就跳过 RCM controller。

### 7.4 验收标准

- demo launch 能打开 RViz MotionPlanning。
- fake plan 和 fake execution 可视化正常。
- planning group、tip frame、joint limits 与 description 对齐。
- Execute 不连接真实 hardware interface。
- 文档中明确 MoveIt 不是 RCM 闭环本体。

---

## 8. 功能包：`agx_arm_sim`

### 8.1 作用和边界

`agx_arm_sim` 用于 MuJoCo、replay、模型同步和仿真观察。第一版建议只读：订阅 joint state 或 rosbag，把状态同步到 MuJoCo，不发布真实硬件命令。

它应该放：

| 内容 | 说明 |
| --- | --- |
| MuJoCo/MJCF 模型 | 从 URDF 和 joint map 派生 |
| replay 节点 | 订阅 `/sim/joint_states` 或 rosbag |
| joint map sim | ROS joint name 到 MuJoCo qpos index |
| sim launch | 只在 `/sim` namespace 下运行 |
| 差异记录 | 惯量、阻尼、关节限制和真实机械臂差异 |

它不应该放：

- SDK/CAN 写命令。
- live command owner。
- 默认 remap 到 `/live` 的控制 topic。

### 8.2 推荐目录

```text
src/agx_arm_sim/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── mujoco_replay.launch.py
│   └── sim_view.launch.py
├── config/
│   ├── mujoco_replay.yaml
│   └── joint_map_sim.yaml
├── models/
│   └── piper_rcm.xml
└── src/
    └── mujoco_replay_node.cpp
```

### 8.3 一步步搭建仿真只读链路

第 1 步：先加载模型。

确认 MJCF 或转换模型能打开，关节数量、方向、限制和 RViz 中一致。

第 2 步：写 joint map。

```yaml
mujoco_replay:
  ros__parameters:
    model_path: "REPLACE_WITH_MJCF"
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    input_topic: /sim/joint_states
    publish_control_topics: false
```

第 3 步：写 replay 节点。

高频同步建议 C++，离线数据处理可以 Python。replay 节点只把输入 joint state 写入 MuJoCo qpos，不发布 live command。

第 4 步：使用 `/sim` namespace。

```text
/sim/joint_states
/sim/rcm_status
/sim/mujoco_state
```

禁止默认 remap 到 `/live/*`。

第 5 步：记录仿真和真实差异。

仿真成功只证明模型和接口映射可用，不证明真实机械臂安全。每次从仿真切到真机，都要重新检查 joint order、单位、timestamp、owner 和 stop action。

### 8.4 验收标准

- MuJoCo 能加载模型。
- replay 时 joint1-joint6 方向与 RViz 一致。
- 所有 sim topic 都在 `/sim` namespace 下。
- 没有任何 `/live/*` command 输出。
- 仿真差异记录可复查。

---

## 9. 功能包：`agx_arm_hw_interface`

### 9.1 作用和边界

`agx_arm_hw_interface` 是 Piper SDK/CAN 和 ROS 2 系统之间的硬件边界。第一版只做 read-only adapter；写控制 command owner 只能在 H0-H6 通过后开发。

它应该放：

| 内容 | 说明 |
| --- | --- |
| read-only adapter | 读取 SDK/CAN 状态，发布 joint state 和 diagnostics |
| joint map | SDK index 到 `joint1`-`joint6` 的映射 |
| unit conversion | 原始单位到 rad、rad/s、SI 单位 |
| watchdog | command freshness、feedback freshness、stop action |
| command owner | 后续唯一真实写命令出口 |
| tests | unit conversion、joint map、watchdog、fail-closed |

它不应该放：

- rcm_teleop 输入解析。
- RCM 数学和 solver。
- MoveIt 规划逻辑。
- MuJoCo 仿真控制。

### 9.2 真机前安全准备

真机上电前完成：

1. 机械臂固定在稳定底座上。
2. 工作空间内没有人、松散线缆、未固定工具或障碍物。
3. 急停或断电方式可触达，操作者和观察员都知道如何使用。
4. 末端工具和线缆不会被腕部旋转拉扯。
5. 第一次上电只允许只读反馈。
6. 禁止运行 SDK demo 的写控制示例。

建议 evidence 目录：

```text
evidence/YYYYMMDD_HHMM_piper_readonly/
├── 00_readme.md
├── 01_photos/
├── 02_versions/
├── 03_can_logs/
├── 04_sdk_readonly/
├── 05_joint_map/
└── 99_incidents/
```

evidence 可以放在外部受控存储，不一定提交进仓库。

### 9.3 SocketCAN 只读流程

安装工具：

```bash
sudo apt update
sudo apt install -y can-utils iproute2 ethtool usbutils
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb || true
```

识别设备：

```bash
lsusb
dmesg | tail -n 80
ip link show
```

配置 CAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate REPLACE_WITH_VENDOR_BITRATE
sudo ip link set can0 up
ip -details -statistics link show can0
```

只读抓包：

```bash
candump -L can0
```

通过条件：

- CAN interface 能 up。
- `candump` 有可解释帧或能明确证明无帧原因。
- 错误计数不持续暴增。
- 没有发送任何写控制帧。
- 日志保存了 bitrate、adapter、时间、设备状态。

### 9.4 SDK 只读流程

当前仓库未集成 Piper SDK。SDK 检查应先作为只读外部 probe。

记录安装：

```bash
pip3 show piper_sdk || true
python3 -c "import piper_sdk; print(piper_sdk)"
```

只读 probe 通过条件：

- SDK 能 import。
- 能记录 SDK 版本或安装路径。
- 能绑定 CAN interface。
- 能读取状态，或明确返回只读失败并保存 traceback。
- 失败时不发布默认零位、全零数组或上次状态。

只读输出建议字段：SDK 版本、CAN interface、构造参数、读取函数名、原始返回结构、解析后 joint 数组、timestamp、错误码、异常栈。

### 9.5 read-only adapter 一步步实现

推荐 C++ 包结构：

```text
src/agx_arm_hw_interface/
├── include/agx_arm_hw_interface/
│   ├── piper_readonly_adapter.hpp
│   ├── piper_command_owner.hpp
│   ├── joint_map.hpp
│   ├── unit_conversion.hpp
│   └── watchdog.hpp
├── src/
│   ├── piper_readonly_adapter.cpp
│   ├── piper_command_owner.cpp
│   ├── joint_map.cpp
│   ├── unit_conversion.cpp
│   └── watchdog.cpp
├── config/
│   ├── piper_readonly.yaml
│   └── piper_command_owner.yaml
├── launch/
│   ├── readonly_hardware.launch.py
│   └── command_owner_low_energy.launch.py
└── test/
    ├── test_unit_conversion.cpp
    ├── test_joint_map.cpp
    └── test_watchdog.cpp
```

第 1 步：只实现读取，不暴露写函数。

第 2 步：实现 joint map。

```yaml
piper_joint_map:
  ros__parameters:
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
    sdk_indices: [0, 1, 2, 3, 4, 5]
    position_unit: rad
    velocity_unit: rad_s
    direction_sign: [1, 1, 1, 1, 1, 1]
```

第 3 步：实现 unit conversion。

所有输出给 ROS 的角度、速度和时间都必须统一单位。单位不明时，adapter 应发布 diagnostics error，而不是猜。

第 4 步：发布 `joint_states` 和 diagnostics。

失败处理：

- 长度不为 6：不发布假 `joint_states`。
- NaN/Inf：进入 error diagnostics。
- timestamp 过期：标记 feedback stale。
- SDK/CAN error：fail-closed。

第 5 步：写测试。

- joint order 对表测试。
- 单位转换测试。
- error code 转 diagnostics 测试。
- timeout/watchdog 测试。

### 9.6 command owner 何时实现

command owner 只能在以下条件满足后实现：

- CAN 只读稳定。
- SDK 只读稳定。
- joint map 冻结。
- RCM controller mock/sim 验证通过。
- H0-H6 门控通过。
- 有观察员和急停方案。

command owner 必须具备：

- lifecycle 状态。
- explicit operator enable。
- command timestamp 和 `valid_for_sec` 检查。
- watchdog heartbeat。
- joint limit 和速度限制。
- stop/hold/fault action。
- single owner 检查。
- SDK/CAN error fail-closed。

写控制禁止直接接收 rcm_teleop 原始 axes；它只接收 controller/safety 授权后的命令。

---

## 10. 功能包：`agx_arm_bringup`

### 10.1 作用和边界

`agx_arm_bringup` 只负责组合 launch、参数、namespace 和安全默认值。它不写算法，不接 SDK 细节，不重新实现 RCM 公式。

bringup 的目标是让每个阶段有单独、可复现、默认安全的启动入口。

### 10.2 推荐目录

```text
src/agx_arm_bringup/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── display.launch.py
│   ├── joystick_dry_run.launch.py
│   ├── mock_rcm.launch.py
│   ├── sim_replay.launch.py
│   ├── readonly_hardware.launch.py
│   └── live_low_energy.launch.py
└── config/
    ├── common_frames.yaml
    ├── safety_limits.yaml
    ├── namespaces.yaml
    └── tool_manifest.yaml
```

### 10.3 一步步写 launch

第 1 步：`display.launch.py`。

启动 description、robot_state_publisher、joint_state_publisher_gui、RViz。禁止启动 controller 和 hardware。

第 2 步：`joystick_dry_run.launch.py`。

启动 `joy_node` 和 `rcm_teleop` dry-run。只观察 topic，不接 command owner。

第 3 步：`mock_rcm.launch.py`。

启动 mock joint state、RCM controller、status 输出。默认 `hardware_write_enabled:=false`。

第 4 步：`sim_replay.launch.py`。

启动 `/sim` namespace 下的 replay/MuJoCo。禁止 remap 到 `/live`。

第 5 步：`readonly_hardware.launch.py`。

启动 CAN/SDK read-only adapter 和 diagnostics。禁止启动 command owner。

第 6 步：`live_low_energy.launch.py`。

只有 H0-H6 通过后才写。必须显式传入：

```text
hardware_write_enabled:=true
evidence_dir:=...
tool_manifest:=...
operator_enable_required:=true
speed_scale_limit:=...
```

没有这些参数时，launch 应失败或保持只读。

### 10.4 launch 默认值

推荐所有 launch 参数默认安全：

```text
hardware_write_enabled:=false
use_sim_time:=false
namespace:=/mock 或 /sim
allow_sdk_write:=false
require_deadman:=true
require_watchdog:=true
```

live launch 不能和 fake/sim controller 同时抢 command owner。一个 launch 文件头部应写清楚：本 launch 属于哪个阶段、是否允许写硬件、启动后要观察哪些 topic、失败时怎么停。

### 10.5 验收标准

- 每个 launch 能单独说明阶段和风险。
- 默认不会写硬件。
- `/sim`、`/mock`、`/live` namespace 不混淆。
- live launch 必须显式参数开启写控制。
- bringup 中没有 RCM 公式、SDK 细节或 rcm_teleop 解析逻辑。

---

## 11. 从 mock 到真机的逐步验收

本章是操作检查清单，不是抽象计划。每一步都必须有输入条件、执行命令、通过标准和停止条件。

### 11.1 H0：仓库和环境基线

输入条件：不接真机。

执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

通过：当前已有包能构建，已有测试能跑，失败项可解释。

停止：构建失败、依赖缺失、package manifest 不一致。

### 11.2 H1：模型显示

输入条件：`agx_arm_description` 资源安装完成。

执行：

```bash
ros2 launch agx_arm_description display_piper.launch.py
```

通过：RViz 显示模型，TF tree 连续，joint direction 可解释。

停止：mesh 缺失、URDF 解析失败、joint/frame 命名和文档不一致。

### 11.3 H2：输入 dry-run

输入条件：不接硬件写口。

执行：

```bash
ros2 launch rcm_teleop rcm_teleop.launch.py
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
```

通过：手柄输入稳定，deadzone 和 mode 可复查。

停止：输入漂移、按钮映射不明、断开手柄后仍持续输出有效 intent。

### 11.4 H3：RCM 数学单元测试

输入条件：`agx_arm_controller` C++ 数学库完成。

执行：

```bash
colcon test --packages-select agx_arm_controller --event-handlers console_direct+
colcon test-result --verbose
```

通过：残差、雅可比、solver、状态机测试都通过。

停止：有限差分不一致、solver failure 无 reason code、状态机 failure path 未覆盖。

### 11.5 H4：mock RCM controller

输入条件：typed command 和 mock feedback 可用。

执行：

```bash
ros2 launch agx_arm_bringup mock_rcm.launch.py
ros2 topic echo /rcm_status
```

通过：不接硬件也能看到 `d_RCM`、solver status、reason code、task scale。

停止：status 缺字段、command 过期仍 active、feedback 过期仍输出 candidate command。

### 11.6 H5：仿真和 replay

输入条件：`/sim` namespace 隔离完成。

执行：

```bash
ros2 launch agx_arm_bringup sim_replay.launch.py
ros2 topic list | grep /sim
```

通过：MuJoCo/replay 只读同步，无 `/live/*` command 输出。

停止：仿真 topic remap 到 live、joint order 不一致、仿真成功被当作真机安全证据。

### 11.7 H6：CAN/SDK 只读

输入条件：现场安全准备完成，禁止写控制。

执行：

```bash
ip -details -statistics link show can0
candump -L can0
```

再运行 SDK 只读 probe 或 read-only adapter。

通过：反馈可读，joint map 初步可解释，error code 可记录。

停止：CAN error 暴增、SDK 读取失败且不可复盘、joint order/单位不明。

### 11.8 H7：read-only adapter 接入 ROS

输入条件：CAN/SDK 只读稳定。

执行：

```bash
ros2 launch agx_arm_bringup readonly_hardware.launch.py
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

通过：真实反馈进入 ROS，diagnostics 能显示 SDK/CAN 状态，invalid feedback 不发布假数据。

停止：反馈过期仍正常发布、单位不明、diagnostics 吞错。

### 11.9 H8：低能量单关节真机

输入条件：H0-H7 通过，command owner 完成并 review，现场有观察员和急停方案。

执行原则：

- 单关节。
- 小步长。
- 低速度。
- 短时。
- 每次只改变一个变量。
- 记录 command、feedback、diagnostics、视频或照片。

通过：命令和反馈方向一致，stop action 可用，watchdog 可实测。

停止：方向不一致、停止延迟不可接受、SDK/CAN error、机械臂异常声音/振动/线缆拉扯。

### 11.10 H8.5：低能量重力补偿真机

输入条件：H8 通过，`GravityCompensator` 单元测试、mock/sim/replay 验证通过，工具载荷和重力方向已记录，command owner 已确认硬件接口支持对应输出模式。

执行原则：

- 先不装工具或使用最小已知载荷。
- 先单关节，再多关节。
- 先短时使能，再逐步延长观察时间。
- 每次只改变一个姿态或一个载荷参数。
- 全程记录 `tau_g_nm`、`tau_limited_nm`、joint feedback、diagnostics、SDK/CAN mode 和现场视频。

通过：使能重力补偿后机械臂没有突跳、漂移、异常振动或持续增大的跟随误差；松开 enable 或触发 stop 后立即退出到 `HOLD` 或 `FAULT`；日志能复盘每个力矩限幅和 reason code。

停止：关节反向、力矩突变、SDK/CAN 报错、模式不支持、工具载荷不确定、机械臂下坠/上冲、操作者无法稳定停止。

### 11.11 H9：低能量 RCM 真机

输入条件：H8 通过，RCM tool manifest 和 pivot calibration 冻结。如果 RCM 真机阶段启用重力补偿，则 H8.5 也必须通过。

执行原则：

- 只做小范围 pivot。
- 再做小范围 insertion。
- 最后做低速 roll。
- 全程记录 `d_RCM` 和 reason code。
- RCM 残差超阈值立即 hold/fault。

通过：低速 RCM 任务中 `d_RCM` 在阈值内，状态机退出条件可触发，stop action 可复查。

停止：残差增长、插入时入口横向偏移、solver failure 后仍输出命令、日志不足以复盘。

---

## 12. 故障处理、接口契约和文档维护

### 12.1 常见故障处理

| 现象 | 优先检查 | 正确处理 |
| --- | --- | --- |
| URDF 显示缺 mesh | install 规则、`package://` 路径、文件大小写 | 回到 description 修路径，不在 RViz 临时加载本地文件 |
| 手柄静止仍漂移 | deadzone、手柄校准、axes 编号 | 增大 deadzone，记录手柄型号和映射 |
| RCM 残差符号反了 | tool axis、frame、joint direction、雅可比有限差分 | 回到 description/tool manifest，不在 solver 硬翻符号 |
| solver 输出速度过大 | singular、condition number、lambda、速度限制 | task scale 或 hold，不逐关节硬裁剪后继续 |
| CAN error 暴增 | bitrate、终端电阻、供电、接地、线缆 | 停止真机测试，只保留只读排查 |
| SDK 读取失败 | SDK 版本、CAN interface、权限、firmware | 保存 traceback，不发布假反馈 |
| live launch 意外启动写口 | launch 默认值、hardware_write_enabled、command owner | 立即停止，修 bringup 默认安全值 |
| MoveIt plan 成功但 RCM 偏了 | MoveIt 不保证 RCM、tool frame、RCM controller 未接管 | 不执行真机，回到 controller/mock 验证 |

### 12.2 接口契约必须写清楚

每个跨包 topic/service/action 都应记录：

```text
名称：/rcm_command
类型：rcm_msgs/msg/RCMCommand
发布者：rcm_teleop 或上层 intent adapter
订阅者：agx_arm_controller
frame：base_link
单位：m、rad、m/s、rad/s
频率：按输入源，controller 检查 valid_for_sec
超时：command_timeout_sec
失败语义：过期后 controller HOLD，不沿用旧命令
```

硬件相关接口还必须记录：

- joint order。
- 单位。
- timestamp 来源。
- SDK/CAN error 映射。
- stop action。
- owner 状态。

### 12.3 reason code 建议

固定 reason code 有利于日志检索和自动测试。建议使用类似：

| reason code | 含义 |
| --- | --- |
| `OK` | 正常 |
| `COMMAND_STALE` | 命令过期 |
| `FEEDBACK_STALE` | 反馈过期 |
| `TF_MISSING` | TF 缺失或时间不一致 |
| `TOOL_MISMATCH` | tool_id 不匹配 |
| `CALIBRATION_MISMATCH` | 标定版本不匹配 |
| `RCM_ERROR_HIGH` | RCM 残差超限 |
| `SOLVER_INFEASIBLE` | 求解不可行 |
| `SINGULARITY_NEAR` | 接近奇异 |
| `JOINT_LIMIT_NEAR` | 接近或预测越过限位 |
| `WATCHDOG_TIMEOUT` | watchdog 超时 |
| `SDK_ERROR` | SDK 返回错误 |
| `CAN_ERROR` | CAN 通信错误 |
| `OWNER_CONFLICT` | command owner 冲突 |
| `OPERATOR_RELEASED` | 操作者释放 deadman/enable |

### 12.4 记录模板

RCM 标定记录：

```text
date:
operator:
tool_id:
calibration_version:
base_frame:
flange_frame:
axis_point_flange_m:
axis_direction_flange:
lambda_min_m:
lambda_max_m:
fit_rms_m:
source_bag/log:
review_result:
```

H gate 记录：

```text
gate: H0/H1/...
date:
operator:
observer:
command:
expected_result:
actual_result:
logs:
incidents:
pass/fail:
next_allowed_step:
```

incident 记录：

```text
time:
stage:
symptom:
last_command:
feedback_state:
diagnostics:
stop_action:
root_cause:
fix:
retest_required:
```

### 12.5 文档维护规则

- 修改 topic、message、frame、单位、timeout、reason code 后，必须同步本文。
- 修改 joint order、tool axis、tool manifest、calibration version 后，必须同步本文和 evidence。
- 新增 live launch 前，必须写清默认是否写硬件、需要哪些 H gate。
- 从 `PLANNED` 改成 `CURRENT` 前，必须补路径、命令和验收结果。
- 不把外部项目映射写进本项目文档；本手册只描述当前项目。

---

## 附录 A：命令速查

构建全部：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建单包：

```bash
colcon build --packages-select rcm_teleop --symlink-install
```

运行当前 rcm_teleop dry-run：

```bash
ros2 launch rcm_teleop rcm_teleop.launch.py
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
```

检查接口：

```bash
ros2 interface list | grep rcm_msgs
ros2 interface show rcm_msgs/msg/RCMCommand
```

重力补偿开发阶段检查：

```bash
# PLANNED：等 agx_arm_controller 测试落地后执行。
colcon test --packages-select agx_arm_controller --event-handlers console_direct+
colcon test-result --verbose
```

检查 package：

```bash
ros2 pkg list | grep agx
ros2 pkg prefix agx_arm_description
ros2 pkg executables rcm_teleop
```

CAN 只读：

```bash
ip -details -statistics link show can0
candump -L can0
```

文档结构检查：

```bash
awk '/^````/ {in4=!in4; next} /^```/ && !in4 {in3=!in3; next} !in3 && !in4 && /^##[#]? / {print FNR ":" $0}' docs/PIPER_RCM_DEVELOPMENT_GUIDE.md
git diff --check -- docs/PIPER_RCM_DEVELOPMENT_GUIDE.md
```

---

## 附录 B：功能包状态表

| 功能包 | 当前状态 | 下一步 | 是否允许写硬件 |
| --- | --- | --- | --- |
| `rcm_teleop` | `CURRENT` dry-run | typed intent、deadman、timeout 测试 | 不允许 |
| `agx_arm_description` | `CURRENT` 源文件 / `PLANNED` 显示完善 | install、display launch、tool frames | 不允许 |
| `rcm_msgs` | `PLANNED` | 新增接口包并冻结字段 | 不允许 |
| `agx_arm_controller` | `CURRENT` 骨架 / `PLANNED` 控制器 | C++ 数学库、重力补偿、状态机、mock launch | 不直接写硬件 |
| `agx_arm_moveit_config` | `PLANNED` | fake plan、planning scene | 不允许 |
| `agx_arm_sim` | `PLANNED` | `/sim` replay、MuJoCo 只读同步 | 不允许 |
| `agx_arm_hw_interface` | `PLANNED` | read-only adapter，后续 command owner | H0-H6 前不允许 |
| `agx_arm_bringup` | `CURRENT` 骨架 / `PLANNED` launch 编排 | 分阶段 launch、安全默认值 | 只有 live_low_energy 且 H gate 通过 |
| `scripts/` | `PLANNED` | CAN/SDK 只读和证据脚本 | 不允许持续控制 |

---

## 附录 C：术语和禁止事项

### C.1 术语

| 术语 | 含义 |
| --- | --- |
| RCM | 工具轴线穿过固定远心点的约束 |
| pivot | RCM 固定点 |
| tool axis | 工具功能轴线，不一定等于外形中心线 |
| insertion | 沿工具轴线插入或退出 |
| roll | 绕工具轴线旋转 |
| gravity compensation | 根据关节角、重力方向和工具载荷估计抵消重力的关节力矩 |
| command owner | 唯一拥有真实硬件写控制权的节点 |
| fail-closed | 不可信或失败时停止输出危险命令 |
| reason code | 可检索、可测试的故障原因字符串 |

### C.2 禁止事项

- 禁止 rcm_teleop、MoveIt、MuJoCo、测试脚本直接调用 SDK 写控制。
- 禁止多个节点同时拥有真实硬件写命令权。
- 禁止 SDK/CAN 读取失败后发布默认零位或全零假反馈。
- 禁止 solver 失败后沿用上一条非零命令。
- 禁止用普通 TCP pose 控制宣称满足 RCM。
- 禁止逐关节硬裁剪后继续宣称 RCM 等式满足。
- 禁止仿真成功后跳过 CAN/SDK 只读和 H gate。
- 禁止 live launch 默认开启写控制。
- 禁止把 `GRAVITY_COMP` 字符串或位置保持模式当作已验证重力补偿。
- 禁止在未确认硬件 torque/current/gravity 接口前，把 `tau_g` 输出写入真实机械臂。

### C.3 参考资料

- ROS 2 Interfaces: `https://docs.ros.org/en/ros2_documentation/rolling/Concepts/Basic/Interfaces-Topics-Services-Actions.html`
- ROS 2 `rclcpp`: `https://docs.ros.org/en/rolling/p/rclcpp/`
- ROS 2 `rclpy`: `https://docs.ros.org/en/ros2_packages/iron/api/rclpy/index.html`
- `ros2_control`: `https://control.ros.org/master/doc/ros2_control/doc/index.html`
- MoveIt 2: `https://moveit.picknik.ai/`
