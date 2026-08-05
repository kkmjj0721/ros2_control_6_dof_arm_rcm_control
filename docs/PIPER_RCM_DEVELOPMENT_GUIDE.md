# Piper RCM、松灵 Piper SDK 与安全开发中文指南（CURRENT / PLANNED / RESEARCH）

## 中文阅读入口 / 中文版本说明 / 松灵 SDK 使用总览

本文是本仓库的中文版本开发指南。少量英文仅保留为 ROS、API、package、topic、command、SDK 函数名、代码符号或状态机器标签；阅读时应以中文入口、中文安全边界和中文 SDK 补充章节为准。此前读者反馈的“乱码”主要不是字符编码错误，而是历史英文段落、英文 only 标题和 API 名混在中文工作流里导致不易阅读；本文件后续维护应优先补中文说明，再保留必要英文符号。

状态标签中文名如下：当前可用（CURRENT）表示当前仓库已经具备可验证文件或 dry-run 行为；计划实现（PLANNED）表示还需要新增源码、launch、测试、审查和硬件门控；研究验证（RESEARCH）表示只能做概念、数学、仿真、证据设计或只读验证，不能对真实机械臂发控制命令。

当前仓库没有集成 `piper_sdk`，没有 SDK adapter，没有 CAN adapter，也没有 live arm 写控制链路。本文中所有松灵 / AgileX Piper SDK 内容均是 PLANNED/RESEARCH 操作流程、证据清单和安全约束，不是当前仓库的实机控制教程；H0-H6 通过前只允许安装核对、版本记录、SocketCAN 只读检查、SDK 只读状态读取设计和 adapter 边界设计。

如何使用这份文档：新手先读本入口、第 0 章范围与安全契约、第 1 章当前仓库地图、第 2 章当前可安全运行命令和第 3 章状态标签；做 SDK/CAN 的读者先读第 5-6 章和文末“松灵 / AgileX Piper SDK 详细使用补充（中文）”；做 RCM、重力补偿或仿真的读者先读第 12-16 章，并始终按 H0-H6 门控和只读优先原则执行。

> 中文版本说明：本文档使用 UTF-8 编写。当前检查未发现典型编码乱码；此前“不顺/像乱码”的主要原因是英文标题和英文正文过多。本版将主标题、目录式章节标题、操作说明、安全边界和 SDK 说明改为中文，必要英文术语仅作为机器标签或 API 名保留。

## 0. 范围与安全契约

本文档是当前仓库的操作与二次开发指南。

它把已经能从仓库文件中验证的行为、计划中的工程工作、以及只允许研究验证的控制概念分开。

三个机器标签必须严格使用：

- `CURRENT`：当前仓库已经包含足够的源码、launch、模型或测试资产，可以按文档描述执行或观察。
- `PLANNED`：文档描述的是未来实现路径、接口约束或脚手架，当前仓库尚未实现完整功能。
- `RESEARCH`：该内容只能用于概念、数学、仿真或证据规划，未获准对真实机械臂发控制命令。

本文档中的 `CURRENT` 范围刻意收窄。

`CURRENT` 包含 ROS 2 workspace 目录结构。

`CURRENT` 包含 `joystick` 包。

`CURRENT` 包含源码中的 Python 节点 `gamepad_controller`。

`CURRENT` 包含一个 launch 文件，它启动标准 `joy_node` 和自定义手柄可执行文件。

`CURRENT` 在根命名空间下包含 `/joy`、`/rcm_mode`、`/rcm_cmd` 这些上层话题流。

`CURRENT` 包含 `src/agx_arm_description` 下的 Piper URDF 与 mesh 资产。

`CURRENT` 包含 bringup 与 controller 的空壳包，便于后续扩展。

`CURRENT` 不包含 Piper 实机硬件驱动。

`CURRENT` 不包含 `piper_sdk` 集成代码。

`CURRENT` 不包含 SDK adapter。

`CURRENT` 不包含 CAN adapter。

`CURRENT` 不包含 `ros2_control` hardware interface。

`CURRENT` 不包含 `joint_trajectory_controller` wiring。

`CURRENT` 不包含 MoveIt 配置包。

`CURRENT` 不包含 MuJoCo runtime bridge。

`CURRENT` 不包含经过验证的 RCM solver。

`CURRENT` 不包含真实机械臂上的重力补偿控制。

`CURRENT` 不包含 torque、impedance、current、gain-level 或 MIT mode 的 live control。

`PLANNED` 内容是路线图、接口约束和实现脚手架。

`PLANNED` 示例必须先落入源码、经过代码审查、测试和硬件门控，才允许用于真实硬件。

`RESEARCH` 内容只能用于概念、数学推导、仿真和证据设计。

任何 `RESEARCH` 控制模式在 H0-H9 硬件门控通过前，不得发送到 live arm。

本文中的 live arm 指已上电、连接 CAN、Ethernet、USB-CAN 或厂商传输链路的真实机械臂。

本文中的 dry run 指 ROS 节点、mock interface、静态模型、仿真或只读工具，不包含 actuator command。

本文中的 evidence package 指用于审查的日志、照片、版本、哈希、测试输出和操作者记录。

任何标为 `PLANNED 示例` 的代码块都不能直接复制到上电机械臂环境运行。

任何 joystick intent 都不能直接接入厂商写控制命令。

`GRAVITY_COMP` 只是当前手柄节点发布的字符串标签，不代表已经实现重力补偿。

`IMPEDANCE` 只是当前手柄节点发布的字符串标签，不代表已经实现阻抗控制。

`RCM_CONTROL` 只是当前手柄节点发布的字符串标签，不代表已经实现 RCM 运动学约束。

当前手柄节点只发布“意图型”消息。

当前手柄节点不拥有硬件安全职责。

当前手柄节点不发布 joint trajectory。

当前手柄节点不发布 wrench command。

当前手柄节点不发布 motor current。

当前手柄节点不发布 torque command。

当前手柄节点不验证真实工具轴。

当前手柄节点不知道已校准的 RCM 点。

当前手柄节点不知道已校准的 TCP。

当前手柄节点不执行 workspace limit。

当前手柄节点不执行 collision limit。

当前手柄节点没有足够的 deadman 语义，不能作为实机使能条件。

任何真实硬件命令链路都必须从只读通信检查开始。

任何写控制链路都必须由安全、数学、测试、构建/部署责任人分别审查。

## 1. 当前仓库地图（CURRENT）

仓库根目录按 ROS 2 colcon workspace 使用。

当前顶层 ROS 包位于 `src` 下。

`src/joystick` 是当前唯一具备用户可运行行为的 ROS 包。

`src/agx_arm_description` 包含 Piper URDF 与 mesh 资产。

`src/agx_arm_bringup` 当前存在，但还没有真实 bringup pipeline。

`src/agx_arm_controller` 当前存在，但还没有 RCM controller。

`joystick` 包使用 `ament_cmake`，但安装的是 Python 可执行节点。

`joystick` 包声明了 `rclpy` 与 `sensor_msgs` 依赖。

手柄 Python 代码还导入了 `std_msgs.msg.String` 与 `geometry_msgs.msg.Vector3`。

手柄 launch 文件启动标准 `joy` 包节点。

手柄 launch 文件启动自定义可执行文件 `joystick.py`。

launch 时节点名当前可能显示为 `rcm_gamepad_controller`。

Python 源码级默认节点名是 `gamepad_controller`。

做接口讨论时，以 `gamepad_controller` 作为当前自定义节点契约。

当前 `/joy` 输入类型是 `sensor_msgs/msg/Joy`。

当前 `/rcm_mode` 输出类型是 `std_msgs/msg/String`。

当前 `/rcm_cmd` 输出类型是 `geometry_msgs/msg/Vector3`。

`/rcm_cmd.x` 是累计的 pitch intent。

`/rcm_cmd.y` 是累计的 yaw intent。

`/rcm_cmd.z` 是累计的 insertion intent。

当前 command units 只是手柄累计意图单位，不是已验证的机器人 SI 单位。

当前按钮映射为 A index 0、B index 1、X index 3、Y index 4。

X 按钮把 mode label 切到 `GRAVITY_COMP`。

Y 按钮把 mode label 切到 `IMPEDANCE`。

B 按钮把 mode label 切到 `RCM_CONTROL`。

A 按钮把累计 command value 清零。

默认手柄节点参数 `deadzone` 是 `0.15`。

默认手柄节点参数 `pitch_step` 是 `0.02`。

默认手柄节点参数 `yaw_step` 是 `0.02`。

默认手柄节点参数 `insertion_step` 是 `0.005`。

launch 文件把 `joy_node` deadzone 设为 `0.1`。

launch 文件把 `joy_node` autorepeat rate 设为 `20.0` Hz。

launch 文件把自定义节点 deadzone 设为 `0.15`。

launch 文件把自定义节点 pitch step 设为 `0.02`。

launch 文件把自定义节点 yaw step 设为 `0.02`。

launch 文件把自定义节点 insertion step 设为 `0.005`。

URDF robot name 是 `piper`。

URDF 包含 fixed `world` 到 `base_link` joint。

URDF 包含 `joint1` 到 `joint6` 六个 revolute joints。

当前运动链末端 link 是 `link6`。

mesh 资产包含 STL 和 DAE 文件。

RViz 前应检查 URDF 中 mesh 路径是否仍符合当前 package 路径约定。

## 2. 当前可安全运行的命令（CURRENT）

下面命令只用于仓库级开发、构建、测试和话题观察。

这些命令本身不会向真实机械臂发送运动命令。

从 workspace 根目录运行。

使用已经 source ROS 2 Humble 的终端。

Ubuntu 22.04 + Humble 环境中，先 source `/opt/ros/humble/setup.bash`。

CURRENT 安全构建与测试命令如下。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

上面命令只验证当前 workspace 能否构建和运行已有测试。

它不能证明 Piper 实机可控。

CURRENT 手柄 dry run launch 命令如下。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select joystick
source install/setup.bash
ros2 launch joystick joystick.launch.py
```

上面命令启动手柄输入和意图输出。

它不能作为硬件 enable 条件。

CURRENT 话题检查命令如下。

```bash
source install/setup.bash
ros2 topic list
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
ros2 topic hz /joy
ros2 topic hz /rcm_cmd
```

这些命令只观察 ROS topic。

看见 `/rcm_cmd` 不代表可以接 SDK 写控制。

CURRENT 接口类型检查命令如下。

```bash
ros2 interface show sensor_msgs/msg/Joy
ros2 interface show std_msgs/msg/String
ros2 interface show geometry_msgs/msg/Vector3
```

这些命令确认 message shape。

它们不验证单位、关节顺序、工具坐标或安全状态。

CURRENT 手柄参数检查命令如下。

```bash
ros2 param list
ros2 param get /rcm_gamepad_controller deadzone
ros2 param get /rcm_gamepad_controller pitch_step
ros2 param get /rcm_gamepad_controller yaw_step
ros2 param get /rcm_gamepad_controller insertion_step
```

如果 launch 后节点显示为 `/gamepad_controller`，则把上述节点名替换为 `/gamepad_controller`。

不要从手柄 topic 测试成功推断硬件 ready。

不要把手柄输出直接连接到厂商写控制命令。

除非相关 H gate 已经通过，否则不要从本文档运行任何 CAN 写命令或 SDK 写命令。

## 3. CURRENT / PLANNED / RESEARCH 标签含义

`CURRENT` 用于当前仓库文件已经支撑的命令、观察或静态资产。

`PLANNED` 用于未来源码实现、接口约束、测试策略或 launch 方案。

`RESEARCH` 用于需要单独验证的控制模式、动力学、底层协议或数学方案。

`CURRENT` 工作可以用 `colcon`、`ros2 launch`、`ros2 topic` 工具演示。

`CURRENT` 工作可以包括 URDF 只读检查。

`CURRENT` 工作可以包括 joystick message flow。

`CURRENT` 工作可以包括静态文档审阅。

`CURRENT` 工作可以包括 package discovery。

`CURRENT` 工作可以包括不命令硬件的本地测试。

`PLANNED` 工作必须进入受版本控制的源码后，才能称为已实现。

`PLANNED` 工作必须包含 launch 文件、测试和安全检查后，才允许进入 live use。

`PLANNED` 工作必须明确 command owner 与 command timeout。

`PLANNED` 工作必须明确 mock 与 simulation 行为。

`PLANNED` 工作必须包含 rollback 或 fail-closed 路径。

`RESEARCH` 工作不得拥有 actuator。

`RESEARCH` 工作不得绕过 ROS lifecycle state。

`RESEARCH` 工作不得在没有书面验收标准时使用厂商 low-level motor control。

`RESEARCH` 工作不得被隐藏在用户友好的 mode label 后面。

在厂商文档、命令限制和 HIL 测试证明前，MIT mode 仍属于 `RESEARCH`。

在厂商文档、命令限制和 HIL 测试证明前，torque command mode 仍属于 `RESEARCH`。

在厂商文档、命令限制和 HIL 测试证明前，impedance command mode 仍属于 `RESEARCH`。

在厂商文档、命令限制和 HIL 测试证明前，current command mode 仍属于 `RESEARCH`。

在厂商文档、命令限制和 HIL 测试证明前，gain-level tuning 仍属于 `RESEARCH`。

在厂商文档、命令限制和 HIL 测试证明前，low-level motor-control mode 仍属于 `RESEARCH`。

“低层电机禁用”指 live hardware 上的 low-level motor-control 写路径保持关闭，直到厂商语义、命令限制和分级 HIL/live 证据通过审查。

当前仓库可以用于开发控制栈。

当前仓库不能证明临床安全。

当前仓库不能证明 RCM 精度。

当前仓库不能证明重力补偿稳定性。

当前仓库不能证明关节力矩正确性。

当前仓库不能证明 Piper CAN command 兼容性。

当前仓库不能证明 MoveIt planning 正确性。

当前仓库不能证明 MuJoCo dynamic fidelity。

## 4. 硬件到货与首次检查 SOP（PLANNED）

本 SOP 从真实 Piper 或兼容机械臂到货时开始。

开箱检查时不要给机器人上电。

机械检查时不要连接 CAN。

第一次电气 smoke test 时不要安装工具。

机械臂 enable 前不要运行 joystick launch。

每一步都要记录到 evidence package。

### 4.1 开箱检查

拍摄未开封包装。

拍摄 shock indicator（如有）。

拍摄标签、序列号和型号标识。

拍摄附件、线缆、电源、USB-CAN adapter 和急停硬件。

搬动机械臂前检查是否有松散零件。

检查是否有漏油、变形、划伤、外壳裂纹、连接器弯曲或紧固件缺失。

记录发货日期、收货日期和存放条件。

记录 vendor、distributor、firmware claim 和随附文档版本。

第一次上电区域保持清空。

第一次上电至少指定一名 operator 和一名 observer。

### 4.2 机械检查

核对 base mounting hole 与工作台或 fixture。

通电前确认 base 稳固。

只有厂商手册允许时，才允许无电状态下移动关节。

若关节卡滞、下坠或发出异常声音，立即停止。

检查 link cover 与 cable route。

检查 connector strain relief。

检查 gripper 或 flange accessory 是否安装牢固。

检查工作空间与桌面、屏幕、笔记本和人员位置的关系。

标记物理 keep-out zone。

只有不会卷入机械臂时，才允许放置 soft stop 或泡沫缓冲。

移除工作区内松散工具。

第一次验证不要安装尖锐、过重或手术器械形态的工具。

### 4.3 急停检查

识别厂商急停设备。

确认 observer 可以够到急停按钮。

确认急停按钮具备机械锁止。

按厂商文档确认 reset procedure。

在 ROS 控制未运行时确认断电行为。

不要把软件 stop 当成唯一 stop path。

不要用 joystick button 充当 emergency stop。

把急停测试结果写入 evidence package。

如果急停行为不明确，阻塞所有 powered development。

### 4.4 电源与接地

按厂商电源标签核对输入电压。

按厂商要求核对电流能力。

连接前确认接头极性。

确认电源线有 strain relief。

确认工作台插座接地。

enable drive 前确认机械臂 base 已固定。

实际可行时，靠近电源操作尽量单手完成，另一只手远离工作空间。

上电时让急停保持可触达。

上电过程中监听异常声音。

留意过热或电气故障气味。

出现非预期运动时立即断电。

机械臂报告 uncontrolled error 时立即断电。

### 4.5 工作空间与人员

定义 operator 角色。

定义 observer 角色。

定义 log owner 角色。

非必要人员离开 keep-out zone。

保持明确的断电路径。

笔记本放在机械臂运动扫掠范围外。

CAN 和 USB 线缆不能进入 joint 或 link pinch point。

固定或走线 CAN/USB，避免线缆拉动机械臂。

只有确认机械臂不会落向 operator 时，才使用低桌面。

优先使用 bolted fixture，不优先使用临时夹具。

### 4.6 证据包

证据包必须包含照片。

证据包必须包含 robot serial。

证据包必须包含 firmware version。

证据包必须包含 vendor SDK version、commit 或 release。

证据包必须包含精确 firmware、SDK release/commit 和 API revision 的兼容证据。

证据包必须包含 OS version。

证据包必须包含 ROS 2 version。

证据包必须包含 CAN adapter model。

证据包必须包含 kernel version。

证据包必须包含 SocketCAN interface name。

证据包必须包含 read-only CAN logs。

证据包必须包含 build logs。

证据包必须包含 test logs。

证据包必须包含实际运行过的 exact commands。

RViz 或 simulation 截图只能在对应功能实现后加入。

证据包必须包含 blockers。

## 5. PC、Ubuntu、ROS 2、CAN 与 SocketCAN SOP（PLANNED）

默认目标基线是 Ubuntu 22.04 与 ROS 2 Humble，除非项目 owner 明确更改。

桌面开发优先使用 x86_64。

ARM64 部署必须先确认 dependency matrix。

PCL、OpenCV、Eigen、Pinocchio 进入源码后，要记录版本、ABI 和安装来源。

不要安装来源不明的 binary blob。

ROS 依赖优先使用官方 ROS package repository。

Piper SDK 与 CAN setup 必须以厂商文档为准。

`piper_sdk` 只能在 version、branch/release、CAN transport 都记录后使用。

### 5.1 OS 与 ROS 基线

CURRENT host setup 检查命令如下。

```bash
lsb_release -a
uname -a
printenv ROS_DISTRO
source /opt/ros/humble/setup.bash
ros2 --version
```

这些命令只采集主机环境信息。

它们不会触发机械臂运动。

常见开发依赖安装命令如下。

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions
sudo apt install -y ros-humble-desktop ros-humble-joy
sudo apt install -y can-utils net-tools iproute2 ethtool
sudo apt install -y libeigen3-dev libopencv-dev libpcl-dev
```

`can-utils` 用于 `candump` 等 CAN 诊断。

`ethtool` 用于部分网卡/CAN 设备诊断。

缺少 `ip` 命令时安装 `iproute2`。

不要默认假设 Pinocchio 已安装。

记录 Pinocchio 来自 apt、conda、source build 还是 robotpkg。

### 5.2 用户权限

CAN 和 serial adapter 经常需要 group permission。

改权限前先检查当前用户组。

udev 规则必须在读取 device ID 后再写。

CURRENT 诊断命令如下。

```bash
id
groups
lsusb
ip link show
dmesg --ctime | tail -n 80
```

如果 CAN adapter 显示为 `can0`，先继续只读检查。

如果 CAN adapter 显示为其他名字，记录准确 interface。

如果没有任何 CAN adapter，停止 CAN validation。

### 5.3 SocketCAN 只读准备

下面命令会配置 SocketCAN interface。

它们会影响主机网络接口状态。

只使用厂商指定 bitrate。

只读验证期间不要发送 CAN frame。

示例只读 setup 如下。

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

上面 bitrate 是示例值，必须按 Piper 当前手册或 SDK 文档确认。

只读 CAN 观察命令如下。

```bash
candump -L can0
```

`candump` 只用于观察 traffic。

H0-H6 前不要运行任何会写入 position、velocity、torque、current、mode、enable、zero 或 MIT 的 vendor demo。

H0-H6 前不要运行 low-level motor-control 示例。

如果 `candump` 看不到 frame，不一定是错误；有些机器人可能在被查询前保持 silent。

如果厂商协议需要 request frame 才返回状态，该动作应视为写控制或主动总线交互，必须等待门控批准。

### 5.4 CAN 证据检查表

记录 `ip link show` 输出。

记录 `ip -details link show can0` 输出。

记录准确 bitrate。

记录 adapter model。

记录 kernel driver。

记录 `candump -L can0` timestamp。

记录 robot power 是否开启。

记录 drive 是否 enable。

记录 vendor SDK 是否正在运行。

记录是否发送过任何 write command。

H0-H6 阶段，write command 的期望答案是 no。

## 6. 松灵 / AgileX Piper SDK 使用手册（PLANNED/RESEARCH）

本节是 SDK 使用手册，不代表当前仓库已经集成 SDK。

当前仓库没有 `piper_sdk` 源码。

当前仓库没有 Piper SDK adapter。

当前仓库没有 CAN adapter。

当前仓库没有 hardware interface。

当前仓库没有 MoveIt、MuJoCo 或 RCM solver 实现。

因此，本节所有 SDK 写控制内容都属于 `PLANNED` 或 `RESEARCH`。

H0-H6 前，本节只允许用于安装、版本记录、只读 SocketCAN 检查、只读 SDK 状态读取设计和 adapter 边界设计。

### 6.1 SDK 是什么

松灵 / AgileX 官方 Piper SDK 是用于控制 Piper 机械臂的 Python SDK。

官方仓库地址是 `https://github.com/agilexrobotics/piper_sdk`。

Python 包名通常记录为 `piper_sdk`，PyPI 包名记录为 `piper-sdk`。

PyPI 说明中依赖 `python-can > 3.3.4`。

官方 README 给出的 SDK 使用方式包含 PyPI 安装、源码安装和 release wheel 安装。

官方 release 持续更新接口能力，例如 FK calculation、SDK joint/gripper limit、load setting demo 等。

因此文档、日志和证据包必须记录 SDK version、commit 或 release。

不要在本文档中固定未经验证的函数名。

如果示例提到 `C_PiperInterface` 或 `C_PiperInterface_V2`，只表示官方 README 或 demo 中出现过该接口名。

实际使用前必须以当前安装版本的 README、demo 和 interface 文档为准。

### 6.2 安装方式

安装前先记录 Python、pip、OS、架构和 ROS 环境。

```bash
python3 --version
python3 -m pip --version
uname -m
lsb_release -a
```

这些命令只采集环境，不访问机械臂。

PyPI 安装路径如下。

```bash
python3 -m pip install --upgrade pip
python3 -m pip install python-can
python3 -m pip install piper_sdk
python3 -m pip show piper_sdk
```

如果项目使用虚拟环境，应在虚拟环境内执行并记录 virtualenv 路径。

源码安装路径如下。

```bash
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk
python3 -m pip install .
python3 -m pip show piper_sdk
```

源码安装必须记录 commit hash。

```bash
git rev-parse HEAD
git status --short
```

wheel 安装路径如下。

```bash
python3 -m pip install ./piper_sdk-REPLACE_WITH_VERSION-py3-none-any.whl
python3 -m pip show piper_sdk
```

wheel 安装必须记录 wheel 文件名、下载来源、hash 和下载日期。

升级、查询和卸载命令如下。

```bash
python3 -m pip show piper_sdk
python3 -m pip install --upgrade piper_sdk
python3 -m pip uninstall piper_sdk
```

升级 SDK 前必须保存旧版本证据。

升级后必须重新跑只读连接、joint-order、单位和限位检查。

### 6.3 CAN 模块依赖

官方 README 的 CAN 模块说明依赖 `can-utils` 和 `ethtool`。

缺少 `ip` 命令时安装 `iproute2`。

Ubuntu 22.04 示例安装命令如下。

```bash
sudo apt update
sudo apt install -y can-utils ethtool iproute2
```

这些包只是诊断和配置工具。

安装这些包不代表机器人已经可以控制。

### 6.4 版本记录表

每次 SDK 测试前，先填写下表。

| 字段 | 示例值 | 必填原因 |
| --- | --- | --- |
| robot serial | `PIPER-REPLACE` | 区分机械臂个体 |
| firmware version | `REPLACE` | 判断固件/API 兼容性 |
| SDK version | `pip3 show piper_sdk` 输出 | 判断 Python 包行为 |
| SDK commit | `git rev-parse HEAD` | 源码安装时定位接口 |
| SDK release / wheel | `vX.Y.Z` 或 wheel 文件名 | wheel 安装时定位接口 |
| API 版本 / 接口语义 | README、demo、interface 文档引用 | 避免函数语义漂移 |
| CAN interface | `can0` 或实际名称 | SocketCAN 绑定 |
| CAN bitrate | 厂商文档确认值 | 错误 bitrate 会导致通信失败 |
| USB-CAN adapter | 型号、序列号、driver | x86_64 与 ARM64 行为可能不同 |
| host architecture | `x86_64` / `aarch64` | wheel 和 driver 兼容性 |
| Python version | `python3 --version` | SDK 运行环境 |
| test date | `YYYY-MM-DD` | 证据时效 |
| operator | 操作者姓名 | 责任追踪 |

版本记录必须跟随 evidence package 保存。

只记录 firmware/SDK 不够；API 行为变化同样可能导致错误运动。

### 6.5 CAN 设备识别与只读 bring-up

先确认系统是否看到 CAN interface。

```bash
ip link
ip -details link show can0
```

如果没有 `can0`，记录实际 interface 名称，不要强行假设。

按厂商 bitrate 设置 interface。

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

上面的 `1000000` 只是示例，必须替换为当前 Piper 文档或 SDK demo 要求的 bitrate。

只读观察使用下面命令。

```bash
candump -L can0
```

不要在 H0-H6 前发送任何运动、enable、zero、home、mode、torque、current、impedance、gain 或 MIT 相关命令。

不要把社区示例中的写帧搬到 live arm。

### 6.6 SDK 只读初始化流程

官方 README 的 simple start 是读取关节角。

这类读取流程可以作为 H5 只读 SDK gate 的候选，但必须先完成 SocketCAN evidence。

`C_PiperInterface` 与 `C_PiperInterface_V2` 可接收 CAN interface 名称。

非官方 CAN 设备可能需要额外参数，例如把第二参数设为 `False`，不要默认假设使用内置 CAN。

下面片段只展示“版本确认和只读边界”，不是可直接运行的 live arm 控制脚本。

```python
#!/usr/bin/env python3
"""PLANNED read-only SDK probe skeleton.

请先执行：python3 -m pip show piper_sdk
请阅读当前版本 README、demo 与 interface 文档。
请确认 CAN interface、bitrate、adapter 类型和固件兼容性。
此文件不得包含 enable、zero、home、motion、write、torque、current、MIT 命令。
"""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PiperSdkIdentity:
    sdk_version: str
    sdk_source: str
    can_interface: str
    can_bitrate: int
    adapter_model: str


class PiperReadOnlyProbe:
    def __init__(self, identity: PiperSdkIdentity) -> None:
        self.identity = identity
        self._sdk: Any | None = None

    def connect(self) -> None:
        raise NotImplementedError(
            "Bind this to the read-only constructor from the pinned piper_sdk version. "
            "If C_PiperInterface_V2 is used, verify the current signature first."
        )

    def read_joint_angles(self) -> list[float]:
        raise NotImplementedError(
            "Bind this to the official read-only joint-angle demo for this SDK version."
        )

    def read_status(self) -> dict[str, Any]:
        raise NotImplementedError(
            "Bind this to read-only status/error/version APIs after reading current docs."
        )
```

该 skeleton 的目的不是猜 API，而是强制把 SDK 版本、CAN interface 和只读方法绑定在同一个审查点。

如果当前 SDK demo 使用 `C_PiperInterface_V2("can0", False)` 之类参数形态，必须在 evidence 中说明第二参数的含义和 CAN adapter 类型。

### 6.7 只读 adapter skeleton

ROS 控制核心不应直接依赖 SDK 原始单位或函数名。

建议先建立只读 adapter 边界，输出 joint state、status、error 和 version。

所有单位转换隔离在 adapter 边界。

控制核心只使用 SI 单位：rad、rad/s、m、s、N、Nm。

下面 skeleton 仍然只读，不包含任何 enable 或 write。

```python
#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PiperJointStateSI:
    names: tuple[str, ...]
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    stamp_s: float


@dataclass(frozen=True)
class PiperStatus:
    sdk_version: str
    firmware_version: str
    api_semantics: str
    can_interface: str
    errors: tuple[str, ...]
    warnings: tuple[str, ...]


class PiperSdkReadOnlyAdapter:
    def __init__(self, sdk_impl: Any, joint_order: tuple[str, ...]) -> None:
        self._sdk = sdk_impl
        self._joint_order = joint_order

    def read_joint_state_si(self) -> PiperJointStateSI:
        raise NotImplementedError(
            "Read raw SDK joint angles, verify joint order, convert to rad, and return SI state."
        )

    def read_status(self) -> PiperStatus:
        raise NotImplementedError(
            "Read SDK/firmware/API/status/error fields without sending actuator commands."
        )
```

adapter 单元测试必须覆盖单位转换、关节顺序、缺字段、过期 timestamp 和 SDK 异常。

adapter 出错时返回 fault，不允许上层用 fallback zero 覆盖未知状态。

### 6.8 H0-H6 前禁止事项

H0-H6 前禁止 enable。

H0-H6 前禁止 motion。

H0-H6 前禁止 zero。

H0-H6 前禁止 write。

H0-H6 前禁止 设零 写入固件。

H0-H6 前禁止 回零 运动。

H0-H6 前禁止 torque control。

H0-H6 前禁止 current control。

H0-H6 前禁止 impedance control。

H0-H6 前禁止 gain-level tuning。

H0-H6 前禁止 low-level motor-control。

H0-H6 前禁止 MIT protocol live control。

官方 notes 明确，MIT protocol for controlling individual joints 是高级功能，误用可能损坏机械臂。

H0-H6 前禁止复制官方 demo 或 community adapter 到 live arm 上运行写控制。

H0-H6 前禁止把 joystick `/rcm_cmd` 连接到 SDK command。

H0-H6 前禁止以“只是低速”为理由绕过 command owner、watchdog 或 joint-order check。

### 6.9 H0-H6 后的计划写控制包装

只有 H0-H6 通过后，才能设计 SDK 写控制 wrapper。

写控制 wrapper 必须只有一个 command owner。

写控制 wrapper 必须有 watchdog。

写控制 wrapper 必须有 joint-order check。

写控制 wrapper 必须有 unit conversion check。

写控制 wrapper 必须有 limit clamp 或 explicit rejection。

写控制 wrapper 必须要求 ROS lifecycle active。

写控制 wrapper 必须检查 command timestamp。

写控制 wrapper 必须在 SDK error、CAN error、stale command、limit violation 时 fail-closed。

写控制 wrapper 必须默认禁用 live backend。

写控制 wrapper 必须先通过 mock backend。

写控制 wrapper 必须先通过 replay test。

写控制 wrapper 必须先通过 HIL test。

写控制 wrapper 必须记录每次 enable、disable、fault 和 recovery。

### 6.10 SDK 故障排查

| 现象 | 可能原因 | 只读诊断 | 处理 |
| --- | --- | --- | --- |
| `ip link` 看不到 CAN | adapter 未识别、驱动缺失、线缆问题 | `lsusb`、`dmesg --ctime` | 停止 SDK 测试，先修复 adapter |
| `can0` 存在但 down | interface 未激活 | `ip -details link show can0` | 按厂商 bitrate 激活 |
| `candump` 无 frame | robot silent、接线错误、bitrate 错误 | `candump -L can0`、`ip -details link show can0` | 保持只读，核对 wiring 和 bitrate |
| SDK 无法导入 | Python 环境或安装路径错误 | `python3 -m pip show piper_sdk` | 记录环境，重装或切换 venv |
| SDK 连接失败 | CAN interface 名称错误 | `ip link` | 使用实际 interface 名称 |
| 非官方 CAN 设备异常 | constructor 参数不匹配 | 当前 SDK README/demo | 按当前版本设置参数，例如非内置 CAN 标志 |
| `SendCanMessage(SEND_MESSAGE_FAILED (100017))` | 连接、总线、机械臂状态或 SDK 传输失败 | 保存完整日志 | 检查连接，按官方 notes 断电重启机械臂后再试 |
| 读取角度不合理 | 单位或 joint order 错误 | adapter unit test、joint-order evidence | 阻塞写控制，重新核对 |
| SDK demo 行为与文档不一致 | SDK/API/firmware 不匹配 | 版本记录表 | 固定版本并回到只读验证 |
| ARM64 可安装但运行异常 | wheel、driver 或 ABI 差异 | `uname -m`、pip metadata、driver log | 单独建立 ARM64 证据 |

出现 SDK 错误时，不要用默认零位或上次状态假装读数有效。

出现 `SendCanMessage(SEND_MESSAGE_FAILED (100017))` 时，按官方 notes 检查连接，并断电重启机械臂后再试。

### 6.11 SDK 资料链接与证据来源

官方 SDK 仓库：`https://github.com/agilexrobotics/piper_sdk`。

SDK 安装、demo、interface 语义和 CAN 说明以官方 README、release notes、demo 目录和当前源码为准。

PyPI `piper-sdk` 页面用于记录 Python package dependency，例如 `python-can > 3.3.4`。

SDK release 页面用于判断接口变化，例如 FK calculation、joint/gripper limit、load setting demo 等能力。

证据包中必须保存访问日期、版本号、commit 或 release 标识。

不要把第三方 adapter 的 README 当成 live arm 安全依据。

### 6.12 文档维护约束

本仓库的使用说明集中在 `docs/PIPER_RCM_DEVELOPMENT_GUIDE.md`。

当前任务不修改 `README.md`。

当前任务不新增第二份 docs Markdown。

当前任务不创建文档归档目录。

如果未来需要拆分文档，必须先由项目 owner 明确批准，并更新文档索引和维护规则。


## 7. 手柄接口契约（CURRENT）

当前手柄契约是 intent interface，不是硬件 command interface。

输入 topic 是 `/joy`。

输入 message type 是 `sensor_msgs/msg/Joy`。

输出 mode topic 是 `/rcm_mode`。

输出 mode type 是 `std_msgs/msg/String`。

输出 command topic 是 `/rcm_cmd`。

输出 command type 是 `geometry_msgs/msg/Vector3`。

The custom node source name is `gamepad_controller`.

The launch name may appear as `rcm_gamepad_controller`.

The mode labels are `IDLE`, `GRAVITY_COMP`, `IMPEDANCE`, and `RCM_CONTROL`.

这些 mode label 只表示用户意图。

这些 mode label 不是硬件 command mode。

The mode labels do not enable gravity compensation.

The mode labels do not enable impedance control.

The mode labels do not enable RCM enforcement.

command vector 只是在软件中累计。

The command vector is reset by the A button.

The command vector increments only while mode is `RCM_CONTROL`.

The left stick Y axis changes pitch intent.

The left stick X axis changes yaw intent.

The right stick Y axis changes insertion intent.

No roll intent exists in the current node.

当前节点没有 deadman button。

当前 command message 没有 command timestamp。

No calibration version exists in the current command message.

No tool ID exists in the current command message.

No validity window exists in the current command message.

No source arbitration exists in the current command message.

当前 command message 没有 command owner。

No safety state exists in the current command message.

下一版接口应把 `Vector3` 替换为 typed command message。

下一版接口应包含 stamp、frame、mode、axes、limits、calibration ID 和 validity timeout。

The current launch can be monitored without hardware.

示例监控命令：

```bash
ros2 launch joystick joystick.launch.py
ros2 topic echo /joy
ros2 topic echo /rcm_mode
ros2 topic echo /rcm_cmd
ros2 topic hz /joy
ros2 topic hz /rcm_cmd
```

CURRENT 手柄 dry run 验收标准：

The node starts without Python exceptions.

The `/joy` topic publishes when the gamepad is moved.

The `/rcm_mode` topic changes when A, B, X, or Y is pressed according to current mapping.

The `/rcm_cmd` topic remains zero outside `RCM_CONTROL` except when previously accumulated values persist.

The `/rcm_cmd` topic resets to zero after A is pressed.

The topic rates remain stable enough for operator input inspection.

## 8. 设零、回零、校准与参考坐标系（PLANNED）

设零 means selecting a software or calibration zero reference.

回零 means commanding or moving the mechanism back to a known home pose.

校零 means validating and correcting the relationship between measured encoder state and the intended zero reference.

Mechanical zero is the physical joint or fixture reference defined by hardware design.

Encoder zero is the sensor count reference used by the drive or vendor firmware.

Software zero is the ROS or controller reference used for kinematics and commands.

TCP zero is the tool center point reference relative to the flange or final link.

Functional axis zero is the task axis reference, such as the insertion axis through the tool.

These are different quantities.

Do not overwrite encoder zero when you only need software zero.

Do not call a software reset homing unless the mechanism actually moved to a verified home.

Do not call joystick A button behavior 回零; it only clears accumulated intent.

PLANNED zeroing flow starts with read-only joint state inspection.

PLANNED zeroing flow records the current joint readings.

PLANNED zeroing flow compares readings to vendor home definitions.

PLANNED zeroing flow stores software offsets in a YAML file.

PLANNED zeroing flow never writes motor firmware offsets during early gates.

PLANNED homing flow should use vendor safe-home procedure after H gates pass.

PLANNED calibration flow should include repeated measurements.

PLANNED calibration flow should include operator sign-off.

PLANNED calibration flow should include rollback to previous offsets.

PLANNED calibration flow should include timestamp and calibration version.

PLANNED calibration flow should include arm serial number.

PLANNED calibration flow should include tool serial number.

PLANNED calibration flow should include flange adapter version.

PLANNED calibration flow should include units.

PLANNED calibration flow should include coordinate frames.

PLANNED 示例：不要直接复制到 live arm。

```yaml
calibration_version: "2026-08-05-piper-benchtop-001"
arm_serial: "REPLACE_WITH_ARM_SERIAL"
tool_serial: "REPLACE_WITH_TOOL_SERIAL"
operator: "REPLACE_WITH_OPERATOR"
frames:
  base_frame: "base_link"
  flange_frame: "link6"
  tcp_frame: "tool_tip"
  rcm_frame: "rcm"
joint_software_zero_rad:
  joint1: 0.0
  joint2: 0.0
  joint3: 0.0
  joint4: 0.0
  joint5: 0.0
  joint6: 0.0
tcp_in_flange_m:
  xyz: [0.0, 0.0, 0.120]
  rpy: [0.0, 0.0, 0.0]
tool_axis_in_tcp:
  direction: [0.0, 0.0, 1.0]
rcm_point_in_base_m:
  xyz: [0.300, 0.000, 0.250]
evidence:
  measured_samples: 10
  max_repeatability_error_m: 0.001
  notes: "Dry-run placeholder only."
```

PLANNED 示例：不要直接复制到 live arm。

```python
#!/usr/bin/env python3
from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass
class ZeroOffsets:
    joint_names: list[str]
    offsets_rad: list[float]
    calibration_version: str


def load_zero_offsets(path: Path) -> ZeroOffsets:
    data = yaml.safe_load(path.read_text())
    joint_map = data["joint_software_zero_rad"]
    names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    return ZeroOffsets(
        joint_names=names,
        offsets_rad=[float(joint_map[name]) for name in names],
        calibration_version=str(data["calibration_version"]),
    )


def apply_software_zero(raw_positions_rad: list[float], offsets: ZeroOffsets) -> list[float]:
    if len(raw_positions_rad) != len(offsets.offsets_rad):
        raise ValueError("joint count mismatch")
    return [q - dq for q, dq in zip(raw_positions_rad, offsets.offsets_rad)]


def main() -> None:
    offsets = load_zero_offsets(Path("config/piper_calibration.yaml"))
    print(offsets)


if __name__ == "__main__":
    main()
```

设零/回零/校准验收标准：

The operator can identify which zero is being changed.

The old calibration file is preserved by version control or evidence process.

The new calibration file names the arm and tool.

The transformation chain is explicit.

The live controller rejects commands when calibration is missing.

The live controller rejects commands when calibration version does not match runtime config.

The live controller rejects commands when joint state age exceeds timeout.

## 9. URDF、TF 与 RViz 计划工作（CURRENT/PLANNED）

CURRENT URDF can be inspected as a model asset.

PLANNED RViz work should expose the current model through `robot_state_publisher`.

PLANNED RViz work should add frames for flange, tool0, tool_tip, tool_axis, and rcm only after calibration definitions exist.

PLANNED TF should keep `world` and `base_link` semantics stable.

PLANNED TF should document whether `world` is lab world or robot mounting world.

PLANNED TF should document whether `base_link` is mechanical base or vendor base.

PLANNED TF should publish joint states from mock data before hardware.

PLANNED TF should validate axis signs against visual markers.

PLANNED 示例：不要直接复制到 live arm。

```xml
<!-- URDF/xacro snippet for future tool frames only. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="piper_rcm">
  <xacro:property name="tool_length" value="0.120" />

  <link name="tool0" />
  <joint name="link6_to_tool0" type="fixed">
    <parent link="link6" />
    <child link="tool0" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <link name="tool_tip" />
  <joint name="tool0_to_tool_tip" type="fixed">
    <parent link="tool0" />
    <child link="tool_tip" />
    <origin xyz="0 0 ${tool_length}" rpy="0 0 0" />
  </joint>

  <link name="rcm" />
  <joint name="world_to_rcm" type="fixed">
    <parent link="world" />
    <child link="rcm" />
    <origin xyz="0.30 0.00 0.25" rpy="0 0 0" />
  </joint>
</robot>
```

PLANNED 示例：不要直接复制到 live arm。

```launch.py
# launch.py snippet for planned RViz visualization.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution([
                FindPackageShare("agx_arm_description"),
                "urdf",
                "piper",
                "urdf",
                "piper_description.urdf",
            ]),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", model])}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ])
```

PLANNED RViz 验收标准：

The robot appears with all six arm joints.

The base frame is stable.

The terminal link matches `link6` unless a planned flange frame is added.

The tool axis marker aligns with the intended insertion direction.

The RCM marker is visually distinct.

The TF tree contains no disconnected required frames.

The launch does not require a physical arm.

## 10. ros2_control 计划硬件接口（PLANNED）

`ros2_control` is PLANNED in this repository.

The hardware interface must be the single owner of physical arm commands.

The hardware interface must separate read state from write command.

The hardware interface must support a mock backend before Piper backend.

The hardware interface must enforce lifecycle activation.

The hardware interface must reject write commands before activation.

The hardware interface must reject stale commands.

The hardware interface must reject commands outside position, velocity, and acceleration limits.

The hardware interface must log SDK version, API version, firmware version, and firmware/SDK/API compatibility status.

The hardware interface watchdog must define heartbeat source, timeout threshold, stop/fail-closed action, recovery condition, and required evidence logs before HIL/live use.

Command enable requires joint-order evidence proving URDF, ROS controller, vendor SDK/API, CAN protocol, MuJoCo, and MoveIt mappings agree for `joint1` through `joint6`.

The hardware interface must expose command mode explicitly.

The hardware interface must start with position trajectory control, not torque control.

The hardware interface must integrate `joint_trajectory_controller` only after mock tests pass.

The hardware interface must keep `piper_sdk` transport behind an adapter boundary.

The adapter boundary allows unit tests without CAN hardware.

Target platforms are x86_64 Ubuntu 22.04 and ARM64 Ubuntu 22.04.

Dependency matrix starts with ROS 2 Humble, Eigen, and the vendor SDK.

PCL and OpenCV are not required for the first hardware interface unless perception enters scope.

Pinocchio or KDL are required only when dynamics or kinematics solvers enter scope.

PLANNED 示例：不要直接复制到 live arm。

```cpp
// C++ skeleton for a future ros2_control SystemInterface.
#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace piper_control
{

class PiperSystemHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr size_t kDof = 6;
  std::array<double, kDof> position_rad_{};
  std::array<double, kDof> velocity_rad_s_{};
  std::array<double, kDof> command_position_rad_{};
  bool active_{false};
};

}  // namespace piper_control
```

PLANNED 示例：不要直接复制到 live arm。

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 2.0
```

PLANNED 示例：不要直接复制到 live arm。

```xml
<!-- URDF ros2_control snippet for future hardware plugin. -->
<ros2_control name="PiperSystem" type="system">
  <hardware>
    <plugin>piper_control/PiperSystemHardware</plugin>
    <param name="backend">mock</param>
    <param name="can_interface">can0</param>
    <param name="sdk_version">PIN_THIS_BEFORE_LIVE_USE</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
  <joint name="joint2">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
  <joint name="joint3">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
  <joint name="joint4">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
  <joint name="joint5">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
  <joint name="joint6">
    <command_interface name="position" />
    <state_interface name="position" />
    <state_interface name="velocity" />
  </joint>
</ros2_control>
```

PLANNED 示例：不要直接复制到 live arm。

```launch.py
# launch.py snippet for planned controller manager bringup.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": "REPLACE_WITH_ROBOT_DESCRIPTION"},
                "config/piper_controllers.yaml",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
            output="screen",
        ),
    ])
```

ros2_control 验收标准：

The mock backend builds on x86_64.

The mock backend builds on ARM64.

The mock backend passes controller manager launch tests.

The mock backend exposes six joints.

The mock backend rejects stale commands.

The Piper backend remains disabled by default.

The Piper backend requires explicit parameter `backend:=piper_sdk` or equivalent.

The Piper backend logs read-only connection details before activation.

The Piper backend fails closed on SDK errors.

The Piper backend never writes in `on_init`.

## 11. 二次开发模式（PLANNED）

Secondary development should add small ROS nodes with explicit contracts.

Do not let a convenience node become a hidden command path.

Do not parse joystick strings in multiple places.

Do not duplicate calibration loading in every node.

Create shared message definitions only when the interface stabilizes.

Keep parameters in YAML files.

Keep test fixtures under the package that owns the behavior.

Keep mock backends deterministic.

Keep all real hardware writes behind lifecycle activation.

PLANNED 示例：不要直接复制到 live arm。

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class RcmIntentMonitor(Node):
    def __init__(self):
        super().__init__("rcm_intent_monitor")
        self.mode = "IDLE"
        self.create_subscription(String, "/rcm_mode", self.on_mode, 10)
        self.create_subscription(Vector3, "/rcm_cmd", self.on_cmd, 10)

    def on_mode(self, msg: String) -> None:
        self.mode = msg.data

    def on_cmd(self, msg: Vector3) -> None:
        if self.mode != "RCM_CONTROL":
            return
        self.get_logger().info(
            f"intent pitch={msg.x:.4f} yaw={msg.y:.4f} insertion={msg.z:.4f}"
        )


def main() -> None:
    rclpy.init()
    node = RcmIntentMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

PLANNED 示例：不要直接复制到 live arm。

```cpp
// C++ skeleton for a future controller that consumes typed RCM commands.
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace piper_control
{

class RcmController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
};

}  // namespace piper_control
```

PLANNED 示例：不要直接复制到 live arm。

```yaml
rcm_controller:
  ros__parameters:
    command_topic: "/rcm_cmd_typed"
    calibration_file: "config/piper_calibration.yaml"
    command_timeout_s: 0.10
    max_pitch_rad: 0.35
    max_yaw_rad: 0.35
    max_insertion_m: 0.08
    max_joint_velocity_rad_s: 0.30
    require_deadman: true
    backend: "mock"
```

测试策略：

Unit-test parameter parsing.

Unit-test missing calibration rejection.

Unit-test stale command rejection.

Unit-test invalid mode rejection.

Unit-test axis sign conversion.

Unit-test limit clipping or rejection policy.

Integration-test topic flow with mock nodes.

Launch-test controller manager startup.

Simulation-test a nominal RCM command sequence.

HIL-test only after mock and simulation pass.

## 12. 重力补偿（RESEARCH）

重力补偿 means computing torques or commands that counteract gravity at the current robot configuration.

当前仓库没有实现重力补偿。

手柄 `GRAVITY_COMP` 只是 mode label。

真正的重力补偿需要 dynamic model。

True gravity compensation needs link masses.

True gravity compensation needs center-of-mass locations.

True gravity compensation needs inertia tensors.

True gravity compensation needs motor-side or joint-side torque semantics.

True gravity compensation needs friction treatment.

True gravity compensation needs gear ratio treatment.

True gravity compensation needs gravity vector orientation.

True gravity compensation needs torque limits.

True gravity compensation needs rate limits.

True gravity compensation needs a verified safety stop.

True gravity compensation needs model validation before live use.

Pinocchio route uses URDF plus inertial properties to compute `rnea(q, v=0, a=0)`.

KDL route uses a chain dynamics solver and gravity vector.

Both routes require correct inertial tags.

The current URDF should be audited for inertial completeness before dynamics.

If inertial properties are missing or approximate, gravity compensation remains RESEARCH.

If vendor firmware exposes gravity compensation internally, treat activation as RESEARCH until documented.

Mock tests must run first.

Simulation tests must run second.

HIL tests must run third.

Live arm tests must use reduced torque limits and physical emergency stop.

PLANNED 示例：不要直接复制到 live arm。

```python
#!/usr/bin/env python3
# Pinocchio-style gravity vector skeleton.
import numpy as np


def compute_gravity_torque(model, data, q: np.ndarray) -> np.ndarray:
    if q.shape != (model.nq,):
        raise ValueError("configuration size mismatch")
    v = np.zeros(model.nv)
    a = np.zeros(model.nv)
    # tau = pinocchio.rnea(model, data, q, v, a)
    tau = np.zeros(model.nv)
    return tau


def clamp_torque(tau: np.ndarray, limit: np.ndarray) -> np.ndarray:
    return np.clip(tau, -limit, limit)
```

PLANNED 示例：不要直接复制到 live arm。

```cpp
// C++ skeleton for future gravity compensation service logic.
#include <array>
#include <stdexcept>

namespace piper_control
{

struct GravityCompensationInput
{
  std::array<double, 6> q_rad;
};

struct GravityCompensationOutput
{
  std::array<double, 6> tau_nm;
  bool model_valid;
};

GravityCompensationOutput computeGravityCompensation(
  const GravityCompensationInput & input)
{
  (void) input;
  GravityCompensationOutput output{};
  output.model_valid = false;
  return output;
}

}  // namespace piper_control
```

重力补偿验收标准：

The dynamic model includes mass, CoM, and inertia for every moving link.

The gravity vector sign is verified in simulation.

The torque vector sign is verified against known poses.

The torque vector is bounded.

The controller exits on stale joint state.

The controller exits on missing model.

SDK write failure 时 controller 必须退出或 fail-closed。

The controller exits on emergency stop.

The controller is validated in mock.

The controller is validated in simulation.

The controller is validated in HIL.

The controller is reviewed before live arm activation.

## 13. RCM 控制设计（RESEARCH/PLANNED）

RCM means Remote Center of Motion.

For this project, RCM means the tool axis should pass through a fixed point in space.

The fixed point is usually the trocar or entry point.

The tool axis must be calibrated.

The TCP must be calibrated.

The flange-to-tool transform must be calibrated.

The base-to-RCM point must be calibrated.

RCM applies only to an already calibrated 有效直杆 tool-axis segment.

The RCM point must lie within the allowed range of that effective straight segment and must not be accepted on the 无限延长线 outside the calibrated segment.

Curved, flexible, non-rigid, multi-segment, non-axisymmetric, eccentric-clamp, or complex end-effector tools must not be declared RCM-compliant with a single straight-axis model unless a separate model and validation evidence are approved.

软件 RCM 不是硬件安全屏障; it cannot replace mechanical limits, physical emergency stop, fixture constraints, supervisor ownership, or low-energy HIL/live gates.

The current repository does not implement this solver.

The current `/rcm_cmd` message represents pitch, yaw, and insertion intent only.

A future solver should transform pitch, yaw, and insertion intent into joint targets.

A future solver should enforce the RCM point geometrically.

A future solver should enforce joint limits.

A future solver should enforce velocity limits.

A future solver should enforce acceleration limits.

A future solver should enforce singularity handling.

A future solver should enforce collision constraints when available.

Tool calibration formula:

Let `p_r` be the RCM point in base frame.

Let `p_t(q)` be a point on the tool axis in base frame.

Let `a_t(q)` be the unit tool axis in base frame.

The point-to-line residual is `e = (I - a_t a_t^T) (p_r - p_t)`.

The RCM constraint target is `e = 0`.

The telecentric deviation metric is the residual norm `||e||`.

The residual 阈值 is `rcm_residual_threshold_m`, frozen by safety and math review before HIL/live use.

If `||e||` exceeds `rcm_residual_threshold_m` for `rcm_residual_max_ticks`, the controller enters `HOLD`, stops sending new targets, and logs reason code `RCM_RESIDUAL_THRESHOLD_EXCEEDED`.

If `||e||` keeps an increasing trend for `rcm_residual_divergence_ticks`, the controller enters `HOLD`; if recovery does not reduce residual before timeout, it enters `FAULT`, stops sending new targets, and logs reason code `RCM_RESIDUAL_DIVERGING`.

DLS means damped least squares.

DLS can solve small joint increments from task residuals.

DLS should increase damping near singularities.

QP means quadratic programming.

QP can handle joint limits and inequality constraints directly.

DLS is simpler for early mock validation.

QP is better for bounded live behavior after solver review.

If the DLS/QP solver is infeasible, the Jacobian is singular, constraints conflict, or the residual does not decrease after a bounded step, the controller enters `HOLD` or `FAULT`, stops sending new targets, and records a reason code such as `SOLVER_INFEASIBLE`, `JACOBIAN_SINGULAR`, `CONSTRAINT_CONFLICT`, or `RESIDUAL_NOT_DESCENDING`.

PLANNED 示例：不要直接复制到 live arm。

```python
#!/usr/bin/env python3
import numpy as np


def point_to_line_residual(p_rcm: np.ndarray, p_tool: np.ndarray, axis: np.ndarray) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    projector = np.eye(3) - np.outer(axis, axis)
    return projector @ (p_rcm - p_tool)


def damped_least_squares_step(jacobian: np.ndarray, residual: np.ndarray, damping: float) -> np.ndarray:
    lhs = jacobian @ jacobian.T + (damping ** 2) * np.eye(jacobian.shape[0])
    return jacobian.T @ np.linalg.solve(lhs, residual)


def solve_rcm_increment(jacobian: np.ndarray, residual: np.ndarray) -> np.ndarray:
    damping = 0.05
    dq = damped_least_squares_step(jacobian, residual, damping)
    return np.clip(dq, -0.02, 0.02)
```

RCM state machine:

State `UNCONFIGURED` loads parameters only.

State `CALIBRATION_REQUIRED` waits for valid tool and RCM calibration.

State `READY` accepts commands with hardware disabled or mock backend.

State `ARMED` accepts commands with explicit deadman and live backend.

State `ACTIVE_RCM` solves and sends bounded joint targets.

State `HOLD` holds the last safe target or stops motion.

State `FAULT` rejects all commands until reset.

State transitions must be logged.

State transitions must include reason codes.

State transitions must include timestamps.

RCM acceptance:

The solver drives residual toward zero in simulation.

The solver keeps insertion along the intended tool axis.

The solver rejects missing calibration.

The solver rejects stale `/rcm_cmd`.

The solver rejects mode mismatch.

The solver respects joint limits.

The solver respects velocity limits.

The solver has deterministic behavior in mock replay.

The solver error metric is logged.

The solver supports replay from recorded intent messages.

solver 绝不直接写 `piper_sdk`；它只能通过批准的 command owner 输出。

## 14. MoveIt 与 RViz 计划流程（PLANNED）

MoveIt is PLANNED.

MoveIt should not be treated as a safety layer.

MoveIt can provide planning, visualization, and collision checking.

MoveIt configuration should be generated only after URDF frames are stable.

MoveIt SRDF should include planning groups for the six arm joints.

MoveIt should know the end effector frame.

MoveIt should know the tool frame after calibration.

MoveIt should not command hardware until ros2_control mock validation passes.

MoveIt with `joint_trajectory_controller` should first run against mock hardware.

MoveIt planned files:

`src/agx_arm_moveit_config/config/piper.srdf`

`src/agx_arm_moveit_config/config/kinematics.yaml`

`src/agx_arm_moveit_config/config/joint_limits.yaml`

`src/agx_arm_moveit_config/config/moveit_controllers.yaml`

`src/agx_arm_moveit_config/launch/demo.launch.py`

These files do not exist in CURRENT repository.

PLANNED 示例：不要直接复制到 live arm。

```yaml
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller

  joint_trajectory_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
```

MoveIt 验收标准：

The planning scene loads without missing meshes.

The six-joint planning group exists.

The end effector frame is correct.

The planned trajectory stays within joint limits.

The controller interface is mock during first validation.

The `joint_trajectory_controller` action is available.

RViz displays the model, planned path, and current state.

## 15. MuJoCo 与 Sim2Real 计划流程（PLANNED/RESEARCH）

MuJoCo is PLANNED.

MuJoCo can support simulation-first validation.

MuJoCo must use an MJCF model whose joints match the ROS joint names or has an explicit mapping.

MuJoCo must use consistent units.

MuJoCo must include actuator limits.

MuJoCo must include inertial parameters before dynamics claims.

MuJoCo must include a tool model before RCM simulation claims.

MuJoCo must publish or bridge joint states into ROS for replay.

MuJoCo must consume bounded commands from the same high-level controller used in mock.

Sim2Real must keep the same command contract across mock, simulation, HIL, and live.

Sim2Real must record differences in timing, limits, friction, backlash, and latency.

Planned files:

`src/agx_arm_sim/mujoco/piper.xml`

`src/agx_arm_sim/launch/mujoco_bridge.launch.py`

`src/agx_arm_sim/config/sim2real.yaml`

`src/agx_arm_sim/test/test_mujoco_replay.py`

These files do not exist in CURRENT repository.

PLANNED 示例：不要直接复制到 live arm。

```mjcf
<!-- MJCF snippet for planned simulation work only. -->
<mujoco model="piper_rcm_mock">
  <compiler angle="radian" coordinate="local" />
  <option timestep="0.002" gravity="0 0 -9.81" />
  <worldbody>
    <body name="base_link" pos="0 0 0">
      <body name="link1">
        <joint name="joint1" type="hinge" axis="0 0 1" range="-2.6 2.6" />
        <geom type="capsule" size="0.03 0.10" mass="1.0" />
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="joint1_position" joint="joint1" kp="20" ctrlrange="-2.6 2.6" />
  </actuator>
</mujoco>
```

PLANNED 示例：不要直接复制到 live arm。

```yaml
sim2real:
  joint_name_map:
    mujoco: [joint1, joint2, joint3, joint4, joint5, joint6]
    ros: [joint1, joint2, joint3, joint4, joint5, joint6]
  command_period_s: 0.01
  state_timeout_s: 0.05
  latency_budget_s: 0.02
  backend_order:
    - mock
    - mujoco
    - hil
    - live
```

MuJoCo 验收标准：

The model loads without warnings.

Joint names match mapping.

Joint limits match ROS config.

Actuator limits are conservative.

The bridge publishes `/joint_states` at the expected rate.

The bridge receives commands through the same planned controller interface.

Replay tests produce deterministic results.

RCM residual can be measured in simulation.

## 16. H0-H9 硬件门控（PLANNED）

H0 is documentation and scope gate.

H1 is unboxing and mechanical inspection gate.

H2 is emergency-stop and power safety gate.

H3 is PC, OS, ROS, and dependency gate.

H4 is SocketCAN read-only gate.

H5 is vendor SDK read-only gate.

H6 is mock and simulation control gate.

H7 is HIL low-energy command gate.

H8 is supervised live position command gate.

H9 is extended scenario validation gate.

H0-H6 must pass before any write-control experiment.

H0-H6 must pass before MIT mode.

H0-H6 must pass before torque control.

H0-H6 must pass before impedance control.

H0-H6 must pass before current control.

H0-H6 must pass before gain-level tuning.

H0-H6 must pass before low-level motor-control.

H0-H6 must pass before vendor examples that move the arm.

H0-H6 must pass before `cansend`.

H0-H6 must pass before `piper_sdk` write commands.

H0 acceptance:

The team can state CURRENT, PLANNED, and RESEARCH boundaries.

The team can state what the repository can and cannot do.

The team can state the command owner plan.

H1 acceptance:

The arm has no visible shipping damage.

The serial number is recorded.

The workspace is defined.

H2 acceptance:

Emergency stop is reachable and tested according to vendor guidance.

Power supply is verified.

The arm is mounted or restrained safely.

H3 acceptance:

Ubuntu version is recorded.

ROS 2 version is recorded.

Build dependencies are recorded.

The workspace builds.

The joystick launch runs dry.

H4 acceptance:

CAN adapter is detected.

SocketCAN interface is configured.

`candump` read-only output is captured.

No CAN writes occurred.

H5 acceptance:

Vendor SDK version is pinned.

`piper_sdk` read-only connection behavior is documented.

Firmware version is captured if read-only access exists.

No motion command occurred.

H6 acceptance:

Mock backend tests pass.

Simulation tests pass if simulation exists.

Controller rejects stale commands.

Controller rejects missing calibration.

Controller rejects invalid mode.

H7 acceptance:

HIL fixture is physically constrained.

Command limits are reduced.

Observer owns emergency stop.

Logs prove low-energy behavior.

H8 acceptance:

Position command path is supervised.

Command owner is unique.

Joint limits are enforced.

Velocity limits are enforced.

Emergency stop behavior is verified.

H9 acceptance:

RCM scenarios pass with residual threshold.

Repeated sessions pass.

Calibration reload passes.

Fault injection passes.

Recovery procedure passes.

## 17. 依赖矩阵（x86_64 / ARM64）

目标 x86_64 平台是 Ubuntu 22.04 + ROS 2 Humble。

目标 ARM64 平台默认是 Ubuntu 22.04 + ROS 2 Humble，除非硬件要求其他版本。

核心构建工具是 CMake、colcon、Python 3 和 ament。

Core ROS dependencies are `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `launch`, and `launch_ros` for current joystick work.

Planned control dependencies include `hardware_interface`, `controller_interface`, `controller_manager`, `joint_state_broadcaster`, and `joint_trajectory_controller`.

Planned kinematics dependencies may include KDL, Pinocchio, Eigen, or MoveIt.

Planned perception dependencies may include OpenCV and PCL only when perception scope exists.

Eigen is expected to be architecture-independent through apt or source.

OpenCV package names and ABI must be recorded for ARM64.

PCL package names and ABI must be recorded for ARM64.

Pinocchio installation route must be recorded for both architectures.

MuJoCo availability must be checked separately on ARM64.

部署前必须验证 vendor SDK 的 ARM64 支持。

CAN adapter kernel driver 必须在 x86_64 和 ARM64 两个平台分别验证。

Do not assume x86_64 USB-CAN behavior matches ARM64 behavior.

Do not assume Python wheels are available on ARM64.

Do not assume MuJoCo GPU behavior is available on ARM64.

## 18. 构建与测试目标（CURRENT/PLANNED）

CURRENT 构建目标是构建 workspace 并运行现有 package tests。

CURRENT 手柄目标是 dry-run topic validation。

CURRENT description 目标是静态 URDF inspection。

PLANNED 构建目标是添加 mock-first ros2_control integration。

PLANNED 测试目标是在硬件前验证 command interface。

PLANNED launch objective is to support mock, simulation, HIL, and live profiles with explicit parameters.

RESEARCH 目标是在没有 live actuation 的情况下评估 torque、impedance、gravity compensation 和 RCM algorithm。

最小构建命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

最小 launch 命令：

```bash
source install/setup.bash
ros2 launch joystick joystick.launch.py
```

最小 topic 验证命令：

```bash
ros2 topic list
ros2 topic hz /joy
ros2 topic hz /rcm_cmd
ros2 topic echo /rcm_mode --once
```

最小 CAN 只读命令：

```bash
ip link show
ip -details link show can0
candump -L can0
```

## 19. 故障排查表

| Symptom | Likely Cause | Diagnostic Command | Action |
| --- | --- | --- | --- |
| `colcon build` cannot find ROS packages | ROS environment not sourced | `printenv ROS_DISTRO` | Source `/opt/ros/humble/setup.bash` |
| `ros2 launch joystick joystick.launch.py` fails | Workspace overlay not sourced | `ros2 pkg list | rg joystick` | Source `install/setup.bash` |
| `/joy` is missing | Gamepad or `joy_node` not running | `ros2 topic list` | Check USB gamepad and launch output |
| `/rcm_cmd` is missing | Custom node not running | `ros2 node list` | Check Python executable install |
| `/rcm_mode` never changes | Button map mismatch | `ros2 topic echo /joy` | Record button indices and update planned mapping |
| `/rcm_cmd` changes outside expectation | Mode or axis map confusion | `ros2 topic echo /rcm_mode` | Confirm mode is `RCM_CONTROL` |
| `ros2 param get` fails | Node name differs | `ros2 node list` | Try `/gamepad_controller` or `/rcm_gamepad_controller` |
| CAN interface missing | Adapter not detected | `lsusb` | Check adapter, cable, driver |
| `ip link set can0 up` fails | Wrong bitrate or driver issue | `dmesg --ctime` | Confirm vendor bitrate and kernel driver |
| `candump` shows no frames | Robot silent or bus issue | `ip -details link show can0` | Stay read-only and inspect wiring |
| RViz mesh missing | Package mesh path mismatch | `ros2 run xacro xacro --inorder MODEL` | Fix package paths in planned URDF work |
| MoveIt cannot plan | Config not implemented | `ros2 pkg list | rg moveit` | Build planned MoveIt package first |
| MuJoCo model fails load | MJCF not implemented | `python3 -c 'import mujoco'` | Build planned simulation package first |
| Controller does not activate | Hardware interface missing | `ros2 control list_hardware_interfaces` | Implement mock ros2_control first |
| Gravity compensation unstable | Model or torque sign invalid | Replay known poses | Return to mock and simulation |
| RCM residual grows | Jacobian sign or frame mismatch | Log residual and TF | Re-check tool calibration and frames |
| Live motion unexpected | Command owner or safety fault | Emergency stop | Stop, preserve logs, review H gates |

## 20. 资料链接与参考来源

Use official ROS 2 Humble documentation for launch, parameters, and colcon basics.

Use official ros2_control documentation for `SystemInterface`, controllers, and lifecycle behavior.

Use official MoveIt 2 documentation for configuration generation and planning scene behavior.

Use official MuJoCo documentation for MJCF modeling and actuator definitions.

Use Linux kernel and can-utils documentation for SocketCAN behavior.

Use vendor Piper documentation for CAN bitrate, frame IDs, SDK use, firmware compatibility, and emergency stop behavior.

Use `piper_sdk` only from a pinned vendor-approved source.

Use Pinocchio documentation for rigid-body dynamics and `rnea` usage.

Use Orocos KDL documentation for kinematics and dynamics alternatives.

Record every external link with access date in the evidence package.

Do not treat tutorial snippets as vendor safety approval.

Do not treat community examples as live-control validation.

Do not use third-party Piper adapters on a live arm without source audit and HIL validation.

参考资料检查清单：

- ROS 2 Humble installation and tutorials.
- colcon documentation.
- ros2_control documentation.
- joint_trajectory_controller documentation.
- MoveIt 2 documentation.
- MuJoCo documentation.
- SocketCAN and can-utils documentation.
- Piper vendor manual.
- Piper SDK or `piper_sdk` source documentation.
- Pinocchio documentation.
- Orocos KDL documentation.

## 21. 实用开发顺序

从 CURRENT dry-run 工作开始。

构建 workspace。

启动 joystick。

检查 `/joy`。

检查 `/rcm_mode`。

检查 `/rcm_cmd`。

记录真实 gamepad mapping。

检查 URDF frames。

Add RViz model launch as PLANNED source work.

Add calibration YAML loader as PLANNED source work.

Add typed RCM command message as PLANNED source work.

Add mock ros2_control hardware as PLANNED source work.

Add controller manager launch as PLANNED source work.

Add `joint_trajectory_controller` mock validation.

Add RCM solver unit tests.

Add RCM replay tests.

Add MuJoCo model after URDF and inertials are settled.

Add MoveIt config after frames and joint limits are settled.

Add SocketCAN read-only evidence after hardware arrives.

Add vendor SDK read-only evidence after SocketCAN is stable.

Add HIL only after mock and simulation pass.

Add live position control only after H7 passes.

Keep gravity compensation in RESEARCH until model and safety evidence pass.

Keep MIT mode in RESEARCH until vendor semantics, limits, and HIL evidence pass.

Keep torque control in RESEARCH until vendor semantics, limits, and HIL evidence pass.

Keep impedance control in RESEARCH until vendor semantics, limits, and HIL evidence pass.

Keep current control in RESEARCH until vendor semantics, limits, and HIL evidence pass.

Keep gain-level tuning in RESEARCH until vendor semantics, limits, and HIL evidence pass.

Keep low-level motor-control in RESEARCH until vendor semantics, limits, and HIL evidence pass.

## 22. 本指南完成定义

The guide names CURRENT repository behavior.

The guide names PLANNED implementation work.

The guide names RESEARCH-only control concepts.

The guide includes complete hardware arrival SOP.

The guide includes Ubuntu, ROS 2, CAN, and SocketCAN setup.

The guide includes read-only `candump` validation.

The guide includes current colcon and joystick commands.

The guide includes current joystick interface details.

The guide includes 设零, 回零, and 校零 distinctions.

The guide includes YAML and Python examples for calibration planning.

The guide includes URDF/xacro and RViz planned examples.

The guide includes ros2_control C++, YAML, and launch planned examples.

The guide includes secondary development Python and C++ skeletons.

The guide includes gravity compensation principles and skeletons.

The guide includes RCM formulas, DLS/QP discussion, solver skeleton, state machine, and acceptance.

The guide includes MoveIt planned files and flow.

The guide includes MuJoCo planned files, flow, and MJCF snippet.

The guide includes H0-H9 gates.

The guide keeps MIT, torque, impedance, current, gain-level, and low-level motor-control disabled through H0-H6.

The guide includes troubleshooting and reference sources.

The guide does not claim unavailable source code exists.

The guide does not authorize live control.

The guide should be updated when actual source packages implement PLANNED items.

## 最终补充：当前仓库、二次开发、低层电机禁用、故障排查与资料链接核对

本节用于把操作手册中的关键边界再次集中说明，避免读者只阅读单个代码块后误操作实机。

### 当前仓库能做什么

`CURRENT`：当前仓库可以作为 ROS 2 workspace 构建，可以查看 Piper URDF/mesh 资源，可以运行 `joystick` 包的 `gamepad_controller` 节点，并通过 `/joy` 生成 `/rcm_mode` 与 `/rcm_cmd` 的上层意图消息。当前仓库没有实机 Piper 硬件驱动、没有 validated `piper_sdk` 写控制、没有 ros2_control hardware interface、没有 MoveIt 执行链、没有 MuJoCo bridge、没有重力补偿实机控制器、没有闭环 RCM controller。

### 二次开发边界

`PLANNED`：二次开发应先在 mock、仿真、HIL、只读 CAN 证据链上完成接口收敛，再进入低速实机。任何新增节点、controller、SDK adapter、MoveIt bridge、MuJoCo bridge、重力补偿模块或 RCM solver 都必须保留 CURRENT/PLANNED/RESEARCH 标注，不能把示例代码直接当成 live arm 指令。

### RCM 工具轴有效段与不规则工具限制

`PLANNED`：本文的 RCM 模型只适用于已标定的刚性工具功能轴，并且 RCM 点必须落在有效直杆工具轴段的允许范围内。有效直杆段需要在工具 manifest 中记录 `lambda_min`、`lambda_max`、标定数据来源、拟合残差和版本号。RCM 点不能只落在数学上的无限延长线；如果 RCM 点在无限延长线但不在真实可通过的有效直杆段内，必须判定为不合格。

远心点偏差指标定义为残差范数 `||e||`，其中 `e` 是 RCM 点到工具功能轴的垂直残差。阈值必须由项目风险评审和实测冻结；持续超限、残差增长趋势、DLS/QP solver 不可行、雅可比奇异、约束冲突或残差不下降时，控制状态必须进入 `HOLD` 或 `FAULT`，停止发送新目标，并记录 reason code、关节状态、输入命令、solver 状态和时间戳。

弯曲工具、柔性工具、非刚性工具、多段工具、非轴对称工具、偏心夹具或复杂末端几何，不得仅用单一直线轴 RCM 模型声明合格。除非另行建立几何/柔性/接触模型并通过 mock、仿真、HIL 和低能量 live 门控验证，否则这些工具只能标为 `RESEARCH`。

软件 RCM 不是硬件安全屏障，不能替代机械限位、物理急停、夹具约束、监督员、低速单关节验证、watchdog 和低能量 HIL/live 门控。软件 RCM 只是一层控制目标或约束，任何证据缺失都必须 fail closed。

### 低层电机禁用与 H0-H6 门控

`RESEARCH`：MIT、torque、impedance、current、gain-level、low-level motor-control 等低层电机禁用策略在 H0-H6 完成前一律生效。H0-H6 证据完成前，禁止把 `piper_sdk`、`piper_ros`、community adapter 或 MIT-mode script 的示例命令复制到 live arm 上运行。

在任何多关节 HIL/live 运动前，必须先完成“低速单关节 only”门控：一次只允许一个关节动作；最大速度、最大步长、最大持续时间和停止距离由项目风险评审冻结；观察员和 e-stop 所有人必须明确；日志必须记录目标关节、方向、速度、步长、反馈、停止方式和异常码。

watchdog 必须有明确 heartbeat source、timeout threshold、stop/fail-closed action、recovery condition 和 evidence logs。控制节点卡死、SDK 阻塞、topic 静默、CAN 异常或 command timestamp 过期时，必须停止发送新目标，并进入 `HOLD` 或 `FAULT`。

joint-order 核对必须覆盖 URDF、ROS controller、vendor SDK/API、CAN protocol、MuJoCo 和 MoveIt。只有 joint name、index、方向、单位、限位和零位定义全部一致后，才能进入 command enable。

firmware/SDK/API 版本兼容证据必须记录 robot serial、firmware version、SDK commit/tag、API method semantics、CAN bitrate、joint order、unit scaling 和测试日期。只记录 firmware/SDK 不够；API 行为变化同样可能导致错误运动。

### 故障排查与资料链接核对

故障排查必须优先按 fail-closed 处理：看不到 `/joy` 就不测试上层意图；看不到 `/rcm_cmd` 就不接控制器；看不到 CAN 只读反馈就不发任何 SDK 写命令；joint-order 未核对就不允许 command enable；watchdog 未实测就不允许持续运动；RCM 残差无法下降就进入 `HOLD` 或 `FAULT`。

资料链接应优先使用官方或一手资料：AgileX/Piper SDK、ROS 2 Humble、URDF、tf2、robot_state_publisher、ros2_control、joint_trajectory_controller、MoveIt 2、MoveIt Servo、MuJoCo、SocketCAN、Pinocchio 和 KDL。第三方文章只能作为辅助理解，不能作为 live arm 安全依据。

PLANNED 示例：不要直接复制到 live arm。

```launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": "<planned xacro output>"}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", "config/piper_view.rviz"],
        ),
    ])
```

PLANNED 示例：不要直接复制到 live arm。

```mjcf
<mujoco model="piper_planned">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base_link">
      <body name="link1">
        <joint name="joint1" type="hinge" axis="0 0 1" limited="true" range="-2.6 2.6"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="joint1_position" joint="joint1" kp="20"/>
  </actuator>
</mujoco>
```

## 如何使用这份文档（中文索引）

新手阅读顺序：先看“中文阅读入口 / 中文版本说明 / 松灵 SDK 使用总览”、第 0 章“范围与安全契约”、第 1 章“当前仓库地图（CURRENT）”、第 2 章“当前可安全运行的命令（CURRENT）”和第 3 章“CURRENT / PLANNED / RESEARCH 标签含义”。这些章节只说明当前仓库能做什么、不能做什么，以及哪些命令属于 dry run。

SDK 与 CAN 阅读顺序：先看第 5 章“PC、Ubuntu、ROS 2、CAN 与 SocketCAN SOP（PLANNED）”、第 6 章“松灵 / AgileX Piper SDK 使用手册（PLANNED/RESEARCH）”，再看下面的“松灵 / AgileX Piper SDK 详细使用补充（中文）”。这些内容只用于安装核对、只读验证、adapter 设计和未来实现规划，不表示当前仓库已经具备 live arm 控制能力。

RCM、重力补偿和仿真阅读顺序：先看第 12 章“重力补偿（RESEARCH）”、第 13 章“RCM 控制设计（RESEARCH/PLANNED）”、第 15 章“MuJoCo 与 Sim2Real 计划流程（PLANNED/RESEARCH）”和第 16 章“H0-H9 硬件门控（PLANNED）”。任何 RCM、重力补偿、MIT 单关节、torque、impedance、current 或 low-level motor-control 内容，在 H0-H6 前都不能用于真实机械臂写控制。

状态标签中文名：当前可用（CURRENT）只表示当前仓库文件已经支持的 dry-run、构建、观察或静态检查；计划实现（PLANNED）只表示未来需要落地到源码、launch、测试和审查的工程工作；研究验证（RESEARCH）只表示可用于概念、数学、仿真、证据规划或只读实验，不允许直接驱动 live arm。

## 松灵 / AgileX Piper SDK 详细使用补充（中文）

本补充是 SDK 使用和接入规划清单，不是当前仓库 live arm 控制教程。当前仓库没有 SDK 集成、没有 SDK adapter、没有 CAN adapter、没有 ros2_control hardware interface、没有 `joint_trajectory_controller` wiring，也没有经过验证的 Piper 实机写控制链路；所有 SDK 写控制内容在本文中均属于计划实现（PLANNED）或研究验证（RESEARCH）。

官方来源：松灵 / AgileX Piper SDK 官方仓库为 `https://github.com/agilexrobotics/piper_sdk`。安装、demo、函数名、构造参数、CAN interface 参数、firmware/SDK/API 兼容性和故障排查语义都必须以当前安装版本的官方 README、release notes、demo 目录和源码为准。

### SDK 安装与版本管理

先安装 Python CAN 依赖，再安装 SDK：

```bash
pip3 install python-can
pip3 install piper_sdk
pip3 show piper_sdk
```

源码安装流程：

```bash
git clone https://github.com/agilexrobotics/piper_sdk.git
cd piper_sdk
pip3 install .
pip3 show piper_sdk
```

wheel 安装流程：

```bash
pip3 install ./piper_sdk-REPLACE_WITH_VERSION-py3-none-any.whl
pip3 show piper_sdk
```

常用维护命令：

```bash
pip3 show piper_sdk
pip3 uninstall piper_sdk
pip3 install --upgrade piper_sdk
```

升级或卸载前必须记录旧版本：`pip3 show piper_sdk` 输出、SDK commit/tag、wheel 文件名、Python 版本、操作系统、CAN adapter 型号、机械臂序列号、firmware version 和测试日期。不要只记录“已安装 SDK”，因为 API 语义、单位、joint order 或 demo 行为变化都可能导致错误运动。

### CAN 依赖与 PC/CAN 只读流程

系统依赖建议先安装 `can-utils`、`ethtool` 和 `iproute2`：

```bash
sudo apt update
sudo apt install can-utils ethtool iproute2
```

PC/CAN 只读流程必须先识别 CAN 设备，再配置 SocketCAN，并只做监听：

```bash
ip link
sudo ip link set can0 down
sudo ip link set can0 type can bitrate REPLACE_WITH_VENDOR_BITRATE
sudo ip link set can0 up
ip -details link show can0
candump -L can0
```

`can0` 和 bitrate 都是示例值，必须替换为当前机器实际 interface 和当前 Piper 文档或 SDK demo 要求的参数。非官方 CAN 设备不可假设默认参数；官方 README 提到非官方 CAN 设备需要设置 CAN interface 参数，因此 SDK 初始化时必须把 CAN adapter 类型、interface 名称和相关构造参数写入 evidence。

只读流程通过前禁止运行任何 SDK 写控制 demo。`candump -L can0` 只能证明总线上可观察到报文或无报文，不能证明 joint order、单位、限位、零位或写控制安全。

### SDK 初始化与只读状态读取

SDK 初始化必须只读优先。官方 SDK 中可能存在 `C_PiperInterface` 或 `C_PiperInterface_V2` 这类接口名，但本文不把这些函数名写成当前仓库能力；实际代码必须以安装版本 demo/API 为准，并在 evidence 中记录确切 import 路径、构造参数、CAN interface 参数、只读状态读取函数、返回单位和异常行为。

只读 probe 的最小验收内容：SDK 能导入；能打印 SDK 版本或包信息；能绑定实际 CAN interface；能读取状态或明确返回只读失败；失败时保留完整 traceback、CAN 状态、`ip -details link show can0` 输出和 `candump -L can0` 日志。任何 SDK 读取异常都不能用默认零位、上次状态或全零数组替代。

`SendCanMessage(SEND_MESSAGE_FAILED (100017))` 故障排查：先停止 SDK 测试，检查电源、线缆、CAN adapter、interface 名称、bitrate、终端电阻、机械臂状态和 SDK 构造参数；必要时按官方 notes 断电重启机械臂后再试。重试前必须保存失败日志，不能在不明原因下连续发写控制命令。

### SDK Adapter 规划

SDK adapter 的第一阶段只能是 read-only adapter。它应把 SDK 原始状态转换为 ROS 控制层需要的 SI 单位，显式验证 unit conversion、joint order、joint name、方向、零位、限位、timestamp、firmware/SDK/API evidence 和错误码。adapter 必须记录 logs，并在 SDK 异常、CAN 异常、缺字段、过期 timestamp、joint order 未确认或单位不明确时 fail-closed。

adapter 不能让上层 controller 直接调用 `piper_sdk` 原始写函数。上层只能看到稳定的状态结构、明确的错误状态、只读诊断和未来经过审查的 command owner 接口。任何新增 SDK adapter 代码都必须带 unit test、mock SDK、异常路径测试、单位转换测试、joint order 测试和 logs 样例。

### H0-H6 后的 planned 写控制

H0-H6 通过后才允许规划 SDK 写控制。写控制必须只有 single command owner，不能让 joystick、RCM solver、MoveIt bridge、MuJoCo bridge 或脚本同时拥有 actuator。command owner 必须要求 lifecycle active、fresh timestamp、watchdog heartbeat、limit clamp、joint-order evidence、unit evidence、firmware/SDK/API evidence 和 operator enable。

写控制 wrapper 必须在 stale command、limit violation、SDK error、CAN error、watchdog timeout、lifecycle inactive、timestamp 过期或 command owner 冲突时进入 `HOLD` 或 `FAULT`。恢复只能通过显式 operator action 和新的 evidence 记录完成，不能自动恢复持续运动。

MIT 单关节控制是高级危险功能，误用可能损坏机械臂。H0-H6 前禁用 MIT 单关节、torque、impedance、current、gain-level、low-level motor-control 和任何绕过上层限位的直接电机命令。即使 H0-H6 后评估 MIT 单关节，也必须一次只允许一个关节、低速、短时、限幅、有人值守、e-stop 就绪，并先在 mock/HIL 中通过。

### 禁止事项清单

禁止把本文 PLANNED/RESEARCH 示例复制到上电机械臂环境直接运行。禁止把 `/rcm_cmd`、joystick intent、MoveIt 目标、MuJoCo 输出或 RCM solver 输出直接接到 SDK 写函数。禁止在 H0-H6 前发送 SDK 写控制。禁止在 CAN 只读反馈不可解释时发命令。禁止在 joint order、单位、零位、方向、限位、firmware/SDK/API 证据不完整时 command enable。禁止把 SDK 错误吞掉后继续运动。禁止在 watchdog 未实测时做持续运动。禁止把 `C_PiperInterface`、`C_PiperInterface_V2` 或任何 demo 函数名当成当前仓库已实现能力。禁止把非官方 CAN 设备当成默认官方设备使用。禁止用默认零位或硬编码全零状态掩盖 SDK 读取失败。
