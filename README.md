# ros2\_control\_6\_dof\_arm\_rcm\_control

- 本项目旨在基于 `ros2_control` 框架，为六轴机械臂（如 AgileX 系列）开发并实现具备柔顺交互能力的 RCM（Remote Center of Motion，远程运动中心）控制算法

# 📝 项目介绍

- **核心目标**：基于 `ros2_control` 硬件抽象层开发，实现高实时性的 RCM 约束控制算法；

- **拖动示教与标定（重力补偿）**：引入重力补偿模式，精准抵消机械臂自重。允许用户通过直接拖动机械臂，在物理空间中直观、无阻力地进行不动点（RCM约束点）的末端坐标标记；

- **柔顺交互（阻抗控制）**：结合阻抗控制算法，使机械臂在运动和受力时具备弹簧阻尼特性，极大提高了与外界环境（或医护人员）交互的安全性与鲁棒性；

- **遥操作 RCM 控制**：支持使用手柄（Joy）遥操作，对机械臂末端工具杆进行姿态调整，同时严格保证工具杆轴线穿过标定的 RCM 不动点；

- **虚实结合（Sim2Real）**：提供基于 MuJoCo 的高保真物理仿真环境，实现从算法仿真到真机部署的代码“零修改”平滑迁移。

# 📚 入门指南

### 💻 开发环境与依赖 

- 我们在以下环境中进行测试代码：

    - **OS**: Ubuntu 22\.04

    - **ROS 2**: Humble

    - **Python**: 3\.10

    - **Simulation**: Mujoco 3\.3\.7

    - **Motion Planning**: Moveit2

    - **Dynamics Library**: Pinocchio

### 🚀 编译与安装 \(Build and source\)

```Python
colcon build
source install/setup.bash
```

### 📖 使用教程 \(Tutorial\)

- To display the robot model in RViz:

    ```Python
    
    ```

- To display the robot model in Mujoco:

    ```Python
    
    ```

# 🎬 演示视频

- 

# 🔗 参考项目与致谢

- [AgileX Arm URDF](https://github.com/agilexrobotics/agx_arm_urdf) \- 松灵六轴机械臂 URDF 模型文件

- [AgileX pyAgxArm SDK](https://github.com/agilexrobotics/pyAgxArm) \- 松灵机械臂 Python SDK

- 



