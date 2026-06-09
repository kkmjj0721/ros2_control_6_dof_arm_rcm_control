import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动官方的 joy 节点 (读取硬件，发布 /joy 话题)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.1,  # joy节点自带的死区参数
                'autorepeat_rate': 20.0 # 设置发送频率为20Hz，保证连续累加平滑
            }]
        ),
        
        # 2. 启动自定义的 RCM 摇杆解算节点
        Node(
            package='joystick',
            executable='joystick.py',  # 对应 CMakeLists 中 install 的名字
            name='rcm_gamepad_controller',
            output='screen',
            parameters=[{
                # 可以在这里覆盖 python 代码中的默认参数，方便后期调参
                'deadzone': 0.15,
                'pitch_step': 0.02,
                'yaw_step': 0.02,
                'insertion_step': 0.005
            }]
        )
    ])