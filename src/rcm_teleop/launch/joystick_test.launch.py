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
            # 你可以在这里加参数，例如指定特定的手柄设备
            # parameters=[{'device_name': '/dev/input/js0'}]
        ),

        # 2. 启动你自定义的 Python 测试节点 (订阅 /joy 话题，打印数据)
        Node(
            package='rcm_teleop',
            executable='joystick_test.py',
            name='gamepad_controller',
            output='screen'
        )
    ])
