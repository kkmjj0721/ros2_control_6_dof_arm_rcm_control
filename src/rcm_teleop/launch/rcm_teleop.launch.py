from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def input_source_is(source_name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('input_source'), "' == '", source_name, "'"
    ]))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_source',
            default_value='gamepad',
            choices=['gamepad', 'keyboard'],
            description='Select teleoperation input source: gamepad or keyboard.'
        ),

        # 1. Gamepad source: official joy node reads hardware and publishes /joy.
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=input_source_is('gamepad'),
            parameters=[{
                'deadzone': 0.1,  # joy节点自带的死区参数
                'autorepeat_rate': 20.0  # 设置发送频率为20Hz，保证连续累加平滑
            }]
        ),

        # 2. Keyboard source: convert key presses into the same /joy layout.
        Node(
            package='rcm_teleop',
            executable='keyboard_to_joy.py',
            name='keyboard_to_joy',
            output='screen',
            emulate_tty=True,
            condition=input_source_is('keyboard'),
            parameters=[{
                'publish_rate': 20.0,
                'axis_value': 1.0,
                'key_hold_sec': 0.15
            }]
        ),

        # 3. Parse /joy into RCM mode and dry-run command topics.
        Node(
            package='rcm_teleop',
            executable='rcm_teleop.py',
            name='rcm_input_controller',
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
