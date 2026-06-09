#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')
        # 订阅 /joy 话题
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.get_logger().info("手柄控制节点已启动，正在监听 /joy...")

    def joy_callback(self, msg):
        self.get_logger().info(f'收到手柄数据 -> 摇杆: {msg.axes}, 按键: {msg.buttons}')

def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已手动停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()