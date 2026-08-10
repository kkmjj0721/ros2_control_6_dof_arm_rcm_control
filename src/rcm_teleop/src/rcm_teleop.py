#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class RCMTeleopController(Node):
    def __init__(self):
        super().__init__('rcm_input_controller')

        # --- 手柄按键索引映射 (Button Mapping) ---
        self.btn_idx_a = 0
        self.btn_idx_b = 1
        self.btn_idx_x = 3
        self.btn_idx_y = 4

        # --- 声明 ROS 2 参数 ---
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('pitch_step', 0.02)
        self.declare_parameter('yaw_step', 0.02)
        self.declare_parameter('insertion_step', 0.005)

        # --- 话题订阅与发布 ---
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.mode_pub = self.create_publisher(String, 'rcm_mode', 10)
        self.cmd_pub = self.create_publisher(Vector3, 'rcm_cmd', 10)

        # --- 运动解算状态变量 ---
        self.pitch = 0.0
        self.yaw = 0.0
        self.insertion = 0.0

        self.current_mode = "IDLE"
        self.last_buttons = []

        self.get_logger().info("✅ RCM 输入解算节点已启动！")
        self.get_logger().info(
            "按键提示: [X]重力补偿 | [Y]标定RCM点 | "
            "[B]RCM遥操作 | [A]指令清零归位"
        )

    def joy_callback(self, msg):
        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)

        # 越界保护（防止手柄断开或数据异常报错）
        max_idx = max(self.btn_idx_a, self.btn_idx_b, self.btn_idx_x, self.btn_idx_y)
        if len(msg.buttons) <= max_idx:
            return

        deadzone = self.get_parameter('deadzone').value

        # 单步运动步长
        pitch_step = self.get_parameter('pitch_step').value
        yaw_step = self.get_parameter('yaw_step').value
        insertion_step = self.get_parameter('insertion_step').value

        # ==========================================
        # 1. 模式切换解算 (根据上面的自定义索引匹配)
        # ==========================================
        btn_a = msg.buttons[self.btn_idx_a]
        btn_b = msg.buttons[self.btn_idx_b]
        btn_x = msg.buttons[self.btn_idx_x]
        btn_y = msg.buttons[self.btn_idx_y]

        last_a = self.last_buttons[self.btn_idx_a]
        last_b = self.last_buttons[self.btn_idx_b]
        last_x = self.last_buttons[self.btn_idx_x]
        last_y = self.last_buttons[self.btn_idx_y]

        mode_changed = False

        # 按键 X (标定/重力补偿模式)
        if btn_x == 1 and last_x == 0:
            self.current_mode = "GRAVITY_COMP"
            mode_changed = True

        # 按键 Y (在重力补偿模式下标定 RCM 点，保持当前模式)
        if btn_y == 1 and last_y == 0:
            if self.current_mode == "GRAVITY_COMP":
                calibrate_msg = String()
                calibrate_msg.data = "CALIBRATE_RCM"
                self.mode_pub.publish(calibrate_msg)
                self.get_logger().info("📍 RCM 点标定请求已发送，按 B 进入 RCM 控制模式")
            else:
                self.get_logger().warn("请先按 X 进入重力补偿模式，再按 Y 标定 RCM 点")

        # 按键 B (RCM控制模式)
        if btn_b == 1 and last_b == 0:
            self.current_mode = "RCM_CONTROL"
            mode_changed = True

        # 按键 A (指令归零)
        if btn_a == 1 and last_a == 0:
            self.pitch = 0.0
            self.yaw = 0.0
            self.insertion = 0.0
            self.get_logger().info("🔄 累加指令已清零归位！")

        self.last_buttons = list(msg.buttons)

        if mode_changed:
            self.get_logger().info(f"🌟 模式切换为: {self.current_mode}")

        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        # ==========================================
        # 2. 连续运动指令解算 (摇杆死区过滤 + 累加)
        # ==========================================
        left_stick_y = msg.axes[1]
        left_stick_x = msg.axes[0]
        right_stick_y = msg.axes[3]

        if self.current_mode == "RCM_CONTROL":
            if abs(left_stick_y) > deadzone:
                self.pitch += left_stick_y * pitch_step

            if abs(left_stick_x) > deadzone:
                self.yaw += left_stick_x * yaw_step

            if abs(right_stick_y) > deadzone:
                self.insertion += right_stick_y * insertion_step

        cmd_msg = Vector3()
        cmd_msg.x = self.pitch
        cmd_msg.y = self.yaw
        cmd_msg.z = self.insertion
        self.cmd_pub.publish(cmd_msg)

        # 降低日志刷屏频率，可以仅在数值变动时才打印，或者注释掉这一句
        self.get_logger().info(
            f"[状态] 模式: {self.current_mode} | "
            f"Pitch: {self.pitch:.3f} | "
            f"Yaw: {self.yaw:.3f} | "
            f"Insert: {self.insertion:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RCMTeleopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
