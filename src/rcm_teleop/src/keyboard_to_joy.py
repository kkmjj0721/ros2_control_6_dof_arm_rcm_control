#!/usr/bin/env python3
import os
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class KeyboardToJoy(Node):
    AXIS_INDEX = {
        'yaw': 0,
        'pitch': 1,
        'insertion': 3,
    }

    BUTTON_INDEX = {
        'reset': 0,
        'rcm_control': 1,
        'gravity_comp': 3,
        'calibrate_rcm': 4,
    }

    AXIS_KEYS = {
        'w': ('pitch', 1.0),
        's': ('pitch', -1.0),
        'a': ('yaw', -1.0),
        'd': ('yaw', 1.0),
        'q': ('insertion', 1.0),
        'e': ('insertion', -1.0),
    }

    BUTTON_KEYS = {
        'r': BUTTON_INDEX['reset'],
        'b': BUTTON_INDEX['rcm_control'],
        'g': BUTTON_INDEX['gravity_comp'],
        'c': BUTTON_INDEX['calibrate_rcm'],
    }

    def __init__(self):
        super().__init__('keyboard_to_joy')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('axis_value', 1.0)
        self.declare_parameter('key_hold_sec', 0.15)

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.axis_directions = {axis: 0.0 for axis in self.AXIS_INDEX}
        self.axis_deadlines = {axis: 0.0 for axis in self.AXIS_INDEX}
        self.button_pulses = set()
        self.tty_fd = None
        self.original_terminal_settings = None

        self._open_keyboard()

        publish_rate = float(self.get_parameter('publish_rate').value)
        timer_period = 1.0 / max(publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'Keyboard source ready: W/S pitch, A/D yaw, Q/E insertion, '
            'G gravity, C calibrate, B RCM, R reset.'
        )

    def _open_keyboard(self):
        try:
            self.tty_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.original_terminal_settings = termios.tcgetattr(self.tty_fd)
            tty.setcbreak(self.tty_fd)
        except OSError as exc:
            self.tty_fd = None
            self.get_logger().error(f'Unable to open /dev/tty for keyboard input: {exc}')

    def close(self):
        if self.tty_fd is None:
            return

        if self.original_terminal_settings is not None:
            termios.tcsetattr(
                self.tty_fd,
                termios.TCSADRAIN,
                self.original_terminal_settings,
            )
        os.close(self.tty_fd)
        self.tty_fd = None

    def _read_keys(self):
        if self.tty_fd is None:
            return []

        keys = []
        while True:
            readable, _, _ = select.select([self.tty_fd], [], [], 0.0)
            if not readable:
                break

            try:
                chunk = os.read(self.tty_fd, 16)
            except BlockingIOError:
                break

            if not chunk:
                break

            keys.extend(chunk.decode(errors='ignore'))

        return keys

    def _handle_key(self, key, now):
        if key == '\x03':
            rclpy.shutdown()
            return

        key = key.lower()
        key_hold_sec = float(self.get_parameter('key_hold_sec').value)

        if key in self.AXIS_KEYS:
            axis_name, direction = self.AXIS_KEYS[key]
            self.axis_directions[axis_name] = direction
            self.axis_deadlines[axis_name] = now + key_hold_sec
            return

        if key in self.BUTTON_KEYS:
            self.button_pulses.add(self.BUTTON_KEYS[key])

    def timer_callback(self):
        now = time.monotonic()
        for key in self._read_keys():
            self._handle_key(key, now)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'keyboard'
        msg.axes = [0.0, 0.0, 0.0, 0.0]
        msg.buttons = [0] * 5

        axis_value = float(self.get_parameter('axis_value').value)
        for axis_name, axis_index in self.AXIS_INDEX.items():
            if self.axis_deadlines[axis_name] > now:
                msg.axes[axis_index] = self.axis_directions[axis_name] * axis_value

        for button_index in self.button_pulses:
            msg.buttons[button_index] = 1
        self.button_pulses.clear()

        self.joy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
