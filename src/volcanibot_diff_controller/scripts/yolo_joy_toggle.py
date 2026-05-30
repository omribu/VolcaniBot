#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class YoloJoyToggle(Node):
    def __init__(self):
        super().__init__("yolo_joy_toggle")
        self.declare_parameter("toggle_button", 4)
        self.declare_parameter("enable_service", "/yolo/yolo_node/enable")

        self._button = self.get_parameter("toggle_button").get_parameter_value().integer_value
        svc_name = self.get_parameter("enable_service").get_parameter_value().string_value

        self._enabled = False
        self._prev_buttons = []

        self._client = self.create_client(SetBool, svc_name)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)

        # Disable YOLO on startup once the service is available (3 s gives nodes time to come up)
        self._init_timer = self.create_timer(3.0, self._initial_disable)

    def _initial_disable(self):
        self._init_timer.cancel()
        self._call_service(False)

    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)
        if (
            self._button < len(buttons)
            and len(self._prev_buttons) > self._button
            and buttons[self._button] == 1
            and self._prev_buttons[self._button] == 0
        ):
            self._enabled = not self._enabled
            self._call_service(self._enabled)
        self._prev_buttons = buttons

    def _call_service(self, enable: bool):
        req = SetBool.Request()
        req.data = enable

        def _done(future):
            try:
                future.result()
                self.get_logger().info(f"YOLO detection {'ENABLED' if enable else 'DISABLED'}")
            except Exception as e:
                self.get_logger().warn(f"YOLO enable service call failed: {e}")

        self._client.call_async(req).add_done_callback(_done)


def main():
    rclpy.init()
    node = YoloJoyToggle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
