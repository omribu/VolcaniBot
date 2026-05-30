#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Twist
from yolo_msgs.msg import DetectionArray


class PersonTargetNode(Node):
    DISABLED = "DISABLED"
    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"

    def __init__(self):
        super().__init__("person_target_node")

        self.declare_parameter("toggle_button", 4)
        self.declare_parameter("detections_topic", "/yolo/detections_3d")
        self.declare_parameter("yolo_enable_service", "/yolo/yolo_node/enable")
        self.declare_parameter("lost_timeout", 2.0)

        # ── velocity: linear ──────────────────────────────────────────────────
        self.declare_parameter("min_distance", 1.35)    # m — start moving
        self.declare_parameter("max_distance", 2.50)    # m — full speed
        self.declare_parameter("max_linear_vel", 0.3)   # m/s magnitude (applied as negative)

        # ── velocity: angular ─────────────────────────────────────────────────
        self.declare_parameter("max_angular_vel", 0.5)  # rad/s cap
        self.declare_parameter("max_angle_deg", 45.0)   # deg at which angular vel saturates

        self.declare_parameter("safety_timeout", 0.5)   # seconds

        self._button = self.get_parameter("toggle_button").get_parameter_value().integer_value
        det_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        svc_name = self.get_parameter("yolo_enable_service").get_parameter_value().string_value
        self._lost_timeout = self.get_parameter("lost_timeout").get_parameter_value().double_value
        self._min_dist = self.get_parameter("min_distance").get_parameter_value().double_value
        self._max_dist = self.get_parameter("max_distance").get_parameter_value().double_value
        self._max_linear = self.get_parameter("max_linear_vel").get_parameter_value().double_value
        self._max_angular = self.get_parameter("max_angular_vel").get_parameter_value().double_value
        self._max_angle_rad  = math.radians(self.get_parameter("max_angle_deg").get_parameter_value().double_value)
        self._safety_timeout = self.get_parameter("safety_timeout").get_parameter_value().double_value


        self._state: str = self.DISABLED
        self._locked_id: Optional[str] = None
        self._prev_buttons: list = []
        self._last_seen: float = 0.0
        self._last_pos: Optional[PointStamped] = None
        self._last_pos_time: Optional[float] = None

        self._yolo_client = self.create_client(SetBool, svc_name)

        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(DetectionArray, det_topic, self._detections_cb, 10)

        self._state_pub = self.create_publisher(String, "/follow_person/state", 10)
        self._pos_pub = self.create_publisher(PointStamped, "/follow_person/target_position", 10)
        self._cmd_pub = self.create_publisher(Twist, "/nav_vel", 10)

        self.create_timer(0.05, self._control_loop)  # 20 Hz velocity commands
        self.create_timer(1.0,  self._publish_state) # 1 Hz state broadcast

        self.get_logger().info(f"PersonTargetNode ready — press button {self._button} (LB) to toggle tracking")

    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)
        rising = (
            self._button < len(buttons)
            and len(self._prev_buttons) > self._button
            and buttons[self._button] == 1
            and self._prev_buttons[self._button] == 0
        )
        if rising:
            if self._state == self.DISABLED:
                self._set_state(self.SEARCHING)
                self._call_enable(True)
            else:
                self._locked_id = None
                self._last_pos  = None
                self._last_pos_time = None
                self._set_state(self.DISABLED)
                self._call_enable(False)
        self._prev_buttons = buttons

    def _detections_cb(self, msg: DetectionArray):
        if self._state == self.DISABLED:
            return

        persons = [
            d for d in msg.detections
            if d.class_name == "person" and d.bbox3d.size.x > 0.0
        ]
        now = self.get_clock().now().nanoseconds / 1e9

        # Lock onto the closest person when first searching
        if self._state == self.SEARCHING and persons:
            closest = min(persons, key=self._dist3d)
            self._locked_id = closest.id
            self._last_seen = now
            self._set_state(self.TRACKING)
            self.get_logger().info(f"Locked onto person id='{closest.id}' at {self._dist3d(closest):.2f} m")

        if self._state == self.TRACKING:
            target = next((d for d in persons if d.id == self._locked_id), None)
            if target:
                self._last_seen = now
                pt = PointStamped()
                pt.header.stamp    = msg.header.stamp
                pt.header.frame_id = target.bbox3d.frame_id
                pt.point.x = target.bbox3d.center.position.x
                pt.point.y = target.bbox3d.center.position.y
                pt.point.z = target.bbox3d.center.position.z
                self._pos_pub.publish(pt)
                self._last_pos      = pt
                self._last_pos_time = now
            elif now - self._last_seen > self._lost_timeout:
                self.get_logger().warn(f"Person '{self._locked_id}' lost — resuming search")
                self._locked_id = None
                self._last_pos  = None
                self._last_pos_time = None
                self._set_state(self.SEARCHING)


    def _control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # Safety gate 1: LB not active - zero immediately
        if self._state != self.TRACKING:
            self._cmd_pub.publish(Twist())
            return

        # Safety gate 2: position stale or never received - zero immediately
        if self._last_pos_time is None or now - self._last_pos_time > self._safety_timeout:
            self._cmd_pub.publish(Twist())
            return

        x = self._last_pos.point.x  
        y = self._last_pos.point.y  

        # Linear velocity 
        dist = abs(x)
        if dist <= self._min_dist:
            linear_x = 0.0
        else:
            t = min(1.0, (dist - self._min_dist) / (self._max_dist - self._min_dist))
            linear_x = -t * self._max_linear  # negative = drive backward toward person

        # Angular velocity
        angle_error = math.atan2(y, -x)
        K_ang = self._max_angular / self._max_angle_rad
        angular_z = max(-self._max_angular, min(self._max_angular, -K_ang * angle_error))

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _dist3d(detection) -> float:
        p = detection.bbox3d.center.position
        return math.sqrt(p.x**2 + p.y**2 + p.z**2)

    def _set_state(self, state: str):
        self._state = state
        self.get_logger().info(f"State → {state}")

    def _publish_state(self):
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _call_enable(self, enable: bool):
        req = SetBool.Request()
        req.data = enable

        def _done(future):
            try:
                future.result()
                self.get_logger().info(f"YOLO {'ENABLED' if enable else 'DISABLED'}")
            except Exception as e:
                self.get_logger().warn(f"YOLO enable call failed: {e}")

        self._yolo_client.call_async(req).add_done_callback(_done)


def main():
    rclpy.init()
    node = PersonTargetNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
