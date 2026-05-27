#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ArucamPublisherNode(Node):
    def __init__(self):
        super().__init__('arducam_publisher')

        self.declare_parameter('camera_device', '/dev/Arducam')
        self.declare_parameter('image_topic',   '/arducam/image_raw')
        self.declare_parameter('fps',           30.0)

        camera_device = self.get_parameter('camera_device').value
        image_topic   = self.get_parameter('image_topic').value
        fps           = self.get_parameter('fps').value

        self._cap = cv2.VideoCapture(camera_device)
        if not self._cap.isOpened():
            self.get_logger().fatal(f'Cannot open camera device: {camera_device}')
            raise RuntimeError(f'Cannot open {camera_device}')

        self.get_logger().info(f'Opened {camera_device} → publishing on {image_topic} at {fps} Hz')

        self._bridge = CvBridge()
        self._pub    = self.create_publisher(Image, image_topic, 10)
        self._timer  = self.create_timer(1.0 / fps, self._timer_cb)

    def _timer_cb(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arducam'
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucamPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
