#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class DetectPersonNode(Node):
    def __init__(self):
        super().__init__('detect_person_node')

        # Parameters
        self.declare_parameter('camera_topic',          '/arducam/image_raw')
        self.declare_parameter('mask_topic',            '/yolo/person_mask')
        self.declare_parameter('model_path',            'yolov8n.pt')
        self.declare_parameter('confidence_threshold',  0.5)
        self.declare_parameter('max_fps',               5.0)
        self.declare_parameter('device',                '')
        self.declare_parameter('display',               True)

        camera_topic        = self.get_parameter('camera_topic').value
        mask_topic          = self.get_parameter('mask_topic').value
        model_path          = self.get_parameter('model_path').value
        self._conf          = self.get_parameter('confidence_threshold').value
        max_fps             = self.get_parameter('max_fps').value
        self._device        = self.get_parameter('device').value
        self._display       = self.get_parameter('display').value

        self._min_interval  = 1.0 / max(max_fps, 0.1)
        self._last_run      = 0.0

        # YOLO model
        self.get_logger().info(f'Loading YOLO model from {model_path}')
        self._model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded')

        try:
            import torch
            if torch.cuda.is_available():
                self.get_logger().info(f'CUDA available: {torch.cuda.get_device_name(0)}')
            else:
                self.get_logger().warn(
                    'CUDA not available — YOLO running on CPU. '
                    'Install Jetson PyTorch wheel for GPU acceleration.'
                )
        except ImportError:
            pass

        self._bridge = CvBridge()
        self._sub = self.create_subscription(Image, camera_topic, self._image_cb, 1)
        self._pub = self.create_publisher(Image, mask_topic, 1)

        self.get_logger().info(
            f'Subscribed to {camera_topic} | publishing mask on {mask_topic} | '
            f'max_fps={max_fps} | conf={self._conf}'
        )

    def _image_cb(self, msg: Image):
        # Rate-limit inference
        now = time.monotonic()
        if now - self._last_run < self._min_interval:
            return
        self._last_run = now

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        try:
            results = self._model(
                frame,
                classes=[0],        # class 0 = person in COCO
                conf=self._conf,
                device=self._device if self._device else None,
                verbose=False,
            )[0]
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return

        h, w = frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)

        annotated = frame.copy()
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            mask[y1:y2, x1:x2] = 255
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            conf_val = float(box.conf[0])
            cv2.putText(annotated, f'person {conf_val:.2f}',
                        (x1, max(y1 - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish binary person mask
        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self._pub.publish(mask_msg)

        if self._display:
            cv2.imshow('DetectPerson', annotated)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectPersonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
