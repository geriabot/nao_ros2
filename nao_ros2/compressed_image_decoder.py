#!/usr/bin/env python3

"""
compressed_image_decoder.py

Runs on the computer, not on the Nao. It subscribes to the JPEG image that camera_conversion_node
publishes on /image_rgb/compressed (publish_compressed:=true) and publishes it again, decoded, as
sensor_msgs/Image (rgb8) on /image_rgb, so that the nodes of the computer read the camera as when
the Nao sends it raw, while only the compressed image crosses the Wi-Fi.

It only needs rclpy, sensor_msgs, OpenCV and NumPy, so it runs with ROS 2 Humble or a later
distribution on the computer (for instance Jazzy, talking to the Humble of the Nao):
  ros2 run nao_ros2 compressed_image_decoder                 (a workspace with nao_ros2 built)
  python3 nao_ros2/compressed_image_decoder.py              (any sourced ROS 2, without building)
  input (string, /image_rgb/compressed)   output (string, /image_rgb)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class CompressedImageDecoder(Node):

    def __init__(self):
        super().__init__('compressed_image_decoder')
        source = self.declare_parameter('input', '/image_rgb/compressed').value
        target = self.declare_parameter('output', '/image_rgb').value
        self.image_pub = self.create_publisher(Image, target, 10)
        # The same QoS as the publisher of camera_conversion_node (reliable, depth 10) is matched
        # by a reliable subscriber; a best-effort one would also match it
        self.create_subscription(CompressedImage, source, self.on_compressed, 10)
        self.get_logger().info(f'Decoding {source} into {target}')

    def on_compressed(self, msg):
        bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn('Could not decode a compressed image', throttle_duration_sec=5.0)
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = Image()
        image.header = msg.header
        image.height, image.width = rgb.shape[:2]
        image.encoding = 'rgb8'
        image.is_bigendian = 0
        image.step = image.width * 3
        image.data = rgb.tobytes()
        self.image_pub.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
