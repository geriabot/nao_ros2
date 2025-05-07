#!/usr/bin/env python3

"""
camera_conversion_node.py

This node subscribes to /image_raw, converts the image from YUV422 to RGB, flips it vertically,
and publishes it to /image_rgb along with calibrated CameraInfo to /camera_rgb_info.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class CameraConversionNode(Node):

    def __init__(self):
        super().__init__('camera_conversion_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, '/image_rgb', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_rgb_info', 10)

        # Sample calibration values (replace with real values in the laboratory)
        self.camera_info = CameraInfo()
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.k = [544.303,   0.0  , 319.5,
                                0.0  , 544.303, 239.5,
                                0.0  ,   0.0  ,   1.0]
        self.camera_info.d = [0.0607252, -0.253295, 0.0, 0.0, 0.0]
        self.camera_info.r = [1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0]
        self.camera_info.p = [544.303,   0.0  , 319.5, 0.0,
                                0.0  , 544.303, 239.5, 0.0,
                                0.0  ,   0.0  ,   1.0, 0.0]

        self.get_logger().info('camera_conversion_node initialized.')

    def image_callback(self, msg):
        try:
            # Convert from YUV422 to RGB
            yuv_image = self.bridge.imgmsg_to_cv2(msg)
            img = yuv_image.reshape(480, 640, 2)
            rgb_image = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)

            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            # Resize the image to match camera info dimensions
            rgb_image = cv2.resize(rgb_image, (self.camera_info.width, self.camera_info.height))
            # Ensure the image is in the correct format
            rgb_image = rgb_image.astype('uint8')
            # Check if the image is empty
            if rgb_image.size == 0:
                self.get_logger().error('Received empty image.')
                return
            # Check if the image has the correct number of channels
            if rgb_image.shape[2] != 3:
                self.get_logger().error('Received image does not have 3 channels.')
                return
            # Check if the image has the correct dimensions
            if rgb_image.shape[0] != self.camera_info.height or rgb_image.shape[1] != self.camera_info.width:
                self.get_logger().error('Received image dimensions do not match camera info.')
                return

            # Flip the image vertically
            rgb_image = cv2.rotate(rgb_image, cv2.ROTATE_180)

            # Publish RGB image
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            rgb_msg.header.stamp = msg.header.stamp
            rgb_msg.header.frame_id = 'CameraTop_frame'
            self.image_pub.publish(rgb_msg)

            # Publish calibrated camera info
            self.camera_info.header.stamp = msg.header.stamp
            self.camera_info.header.frame_id = 'CameraTop_frame'
            self.camera_info_pub.publish(self.camera_info)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
