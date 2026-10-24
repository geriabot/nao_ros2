#!/usr/bin/env python3

"""
camera_conversion_node.py

This node subscribes to /image_raw, converts the image from YUV422 to RGB, flips it vertically,
and publishes it to /image_rgb along with calibrated CameraInfo to /camera_rgb_info.

Over Wi-Fi the raw RGB image (640x480, about 900 KB) limits the frame rate. With the parameter
publish_compressed it also publishes the image as JPEG (sensor_msgs/CompressedImage, about 50 KB) on
/image_rgb/compressed, the topic and format of image_transport's compressed transport; with
publish_raw false it stops publishing /image_rgb, so that only the compressed image crosses the
network. On the computer, compressed_image_decoder turns it back into /image_rgb.
  publish_raw (bool, true)          publish /image_rgb
  publish_compressed (bool, false)  publish /image_rgb/compressed
  jpeg_quality (int, 80)            JPEG quality, 1-100
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
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

        self.publish_raw = self.declare_parameter('publish_raw', True).value
        self.publish_compressed = self.declare_parameter('publish_compressed', False).value
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)
        self.image_pub = self.create_publisher(Image, '/image_rgb', 10) if self.publish_raw else None
        self.compressed_pub = (self.create_publisher(CompressedImage, '/image_rgb/compressed', 10)
                               if self.publish_compressed else None)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_rgb_info', 10)

        # Calibration values for nao camera
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

        self.get_logger().info(
            f'camera_conversion_node initialized (raw: {self.publish_raw}, compressed: '
            f'{self.publish_compressed}, JPEG quality {self.jpeg_quality}).')

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
            if self.image_pub is not None:
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
                rgb_msg.header.stamp = msg.header.stamp
                rgb_msg.header.frame_id = 'CameraTop_frame'
                self.image_pub.publish(rgb_msg)

            # Publish it as JPEG, as image_transport's compressed transport does (format string
            # included), so that any compressed subscriber can decode it
            if self.compressed_pub is not None:
                ok, jpeg = cv2.imencode('.jpg', cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR),
                                        [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if ok:
                    compressed = CompressedImage()
                    compressed.header.stamp = msg.header.stamp
                    compressed.header.frame_id = 'CameraTop_frame'
                    compressed.format = 'rgb8; jpeg compressed bgr8'
                    compressed.data = jpeg.tobytes()
                    self.compressed_pub.publish(compressed)

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
