#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations



class ImuTransformer(Node):
    def __init__(self):
        super().__init__('imu_transformer')
        self.pub = self.create_publisher(Imu, '/imu/data_enu', 10)
        self.sub = self.create_subscription(Imu, '/imu/data', self.callback, 10)
        
    def callback(self, msg):

        enu_msg = Imu()
        enu_msg.header = msg.header
        enu_msg.header.frame_id = "base_link"
        
        # Linear acceleration conversion
        enu_msg.linear_acceleration.x = -msg.linear_acceleration.x
        enu_msg.linear_acceleration.y = -msg.linear_acceleration.y
        enu_msg.linear_acceleration.z = -msg.linear_acceleration.z

        # Angular velocity conversion
        enu_msg.angular_velocity.x = msg.angular_velocity.x
        enu_msg.angular_velocity.y = msg.angular_velocity.y
        enu_msg.angular_velocity.z = msg.angular_velocity.z

        # Orientation conversion
        q_orig = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # Convert quaternion from NED to ENU
        q_flip = tf_transformations.quaternion_about_axis(3.14159, (1, 0, 0))
        q_orig = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_new = tf_transformations.quaternion_multiply(q_flip, q_orig)

        enu_msg.orientation.x = q_new[0]
        enu_msg.orientation.y = q_new[1]
        enu_msg.orientation.z = q_new[2]
        enu_msg.orientation.w = q_new[3]

        self.pub.publish(enu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()