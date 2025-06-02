import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

class NaoWalkOdometry(Node):
    def __init__(self):
        super().__init__('nao_walk_odometry')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Initial position (NED frame: X forward, Y right, Z down)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation from IMU
        self.vx = 0.0
        self.vy = 0.0

        self.last_time = self.get_clock().now()
        self.last_twist_time = self.last_time

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.sub_twist = self.create_subscription(Twist, '/walk/current_twist', self.twist_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Timer to publish transform and odometry at fixed rate
        self.timer = self.create_timer(0.05, self.publish_odometry)  # 20 Hz

        self.get_logger().info('nao_walk_odometry node (using IMU in NED coordinates) initialized')

    def imu_callback(self, imu_msg):
        # Get yaw from IMU (accurate orientation)
        orientation_q = imu_msg.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Invert yaw for correct rotation direction
        self.theta = -yaw

    def twist_callback(self, twist_msg):
        # Store velocities
        self.vx = twist_msg.linear.x
        self.vy = twist_msg.linear.y
        self.last_twist_time = self.get_clock().now()

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Zero velocities if no recent twist received
        time_since_last_twist = (current_time - self.last_twist_time).nanoseconds * 1e-9
        if time_since_last_twist > 0.01:
            self.vx = 0.0
            self.vy = 0.0

        # Integrate position using accurate orientation (IMU)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt

        self.x += delta_x
        self.y += delta_y

        # Quaternion from IMU orientation
        odom_quat = quaternion_from_euler(0, 0, self.theta)

        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        # Commentesd as tf is broadcasted in robot_localization
        self.tf_broadcaster.sendTransform(t)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = t.header.frame_id
        odom.child_frame_id = t.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = 0.0  # Angular provided directly from IMU
        

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = NaoWalkOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('NaoWalkOdometry node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('NaoWalkOdometry node destroyed and rclpy shutdown')

if __name__ == '__main__':
    main()
