import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile
import time

from nao_lola_command_msgs.msg import JointStiffnesses, JointPositions
from biped_interfaces.msg import SolePoses

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher_nao')

        self.get_logger().info("Initializing ModeSwitcher...")

        self.sub_target = self.create_subscription(Twist, '/target', self.target_callback, 10)

        self.pub_walk_status = self.create_publisher(Bool, '/walk_status', 10)
        self.pub_walk_control = self.create_publisher(Bool, '/walk_control', 10)
        self.pub_stiffness = self.create_publisher(JointStiffnesses, '/effectors/joint_stiffnesses', 10)
        self.pub_initial_joints_position = self.create_publisher(JointPositions, '/effectors/joint_positions', 10)
        self.pub_initial_position = self.create_publisher(SolePoses, '/motion/sole_poses', 10)

        self.pub_action_req = self.create_publisher(String, "action_req_legs", 10)
        self.pub_action_req_arms = self.create_publisher(String, "action_req_arms", 10)

        self.sub_action_status = self.create_subscription(
            String, "/nao_pos_action/status", self.action_status_callback, QoSProfile(depth=10)
        )

        self.is_walking = False
        self.is_standing = False
        self.swing_completed = False

        self.last_processed_time = self.get_clock().now()
        self.min_processing_interval = 2.0

        self.set_initial_status()

        self.get_logger().info("ModeSwitcher started successfully.")

    def set_initial_status(self):
        self.get_logger().info("Waiting for subscribers to connect for walk status...")
        while self.pub_walk_status.get_subscription_count() == 0 and rclpy.ok():
            time.sleep(0.5)
        if rclpy.ok():
            self.pub_walk_status.publish(Bool(data=False))
            self.get_logger().info("Initial walk status published.")

    def target_callback(self, msg):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_processed_time).nanoseconds / 1e9
        if elapsed_time < self.min_processing_interval:
            return
        
        self.last_processed_time = current_time
        
        self.get_logger().info("Message received on /target. Evaluating movement...")
        if self.is_moving(msg):
            if not self.is_walking:
                self.get_logger().info("The robot is trying to start walking.")
                self.pub_walk_status.publish(Bool(data=True))
                self.ensure_stand_then_walk()
        else:
            if self.is_walking:
                self.get_logger().info("The robot stops.")
                self.is_walking = False
                self.pub_walk_status.publish(Bool(data=False))
                self.pub_walk_control.publish(Bool(data=False))
                self.is_standing = False
                self.ensure_stand()

    def is_moving(self, msg):
        return any([
            msg.linear.x != 0.0, msg.linear.y != 0.0, msg.linear.z != 0.0,
            msg.angular.x != 0.0, msg.angular.y != 0.0, msg.angular.z != 0.0
        ])

    def ensure_stand_then_walk(self):
        if not self.swing_completed:
            self.get_logger().info("Waiting for 'swing' to finish before executing 'stand'...")
            return
            
        if not self.is_standing:
            self.get_logger().info("Executing 'stand'...")
            self.try_stand()
        else:
            self.start_walking()

    def ensure_stand(self):
        if not self.is_standing:
            self.try_stand()

    def try_stand(self):
        self.get_logger().info("Publishing 'stand' on action_req_legs...")
        msg = String()
        msg.data = "stand"
        self.pub_action_req.publish(msg)

    def action_status_callback(self, msg):
        self.get_logger().info(f"Action status received: {msg.data}")

        if "succeeded" in msg.data.lower():
            if "only_legs_fast" in msg.data.lower():
                self.get_logger().info("Swing finished...")
                self.swing_completed = True
                self.is_standing = False
            elif "stand" in msg.data.lower():
                self.get_logger().info("Stand completed...")
                self.is_standing = True

    def start_walking(self):
        self.get_logger().info("Starting walk...")
        self.set_stiffness()
        self.set_walk_sole_position()
        # Publish arms action request
        msg = String()
        msg.data = "armsDown"
        self.pub_action_req_arms.publish(msg)
        
        # Sleep so that the robot has time to get to the initial position
        time.sleep(2.5)
        self.pub_walk_control.publish(Bool(data=True))
        self.is_walking = True
        self.is_standing = False
        self.swing_completed = False
        self.get_logger().info("Walk started successfully.")

    def set_stiffness(self):
        self.get_logger().info("Setting joint stiffness.")
        msg = JointStiffnesses(indexes=list(range(25)), stiffnesses=[1.0] * 25)
        self.pub_stiffness.publish(msg)
        self.get_logger().info("Stiffness set.")

    def set_walk_sole_position(self):
        self.get_logger().info("Setting initial position.")
        msg = SolePoses()
        msg.l_sole.position.y, msg.l_sole.position.z = 0.05, -0.315
        msg.r_sole.position.y, msg.r_sole.position.z = -0.05, -0.315
        self.pub_initial_position.publish(msg)
        self.get_logger().info("Initial position set.")

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping ModeSwitcher.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
