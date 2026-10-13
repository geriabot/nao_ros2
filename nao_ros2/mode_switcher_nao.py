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
        self.has_walked = False
        # A posture of nao_pos_server moving the legs (from "Action started" to "succeeded").
        # A cancelled one does not report its end, hence the timeout
        self.posture_running = False
        self.posture_started_at = 0.0
        self.posture_timeout = 15.0

        self.last_processed_time = self.get_clock().now()
        self.min_processing_interval = 2.0

        self.set_initial_status()
        # /walk_status again every second: nao_pos_publisher waits for the first one before
        # starting the swing, and a single message sent at start-up can be lost (e.g. before
        # the discovery between containers has finished), which left the robot without a swing
        self.walk_status = False
        self.create_timer(1.0, lambda: self.pub_walk_status.publish(Bool(data=self.walk_status)))

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
                self.walk_status = True
                self.pub_walk_status.publish(Bool(data=True))
                self.ensure_stand_then_walk()
        else:
            if self.is_walking:
                self.get_logger().info("The robot stops.")
                self.is_walking = False
                self.walk_status = False
                self.pub_walk_status.publish(Bool(data=False))
                self.pub_walk_control.publish(Bool(data=False))
                self.is_standing = False
                self.ensure_stand()

    def is_moving(self, msg):
        return any([
            msg.linear.x != 0.0, msg.linear.y != 0.0, msg.linear.z != 0.0,
            msg.angular.x != 0.0, msg.angular.y != 0.0, msg.angular.z != 0.0
        ])

    def posture_in_progress(self):
        if self.posture_running and time.monotonic() - self.posture_started_at > self.posture_timeout:
            self.posture_running = False  # its end was not reported (e.g. cancelled)
        return self.posture_running

    def ensure_stand_then_walk(self):
        # Never start walking while a posture moves the legs, e.g. a swing that
        # nao_pos_publisher started while the robot was stopped: the walk and the posture would
        # command the legs at the same time and the robot falls. Called again with the next
        # /target, every 2 s, until it can go on
        if self.posture_in_progress():
            self.get_logger().info("Waiting for the posture in progress to finish...")
            return

        if self.is_standing:
            self.start_walking()
            return

        # At start-up, a swing first. After a stop no swing may come (nao_pos_publisher only
        # sends them while /walk_status is false, and it is already true): stand directly
        if self.swing_completed or self.has_walked:
            self.get_logger().info("Executing 'stand'...")
            self.try_stand()
        else:
            self.get_logger().info("Waiting for 'swing' to finish before executing 'stand'...")

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

        if msg.data.startswith("Action started"):
            self.posture_running = True
            self.posture_started_at = time.monotonic()
            self.is_standing = False
        elif "succeeded" in msg.data.lower():
            self.posture_running = False
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
        self.has_walked = True
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
