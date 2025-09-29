import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from ament_index_python.packages import get_package_share_directory
from nao_lola_command_msgs.msg import SonarUsage




class ActivateSonar(Node):
    def __init__(self):
        super().__init__('activate_sonar_node')

        self.publisher = self.create_publisher(SonarUsage, '/effectors/sonar_usage', 10)

        sonar_activate_msg = SonarUsage()
        sonar_activate_msg.left = True
        sonar_activate_msg.right = True
        self.publisher.publish(sonar_activate_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActivateSonar()
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
