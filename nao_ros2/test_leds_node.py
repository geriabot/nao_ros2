import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nao_led_interfaces.action import LedsPlay
from nao_led_interfaces.msg import LedIndexes, LedModes
from std_msgs.msg import ColorRGBA

class TestLEDsNode(Node):
    def __init__(self):
        super().__init__('test_leds_client')
        self._action_client = ActionClient(self, LedsPlay, '/leds_play')
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Waiting for LED action server...')
        self._action_client.wait_for_server()

        # Build goal
        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.REYE, LedIndexes.LEYE]
        goal_msg.mode = LedModes.BLINKING
        goal_msg.frequency = 2.0
        goal_msg.colors = [ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)] * 8  
        goal_msg.intensities = [1.0] * 12       # full intensity for all 12 LEDs
        goal_msg.duration = 5.0                  # seconds

        self.get_logger().info('Sending LED goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Success: {result.success}')
        self.get_logger().info('Shutting down node...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestLEDsNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
