import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from nao_led_interfaces.action import LedsPlay
from nao_led_interfaces.msg import LedIndexes, LedModes
from std_msgs.msg import ColorRGBA


class SetLEDsNode(Node):
    def __init__(self):
        super().__init__('set_leds_client')

        self._action_client = ActionClient(self, LedsPlay, '/leds_play')

        self.declare_parameter('led_config_file', 'nao_default.yaml')
        led_config_file = self.get_parameter('led_config_file').get_parameter_value().string_value


        package_share_dir = get_package_share_directory('nao_ros2')
        yaml_file = os.path.join(package_share_dir, 'config', led_config_file)
        with open(yaml_file, 'r') as f:
            self.led_config = yaml.safe_load(f)

        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Waiting for LED action server...')
        self._action_client.wait_for_server()

        self.handle_head_leds()
        self.handle_ear_leds()
        self.handle_eye_leds()
        self.handle_chest_led()

    def handle_chest_led(self):
        goal_msg = LedsPlay.Goal()
        chest_conf = self.led_config['chest'][0]

        goal_msg.leds = [LedIndexes.CHEST, 0]

        goal_msg.colors = [
            ColorRGBA(
                r=chest_conf['color']['r'],
                g=chest_conf['color']['g'],
                b=chest_conf['color']['b'],
                a=chest_conf['color']['a']
            )
        ] * 8 

        goal_msg.intensities = [chest_conf['intensity']] * 8

        self.get_logger().info(f"Setting chest LEDs to {chest_conf['mode']} mode")
        if chest_conf['mode'] == 'steady':
            goal_msg.mode = LedModes.STEADY
            goal_msg.frequency = 0.0
        elif chest_conf['mode'] == 'blinking':
            goal_msg.mode = LedModes.BLINKING
            goal_msg.frequency = chest_conf['frequency']
            self.get_logger().info(f"Blinking frequency: {chest_conf['frequency']} Hz")
        elif chest_conf['mode'] == 'loop':
            goal_msg.mode = LedModes.LOOP
            goal_msg.frequency = chest_conf['frequency']
            self.get_logger().info(f"Looping frequency: {chest_conf['frequency']} Hz")
        else:
            self.get_logger().error(f"Unknown mode: {chest_conf['mode']}")
            return

        goal_msg.duration = chest_conf['duration']

        self.get_logger().info('Sending LED goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def handle_eye_leds(self):
        goal_msg = LedsPlay.Goal()
        eye_conf = self.led_config['eyes'][0]

        goal_msg.leds = [LedIndexes.LEYE, LedIndexes.REYE]

        goal_msg.colors = [
            ColorRGBA(
                r=eye_conf['color']['r'],
                g=eye_conf['color']['g'],
                b=eye_conf['color']['b'],
                a=eye_conf['color']['a']
            )
        ] * 8 

        goal_msg.intensities = [eye_conf['intensity']] * 8

        self.get_logger().info(f"Setting eye LEDs to {eye_conf['mode']} mode")
        if eye_conf['mode'] == 'steady':
            goal_msg.mode = LedModes.STEADY
            goal_msg.frequency = 0.0
        elif eye_conf['mode'] == 'blinking':
            goal_msg.mode = LedModes.BLINKING
            goal_msg.frequency = eye_conf['frequency']
            self.get_logger().info(f"Blinking frequency: {eye_conf['frequency']} Hz")
        elif eye_conf['mode'] == 'loop':
            goal_msg.mode = LedModes.LOOP
            goal_msg.frequency = eye_conf['frequency']
            self.get_logger().info(f"Looping frequency: {eye_conf['frequency']} Hz")
        else:
            self.get_logger().error(f"Unknown mode: {eye_conf['mode']}")
            return

        goal_msg.duration = eye_conf['duration']

        self.get_logger().info('Sending LED goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def handle_ear_leds(self):

        goal_msg = LedsPlay.Goal()

        ear_conf = self.led_config['ears'][0]

        goal_msg.leds = [LedIndexes.LEAR, LedIndexes.REAR]

        goal_msg.colors = [
            ColorRGBA(
                r=ear_conf['color']['r'],
                g=ear_conf['color']['g'],
                b=ear_conf['color']['b'],
                a=ear_conf['color']['a']
            )
        ] * 8 

        goal_msg.intensities = [ear_conf['intensity']] * 8

        self.get_logger().info(f"Setting ear LEDs to {ear_conf['mode']} mode")
        if ear_conf['mode'] == 'steady':
            goal_msg.mode = LedModes.STEADY
            goal_msg.frequency = 0.0
        elif ear_conf['mode'] == 'blinking':
            goal_msg.mode = LedModes.BLINKING
            goal_msg.frequency = ear_conf['frequency']
            self.get_logger().info(f"Blinking frequency: {ear_conf['frequency']} Hz")
        elif ear_conf['mode'] == 'loop':
            goal_msg.mode = LedModes.LOOP
            goal_msg.frequency = ear_conf['frequency']
            self.get_logger().info(f"Looping frequency: {ear_conf['frequency']} Hz")
        else:
            self.get_logger().error(f"Unknown mode: {ear_conf['mode']}")
            return

        goal_msg.duration = ear_conf['duration']

        self.get_logger().info('Sending LED goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def handle_head_leds(self):

        goal_msg = LedsPlay.Goal()

        head_conf = self.led_config['head']

        goal_msg.leds = [LedIndexes.HEAD, 0]

        goal_msg.intensities = [head_conf['intensity']] * 12

        self.get_logger().info(f"Setting head LEDs to {head_conf['mode']} mode")
        if head_conf['mode'] == 'steady':
            goal_msg.mode = LedModes.STEADY
            goal_msg.frequency = 0.0
        elif head_conf['mode'] == 'blinking':
            goal_msg.mode = LedModes.BLINKING
            goal_msg.frequency = head_conf['frequency']
            self.get_logger().info(f"Blinking frequency: {head_conf['frequency']} Hz")
        elif head_conf['mode'] == 'looping':
            goal_msg.mode = LedModes.LOOP
            goal_msg.frequency = head_conf['frequency']
            self.get_logger().info(f"Looping frequency: {head_conf['frequency']} Hz")
        else:
            self.get_logger().error(f"Unknown mode: {head_conf['mode']}")
            return

        goal_msg.duration = head_conf['duration']

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
        # self.get_logger().info('Shutting down node...')
        # rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SetLEDsNode()
    # rclpy.spin(node)
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
