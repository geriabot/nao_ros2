import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

class ImuComparer(Node):
    def __init__(self):
        super().__init__('imu_comparer')
        self.raw_acc = {'x': [], 'y': [], 'z': []}
        self.raw_gyro = {'x': [], 'y': [], 'z': []}
        self.filtered_acc = {'x': [], 'y': [], 'z': []}
        self.filtered_gyro = {'x': [], 'y': [], 'z': []}
        self.time = []

        self.sub_raw = self.create_subscription(
            Imu,
            '/imu',
            self.raw_callback,
            10)
        
        self.sub_filtered = self.create_subscription(
            Imu,
            '/imu/data',
            self.filtered_callback,
            10)
        
        self.start_time = None
        self.counter = 0
        self.max_samples = 500 # Max number of samples to collect

    def raw_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.counter < self.max_samples:
            self.raw_acc['x'].append(msg.linear_acceleration.x)
            self.raw_acc['y'].append(msg.linear_acceleration.y)
            self.raw_acc['z'].append(msg.linear_acceleration.z)
            self.raw_gyro['x'].append(msg.angular_velocity.x)
            self.raw_gyro['y'].append(msg.angular_velocity.y)
            self.raw_gyro['z'].append(msg.angular_velocity.z)

            self.time.append(t)
        self.counter += 1

        if self.counter == self.max_samples:
            self.plot_results()

    def filtered_callback(self, msg):
        if len(self.filtered_acc['x']) < self.max_samples:
            self.filtered_acc['x'].append(msg.linear_acceleration.x)
            self.filtered_acc['y'].append(msg.linear_acceleration.y)
            self.filtered_acc['z'].append(msg.linear_acceleration.z)
            self.filtered_gyro['x'].append(msg.angular_velocity.x)
            self.filtered_gyro['y'].append(msg.angular_velocity.y)
            self.filtered_gyro['z'].append(msg.angular_velocity.z)

    def plot_results(self):
        plt.figure(figsize=(15, 10))

        # Linear acceleration
        plt.subplot(2, 1, 1)
        plt.plot(self.time, self.raw_acc['x'], label='Raw Acc X')
        plt.plot(self.time, self.filtered_acc['x'], label='Filtered Acc X')
        plt.plot(self.time, self.raw_acc['y'], label='Raw Acc Y')
        plt.plot(self.time, self.filtered_acc['y'], label='Filtered Acc Y')
        plt.plot(self.time, self.raw_acc['z'], label='Raw Acc Z')
        plt.plot(self.time, self.filtered_acc['z'], label='Filtered Acc Z')
        plt.title('Linear Acceleration Comparison')
        plt.xlabel('Time [s]')
        plt.ylabel('Acceleration [m/s²]')
        plt.legend()
        plt.grid()

        # Angular velocity
        plt.subplot(2, 1, 2)
        plt.plot(self.time, self.raw_gyro['x'], label='Raw Gyro X')
        plt.plot(self.time, self.filtered_gyro['x'], label='Filtered Gyro X')
        plt.plot(self.time, self.raw_gyro['y'], label='Raw Gyro Y')
        plt.plot(self.time, self.filtered_gyro['y'], label='Filtered Gyro Y')
        plt.plot(self.time, self.raw_gyro['z'], label='Raw Gyro Z')
        plt.plot(self.time, self.filtered_gyro['z'], label='Filtered Gyro Z')
        plt.title('Angular Velocity Comparison')
        plt.xlabel('Time [s]')
        plt.ylabel('Angular Velocity [rad/s]')
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()

        self.get_logger().info("Plot finished. Shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImuComparer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()