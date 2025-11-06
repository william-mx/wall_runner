#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int16MultiArray


class WallFollower(Node):
    def __init__(self):
        super().__init__('uss_wall_follower')

        # Mapping sensor indices to tf frames
        self.load_params()

        # --- Class attributes ---
        self.win_width = 20
        self.sensors = ["USS_FC", "USS_FR", "USS_SRF", "USS_SRB"]

        # Resolve sensor indices from name → numeric index
        self.sensor_indices = [self.frame2index[name] for name in self.sensors]

        # Create empty queue: shape (win_width, num_sensors)
        self.buffer = np.zeros((self.win_width, len(self.sensors)), dtype=np.float32)
        self.buffer_ptr = 0  # circular write pointer

        # Publisher
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/autonomous/ackermann_cmd', 10)

        # Subscriber
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1
        self.create_subscription(Int16MultiArray, '/uss_sensors', self.callback, qos_profile)

        self.get_logger().info(f"WallFollower started. Tracking sensors: {self.sensors}")

    def callback(self, msg):
        distances = msg.data  # length 10 array of ints

        # Extract selected sensors in the declared order
        row = np.array([distances[i] for i in self.sensor_indices], dtype=np.float32)

        # Insert into circular buffer
        self.buffer[self.buffer_ptr] = row
        self.buffer_ptr = (self.buffer_ptr + 1) % self.win_width

        # For now, just log every few calls to verify it's working
        if self.buffer_ptr == 0:
            self.get_logger().info(f"Buffer updated:\n{self.buffer}")

        # (Later) compute control here using buffer → pass to Ackermann msg

    def load_params(self):
        # Declare parameters with default mapping
        self.declare_parameters(
            namespace='',
            parameters=[
                ('index2frame.0', 'USS_SRB'),
                ('index2frame.1', 'USS_SRF'),
                ('index2frame.2', 'USS_FR'),
                ('index2frame.3', 'USS_FC'),
                ('index2frame.4', 'USS_FL'),
                ('index2frame.5', 'USS_SLF'),
                ('index2frame.6', 'USS_SLB'),
                ('index2frame.7', 'USS_BL'),
                ('index2frame.8', 'USS_BC'),
                ('index2frame.9', 'USS_BR')
            ]
        )

        # Build lookup tables
        self.frame2index = {
            self.get_parameter(f'index2frame.{i}').value: i
            for i in range(10)
        }
        self.index2frame = {v: k for k, v in self.frame2index.items()}


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
