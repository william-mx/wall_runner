#!/usr/bin/env python3
from collections import deque
import numpy as np
import joblib

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int16MultiArray
from ament_index_python.packages import get_package_prefix

from ros2_numpy import to_ackermann, multiarray_to_np

class WallFollower(Node):
    def __init__(self, model_path):
        super().__init__('uss_wall_follower')

        self.load_params()

        # Sensors and rolling window
        self.sensors = ["USS_FL", "USS_FC", "USS_FR", "USS_SRF", "USS_SRB", "USS_SLB", "USS_SLF"]
        self.win_width = 30
        self.sensor_indices = [self.frame2index[name] for name in self.sensors]

        self.window = deque(maxlen=self.win_width)
        for _ in range(self.win_width):
            self.window.append(np.zeros(len(self.sensors), dtype=np.float32))

        # Fixed driving speed
        self.fixed_speed = 0.8

        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1

        # Publisher
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/autonomous/ackermann_cmd', qos_profile)

        # Subscriber
        self.create_subscription(Int16MultiArray, '/uss_sensors', self.uss_callback, qos_profile)

        self.model = joblib.load(model_path)
        self.get_logger().info(f"Loaded model from: {model_path}")

    def uss_callback(self, msg):
        distances, timestamp_unix = multiarray_to_np(msg)
        signals = distances[self.sensor_indices]

        # add to rolling window
        self.window.append(signals)

        # not enough data yet
        if len(self.window) < self.win_width:
            return

        # flatten window → model input
        x = np.array(self.window, dtype=np.float32).reshape(1, -1)

        # predict steering
        try:
            steering_angle = float(self.model.predict(x)[0])
        except Exception as e:
            self.get_logger().warn(f"Prediction failed: {e}")
            return

        # Create an Ackermann drive message with speed and steering angle
        cmd = to_ackermann(self.fixed_speed, steering_angle, timestamp_unix)

        self.cmd_pub.publish(cmd)

    def load_params(self):
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
        self.frame2index = {self.get_parameter(f'index2frame.{i}').value: i for i in range(10)}
        self.index2frame = {v: k for k, v in self.frame2index.items()}


def main():
    pkg_path = get_package_prefix('wall_runner').replace('install', 'src')
    model_path = pkg_path + '/models/wall_follow_rf.pkl'

    rclpy.init()
    node = WallFollower(model_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
