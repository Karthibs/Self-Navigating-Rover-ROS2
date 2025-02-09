#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class ImuSubNode(Node):
    def __init__(self):
        super().__init__("imu_sub")
        self.subscriber_ = self.create_subscription(
            Imu, "imu_data", self.callback_imu_data, 10)
        self.get_logger().info("imu_sub_started")

    def callback_imu_data(self, msg):
        print("Received IMU Data:")
        print("Linear Acceleration: x={}, y={}, z={}".format(
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ))
        print("Angular Velocity: x={}, y={}, z={}".format(
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ))
        print("Orientation: x={}, y={}, z={}, w={}".format(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ))


def main(args=None):
    rclpy.init(args=args)
    node = ImuSubNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()