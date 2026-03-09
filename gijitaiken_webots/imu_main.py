#!/usr/bin/env python3
import rclpy
from gijitaiken_webots.node.imu_node import ImuNode

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()