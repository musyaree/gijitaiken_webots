#!/usr/bin/env python3
import rclpy
from gijitaiken_webots.node.logic_node import LogicNode

def main(args=None):
    rclpy.init(args=args)
    node = LogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()