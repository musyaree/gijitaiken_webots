#!/usr/bin/env python3
# import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8
import math
from rclpy.qos import qos_profile_sensor_data

from kansei_interfaces.msg import Status

from gijitaiken_webots.utils.kinematics import Kinematics
from gijitaiken_webots.node.process.fall_detection import FallDetection

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_adapter_node")

        self.fall_detector = FallDetection(limit_angle=60.0)

        self.unit_subscriber = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data) 
        
        self.status_publisher = self.create_publisher(Status, '/measurement/status', 10)
        self.fallen_publisher = self.create_publisher(Int8, 'fallen/fallen', 10)

        print("IMU Adapter Node Started with Modular Fall Detection")

    def imu_callback(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        roll_rad, pitch_rad, yaw_rad = Kinematics.quat_to_euler(qx, qy, qz, qw)
        
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)
        
        current_status = self.fall_detector.check_status(roll_deg, pitch_deg)

        fallen_msg = Int8()
        fallen_msg.data = current_status
        self.fallen_publisher.publish(fallen_msg)

        status_msg = Status()
        status_msg.is_calibrated = True
        status_msg.orientation.roll = float(roll_deg)
        status_msg.orientation.pitch = float(pitch_deg)
        status_msg.orientation.yaw = float(yaw_deg)
        self.status_publisher.publish(status_msg)