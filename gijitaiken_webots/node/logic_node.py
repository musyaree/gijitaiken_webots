#!/usr/bin/env python3
# import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tachimawari_interfaces.msg import SetJoints, ControlJoints, SetTorques
from kansei_interfaces.msg import Status

# Import Logic Murni
from gijitaiken_webots.node.process.middleware import Middleware

class LogicNode(Node):
    def __init__(self):
        super().__init__('tachimawari_node')
        
        self.middleware = Middleware()
        self.imu_yaw = 0.0

        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # --- SUBSCRIBERS ---
        self.control_joints_subscriber = self.create_subscription(
            ControlJoints, 
            'joint/control_joints', 
            self.control_joints_callback, 
            qos_reliable
        )
        
        self.set_joints_subscriber = self.create_subscription(
            SetJoints, 'joint/set_joints', self.set_joints_callback, qos_reliable)
        
        self.set_torques_subscriber = self.create_subscription(
            SetTorques, 'joint/set_torques', self.set_torques_callback, qos_reliable)

        self.sub_status = self.create_subscription(
            Status, '/measurement/status', self.status_callback, 10)

        # --- PUBLISHERS ---
        self.webots_joints_publisher = self.create_publisher(
            SetJoints, '/webots/set_joints', qos_reliable)
            
        self.webots_torques_publisher = self.create_publisher(
            SetTorques, '/webots/set_torques', qos_reliable)

        # Timer Loop 8ms (125Hz)
        self.create_timer(0.008, self.update_middleware)

        print("Gijitaiken Tachimawari: Logic Bridge Ready.")

    def update_middleware(self):
        pass

    def control_joints_callback(self, msg):
        self.middleware.set_rules(msg.control_type, msg.ids)

    def status_callback(self, msg):
        self.imu_yaw = msg.orientation.yaw

    def set_joints_callback(self, msg):
        """
        Logic Inti: Terima Perintah -> Cek Middleware -> Kirim ke Webots
        """
        # 1. Cek Validitas (Priority Logic)
        if not self.middleware.validate(msg.control_type):
            return 

        # 2. Filter Joints (Rule Logic)
        filtered_joints = self.middleware.filter_joints(msg.control_type, msg.joints)

        # 3. Publish
        if filtered_joints:
            joint_msg = SetJoints()
            joint_msg.control_type = msg.control_type
            joint_msg.joints = filtered_joints
            self.webots_joints_publisher.publish(joint_msg)

    def set_torques_callback(self, msg):
        self.webots_torques_publisher.publish(msg)