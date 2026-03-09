import rclpy
import math
# import sys
import os
import time

from tachimawari_interfaces.msg import SetJoints, CurrentJoints, Joint, SetTorques
from shisen_interfaces.msg import CameraConfig
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from gijitaiken_webots.driver.device_manager import DeviceManager
from gijitaiken_webots.utils.config_loader import ConfigLoader
from gijitaiken_webots.utils.kinematics import Kinematics

class RobotController:
    def init(self, webots_node, properties):

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('gijitaiken_webots_adapter')

        print("\n" + "="*30, flush=True)
        print("ROBOT CONTROLLER STARTING...", flush=True)
        print("="*30, flush=True)
        
        # 1. INIT DEVICES (Hardware Layer)
        self.device = DeviceManager(webots_node.robot)
        self.timestep = self.device.timestep
        self.device._init_camera(target_fps=1.0)

        # 2. INIT CONFIG (Config Layer)
        pkg_path = get_package_share_directory('gijitaiken_webots')
        config_path = os.path.join(pkg_path, 'config/joint_directions.json')
        self.config_loader = ConfigLoader(self.node, config_path)
        
        # Cache direction map
        self.joint_directions = {}
        self._update_joint_directions()

        # 3. ROS SETUP
        self._init_ros()

        # 4. VARS
        self.step_counter = 0
        self.head_offset_yaw = 0.0
        self.head_offset_pitch = 0.0
        self.last_cheat_time = 0

        print("Robot Controller Initialized Successfully", flush=True)

    def _init_ros(self):
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000
        )

        self.head_offset_subscriber = self.node.create_subscription(SetJoints, '/internal/head_offset', self.head_offset_callback, 10)
        self.set_joints_subscriber = self.node.create_subscription(SetJoints, '/webots/set_joints', self.set_joints_callback, qos_profile)
        self.set_torques_subscriber = self.node.create_subscription(SetTorques, '/webots/set_torques', self.set_torques_callback, 10)
        
        self.current_joints_publisher = self.node.create_publisher(CurrentJoints, '/joint/current_joints', 10)
        self.imu_publisher = self.node.create_publisher(Imu, '/imu', 10)
        
        if self.device.camera:
            self.image_publisher = self.node.create_publisher(Image, '/camera/image', 10)
            self.camera_config_publisher = self.node.create_publisher(CameraConfig, '/camera/camera_config', 10)
        
        self.ball_position_publisher = self.node.create_publisher(Point, '/ball/position', 10)

    def _update_joint_directions(self):
        data = self.config_loader.get_data()
        name_to_id = {v: k for k, v in self.device.id_to_name.items()}
        
        self.joint_directions = {}
        for name, direction in data.items():
            if name in name_to_id:
                self.joint_directions[name_to_id[name]] = direction

    def head_offset_callback(self, msg):
        for joint in msg.joints:
            rad = math.radians(joint.position)
            if joint.id == 19:
                self.head_offset_yaw = rad
            elif joint.id == 20:
                self.head_offset_pitch = rad

    def set_joints_callback(self, msg):
        for joint in msg.joints:
            target_rad = math.radians(joint.position)
            
            # Apply Offset
            if joint.id == 19:
                target_rad += self.head_offset_yaw
            elif joint.id == 20:
                target_rad += self.head_offset_pitch

            # Apply Direction
            direction = self.joint_directions.get(joint.id, 1)
            target_rad *= direction

            self.device.set_joint_position(joint.id, target_rad)

    def set_torques_callback(self, msg):
        for joint_id in msg.ids:
            self.device.set_torque_limit(joint_id, msg.torque_enable)

    def publish_joints(self):
        msg = CurrentJoints()
        for joint_id in sorted(self.device.id_to_name.keys()):
            val_rad = self.device.get_joint_position(joint_id)
            
            direction = self.joint_directions.get(joint_id, 1)
            val_deg = math.degrees(val_rad * direction)

            joint = Joint()
            joint.id = int(joint_id)
            joint.position = float(val_deg)
            msg.joints.append(joint)
        
        self.current_joints_publisher.publish(msg)

    def publish_imu(self):
        if not self.device.imu:
            return
        
        msg = Imu()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        rpy = self.device.imu.getRollPitchYaw()
        cy = math.cos(rpy[2] * 0.5)
        sy = math.sin(rpy[2] * 0.5)
        cp = math.cos(rpy[1] * 0.5)
        sp = math.sin(rpy[1] * 0.5)
        cr = math.cos(rpy[0] * 0.5)
        sr = math.sin(rpy[0] * 0.5)

        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy

        if self.device.gyro:
            val = self.device.gyro.getValues()
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = val[0], val[1], val[2]
        
        if self.device.accel:
            val = self.device.accel.getValues()
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = val[0], val[1], val[2]

        self.imu_publisher.publish(msg)

    def publish_ground_truth(self):
        current_time = time.time()
        if (current_time - self.last_cheat_time) > 0.03: # 30Hz
            self.last_cheat_time = current_time
            
            if self.device.ball_node and self.device.robot_node and self.device.imu:
                ball_pos = self.device.ball_node.getPosition()
                robot_pos = self.device.robot_node.getPosition()
                yaw = self.device.imu.getRollPitchYaw()[2]

                lx, ly = Kinematics.calculate_ball_position_local(ball_pos, robot_pos, yaw)
                
                msg = Point()
                msg.x = lx
                msg.y = ly
                msg.z = 0.0
                self.ball_position_publisher.publish(msg)
                # print(f"ball x: {lx} ball y: {ly}", flush=True)
            else:
                if not self.device.ball_node:
                    print("ball device not found", flush=True)
                elif not self.device.robot_node:
                    print("robot device not found", flush=True)
                else:
                    print("imu device not found", flush=True)
        else:
            print("can't publish ball pos webots", flush=True)

    # ==========================
    # MAIN LOOP
    # ==========================
    def step(self):
        # 1. Spin ROS
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.step_counter += 1

        # 2. Check Config Reload (Every 2s approx)
        if self.step_counter % 200 == 0: # 200 * 8ms approx 1.6s
            if self.config_loader.check_reload():
                self._update_joint_directions()

        # 3. Publish Data
        self.publish_joints()
        self.publish_imu()
        self.publish_ground_truth()

        # 4. Camera (If active & interval met)
        if self.device.camera and (self.step_counter % self.device.camera_interval_step == 0):
            raw = self.device.camera.getImage()
            if raw:
                msg = Image()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                msg.height = self.device.camera.getHeight()
                msg.width = self.device.camera.getWidth()
                msg.encoding = "bgra8"
                msg.step = 4 * msg.width
                msg.data = raw
                self.image_publisher.publish(msg)