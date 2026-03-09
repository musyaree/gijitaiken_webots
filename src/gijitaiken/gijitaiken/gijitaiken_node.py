import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

import mujoco.viewer

from tachimawari_interfaces.msg import SetJoints
from aruku_interfaces.msg import SetWalking
from aruku_interfaces.msg import Point2
from kansei_interfaces.msg import Unit
from sensor_msgs.msg import JointState
from .simulation import Simulation

class GijitaikenNode(Node):
    def __init__(self):
        super().__init__('gijitaiken_node')

        self.declare_parameter('joint_map_config', 'joint_map.yaml')
        config_file = self.get_parameter('joint_map_config').value

        package_path = get_package_share_directory('gijitaiken')

        try:
            yaml_path = os.path.join(package_path, 'config', config_file)

            with open(yaml_path, 'r') as file:
                config_data = yaml.safe_load(file)
                self.id_to_name = {
                    int(motor_id) : motor_name
                    for motor_id, motor_name in config_data['joint_map'].items()
                }
        
            # self.get_logger().info(f"Loaded mapping for {len(self.id_to_name)} joints.")
        except Exception as e:
            self.get_logger().error(f"Failed to load yaml: {e}")
            raise
        
        # simulator
        self.declare_parameter('model_file', 'scene.xml')
        model_filename = self.get_parameter('model_file').value

        model_path = os.path.join(package_path, 'model', model_filename)

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.sim = Simulation(model_path)

        self.viewer = mujoco.viewer.launch_passive(
            self.sim.model, 
            self.sim.data,
            show_left_ui=True, 
            show_right_ui=True
        )

        # ros2 setup
        self.set_joints_subscriber = self.create_subscription(SetJoints, '/joint/set_joints', self.set_joints_callback, 10)
        self.unit_publisher = self.create_publisher(Unit, "/imu/unit", 10)
        self.odometry_publisher = self.create_publisher(Point2, "/walking/set_odometry", 10)
        self.joint_states_publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.create_timer(0.01, self.loop)

    def set_joints_callback(self, msg):
        for joint in msg.joints:
            joint_id = joint.id
            target_pos = joint.position

            if joint_id in self.id_to_name:
                joint_name = self.id_to_name[joint_id]
                self.sim.set_control(joint_name, target_pos)
            else:
                self.get_logger().warn(f"Unknown Id {joint_id}")
                pass

    def loop(self):
        if not self.viewer.is_running():
            raise SystemExit

        for _ in range(5):
            self.sim.step()

        self.viewer.sync()

        self.publish_imu()
        self.publish_odometry()
        self.publish_joint_states()

    def publish_imu(self):
        gyro = self.sim.get_unit("gyro_sensor")
        accel = self.sim.get_unit("accel_sensor")

        msg = Unit()

        # gyro
        msg.gyro.roll = float(gyro[0])
        msg.gyro.pitch = float(gyro[1])
        msg.gyro.yaw = float(gyro[2])

        # accelero
        msg.accelero.x = float(accel[0])
        msg.accelero.y = float(accel[1])
        msg.accelero.z = float(accel[2])

        self.unit_publisher.publish(msg)

    def publish_odometry(self):
        x, y = self.sim.get_odometry()

        msg = Point2()
        msg.x = float(x)
        msg.y = float(y)

        self.odometry_publisher.publish(msg)

    def publish_joint_states(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = []
        msg.position = []
        msg.velocity = []
        # msg.effort = []

        for joint_id, joint_name in self.id_to_name.items():
            pos, vel = self.sim.get_joint_state(joint_name)

            msg.name.append(joint_name)
            msg.position.append(float(pos))
            msg.velocity.append(float(vel))
        
        self.joint_states_publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, "viewer"):
            self.viewer.close()
        super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    node = GijitaikenNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        
            