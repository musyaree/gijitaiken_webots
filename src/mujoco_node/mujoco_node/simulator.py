import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor
import tf2_ros

import os
import time
from typing import Optional
import meshcat.transformations as tf
import mujoco
import numpy as np
import glfw
import traceback

from mujoco.glfw import glfw
from mujoco import MjModel, MjData, mjtObj, MjvCamera, MjvOption, MjvScene, MjrContext, mjr_render, mjv_updateScene

class Simulator:
    def __init__(self, model_dir: Optional[str] = None):
        if model_dir is None:
            model_dir = os.path.join(os.path.dirname(__file__) + "/model/")
        self.model_dir = model_dir

        self.model: MjModel = MjModel.from_xml_path(f"{model_dir}/scene.xml")
        self.data: MjData = MjData(self.model)

        joints = len(self.model.jnt_pos)
        self.dofs = [[k, self.model.jnt(k).name] for k in range(1, joints)]
        self.dofs_to_index = {dof: k for k, dof in self.dofs}

        self.t: float = 0.0
        self.dt: float = self.model.opt.timestep
        self.frame: int = 0
        self.data.ctrl[:] = 0

        self.window = None
        self.mjr_context = None
        self.mjv_scene = None
        self.mjv_camera = MjvCamera()
        self.mjv_option = MjvOption()
        
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0
        
        self.realtime = True

    def get_logger(self):
        return rclpy.logging.get_logger("mujoco_simulator_class")

    def init_viewer(self):        
        self.window = glfw.create_window(1200, 900, "MuJoCo Simulator", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Gagal membuat window GLFW")
        
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        mujoco.mjv_defaultCamera(self.mjv_camera)
        mujoco.mjv_defaultOption(self.mjv_option)
        self.mjv_scene = MjvScene(self.model, maxgeom=1000)
        
        self.mjr_context = MjrContext(self.model, 0)
        
        self._setup_mouse_callbacks()

    def _setup_mouse_callbacks(self):
        glfw.set_cursor_pos_callback(self.window, self.mouse_move_callback)
        glfw.set_mouse_button_callback(self.window, self.mouse_button_callback)
        glfw.set_scroll_callback(self.window, self.scroll_callback)
    
    def mouse_button_callback(self, window, button, action, mods):
        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
        glfw.get_cursor_pos(window) 
        
    def mouse_move_callback(self, window, xpos, ypos):
        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos
        if not (self.button_left or self.button_middle or self.button_right): return
        width, height = glfw.get_window_size(window)
        action = 0
        if self.button_right: action = mujoco.mjtMouse.mjMOUSE_MOVE_H
        elif self.button_left: action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else: action = mujoco.mjtMouse.mjMOUSE_ZOOM
        mujoco.mjv_moveCamera(self.model, action, dx / width, dy / height, self.mjv_scene, self.mjv_camera)

    def scroll_callback(self, window, xoffset, yoffset):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, -0.05 * yoffset, self.mjv_scene, self.mjv_camera)

    def is_viewer_running(self) -> bool:
        if self.window is None: return False
        return not glfw.window_should_close(self.window)

    def render_manual(self):
        if not self.is_viewer_running(): return
        try:
            viewport = mujoco.MjrRect(0, 0, 0, 0)
            glfw.get_framebuffer_size(self.window)
            viewport.width, viewport.height = glfw.get_framebuffer_size(self.window)
            mujoco.mjv_updateScene(self.model, self.data, self.mjv_option, None, self.mjv_camera, mujoco.mjtCatBit.mjCAT_ALL, self.mjv_scene)
            mujoco.mjr_render(viewport, self.mjv_scene, self.mjr_context)
            glfw.swap_buffers(self.window)
            glfw.poll_events()
        except Exception as e:
            self.get_logger().error(f"Error di render_manual: {e}")
            if self.window: glfw.set_window_should_close(self.window, True)
    
    def close_viewer(self):
        if self.window:
            glfw.destroy_window(self.window)
        self.get_logger().info("SIM: GLFW window dihancurkan.")

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def get_q(self, name: str) -> float:
        addr = self.model.jnt_qposadr[self.dofs_to_index[name]]
        return self.data.qpos[addr]

    def get_qdot(self, name: str) -> float:
        addr = self.model.jnt_dofadr[self.dofs_to_index[name]]
        return self.data.qvel[addr]

    def set_q(self, name: str, value: float) -> None:
        addr = self.model.jnt_qposadr[self.dofs_to_index[name]]
        self.data.qpos[addr] = value

    def get_actuator_index(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def set_control(self, name: str, value: float, reset: bool = False) -> None:
        actuator_idx = self.get_actuator_index(name)
        if actuator_idx != -1: 
            self.data.ctrl[actuator_idx] = value
        if reset:
            self.set_q(name, value)

    def get_gyro(self) -> np.ndarray:
        try:
            return self.data.sensor("gyro").data
        except mujoco.FatalError:
            self.get_logger().warn_once("Sensor 'gyro' tidak ditemukan di XML.")
            return np.array([0.0, 0.0, 0.0])

    def step(self) -> None:
        self.t = self.frame * self.dt
        mujoco.mj_step(self.model, self.data)
        self.frame += 1


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info("MuJoCo Simulator Node starting...")

        self.declare_parameter('use_fast_mode', False, 
            ParameterDescriptor(description="True: Jalankan headless (tanpa viewer) secepat mungkin untuk Optuna."))
        self.fast_mode = self.get_parameter('use_fast_mode').value

        # Path Model
        try:
            package_share_dir = get_package_share_directory('mujoco_node')
            model_dir_check = os.path.join(package_share_dir, 'model')
            if not os.path.isdir(model_dir_check):
                raise RuntimeError(f"FATAL: Folder model tidak ditemukan di: {model_dir_check}")
        except Exception as e:
            self.get_logger().error(f"FATAL: Gagal mendapatkan package share directory: {e}")
            raise 
        self.get_logger().info(f"Loading model from: {model_dir_check}")
        
        # Inisialisasi Viewer
        try:
            self.sim = Simulator(model_dir=model_dir_check)
            
            self.sim.init_viewer() 
            
            if not self.fast_mode:
                # Hubungkan keyboard
                # self.get_logger().info("Mode Normal: Menghubungkan callback keyboard...")
                self.key_states = {} 
                glfw.set_key_callback(self.sim.window, self.keyboard_callback)
            else:
                self.key_states = {} 

        except Exception as e:
            self.get_logger().error(f"GAGAL SAAT INISIALISASI: {e}")
            self.get_logger().error(traceback.format_exc())
            raise RuntimeError(f"Gagal inisialisasi: {e}")
        
        # Setup ROS 
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/unit', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) 
        
        if self.fast_mode:
            self.render_rate_hz = 1000.0 
            self.slowdown_factor = None 
            self.sim.realtime = False 
        else:
            self.render_rate_hz = 60.0 
            self.slowdown_factor = 1.0
            self.sim.realtime = True 
        
        self.timer = self.create_timer(1.0 / self.render_rate_hz, self.main_loop)

        realtime_steps = (1.0 / self.render_rate_hz) / self.sim.dt
        
        if self.slowdown_factor is None:
            self.physics_steps_per_render = int(realtime_steps)
        else:
            self.physics_steps_per_render = int( realtime_steps / self.slowdown_factor )

        if self.physics_steps_per_render < 1:
            self.physics_steps_per_render = 1

        '''
        self.get_logger().info(
            f"Timer diatur ke {self.render_rate_hz} Hz. "
            f"Mode Cepat: {self.fast_mode}. "
            f"Fisika akan disub-step: {self.physics_steps_per_render}x per loop.")
        '''
        
        rclpy.get_default_context().on_shutdown(self.on_shutdown_cleanup)
        
        self.get_logger().info("MuJoCo Simulator Node running.")

    def keyboard_callback(self, window, key, scancode, action, mods):
        if action == glfw.PRESS:
            self.key_states[key] = True
            if key == glfw.KEY_R:
                # self.get_logger().info("Executing reset...")
                self.sim.reset() 
        elif action == glfw.RELEASE:
            self.key_states[key] = False

    # Set joint
    def joint_callback(self, msg: JointState):
        for i in range(len(msg.name)):
            joint_name = msg.name[i]
            joint_position = msg.position[i]
            self.sim.set_control(joint_name, joint_position)

    def main_loop(self):
        if not rclpy.ok():
            return
            
        if not self.sim.is_viewer_running(): 
            self.get_logger().info("Simulator window closed. Shutting down node.")
            rclpy.shutdown()
            return

        if not self.fast_mode:
            twist_msg = Twist()
            if self.key_states.get(glfw.KEY_W) or self.key_states.get(glfw.KEY_UP):
                twist_msg.linear.x = 35.0
            elif self.key_states.get(glfw.KEY_S) or self.key_states.get(glfw.KEY_DOWN):
                twist_msg.linear.x = -35.0
            if self.key_states.get(glfw.KEY_A):
                twist_msg.linear.y = 35.0
                twist_msg.angular.z = np.deg2rad(0.03)
            elif self.key_states.get(glfw.KEY_D):
                twist_msg.linear.y = -35.0
                twist_msg.angular.z = np.deg2rad(-0.03)
            if self.key_states.get(glfw.KEY_LEFT):
                twist_msg.angular.z = np.deg2rad(20.0) 
            elif self.key_states.get(glfw.KEY_RIGHT):
                twist_msg.angular.z = np.deg2rad(-20.0)
            self.cmd_vel_pub.publish(twist_msg)
        
        # PUBLISH SENSOR (IMU) 
        try:
            gyro_data = self.sim.get_gyro()
            if gyro_data is not None and len(gyro_data) == 3:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link" 
                imu_msg.angular_velocity.x = float(gyro_data[0])
                imu_msg.angular_velocity.y = float(gyro_data[1])
                imu_msg.angular_velocity.z = float(gyro_data[2])
                self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(
                f"Tidak bisa membaca sensor IMU/Gyro: {e}. " +
                "Pastikan sensor 'gyro' ada di XML Anda.", 
                throttle_duration_sec=5.0) 

        # PUBLISH TF (POSISI ROBOT)
        try:
            qpos = self.sim.data.qpos
            torso_pos = qpos[0:3]; torso_quat = qpos[3:7]
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg(); t.header.frame_id = 'world'; t.child_frame_id = 'base_link'
            t.transform.translation.x = torso_pos[0]; t.transform.translation.y = torso_pos[1]; t.transform.translation.z = torso_pos[2]
            t.transform.rotation.x = torso_quat[1]; t.transform.rotation.y = torso_quat[2]; t.transform.rotation.z = torso_quat[3]; t.transform.rotation.w = torso_quat[0]
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().warn(f"Gagal publish TF: {e}", throttle_duration_sec=5.0)

        # Fisika & Render
        try:
            for _ in range(self.physics_steps_per_render):
                self.sim.step()
            
            self.sim.render_manual()
        except Exception as e:
            if "window has been closed" in str(e).lower():
                self.get_logger().info("MuJoCo window closed by user.")
                rclpy.shutdown()
            else:
                self.get_logger().error(f"Error in main loop step/render: {e}")
                self.get_logger().error(traceback.format_exc())
                rclpy.shutdown()

    def on_shutdown_cleanup(self):
        self.get_logger().info("Hook shutdown dipanggil. Membersihkan viewer...")
        if self.sim:
            self.sim.close_viewer()


def main(args=None):
    try:
        if not glfw.init():
             print("FATAL: Gagal menginisialisasi GLFW di 'main'.")
             return
    except Exception as e:
        print(f"FATAL: Panggilan glfw.init() gagal total: {e}")
        return
    
    rclpy.init(args=args)
    
    node = None
    try:
        node = SimulatorNode()
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Spin interrupted by KeyboardInterrupt.")
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Node shutdown due to unhandled exception: {e}")
        else:
            print(f"Error during Node initialization: {e}")
    finally:
        if node:
            node.on_shutdown_cleanup() 
            if rclpy.ok():
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        glfw.terminate()
        print("rclpy shutdown complete. GLFW terminated.")

if __name__ == '__main__':
    main()