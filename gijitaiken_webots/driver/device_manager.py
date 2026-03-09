class DeviceManager:
    """
    Wrapper untuk menangani akses ke device Webots (Motor, Sensor, Kamera, IMU).
    Memisahkan logika 'Hardware' dari logika 'ROS'.
    """
    def __init__(self, webots_robot):
        self.robot = webots_robot
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.id_to_name = {
            19: "head_yaw",
            20: "head_pitch",

            2: "left_shoulder_pitch",
            4: "left_shoulder_roll",
            24: "left_shoulder_yaw",
            6: "left_elbow",
            22: "left_gripper",

            1: "right_shoulder_pitch",
            3: "right_shoulder_roll",
            23: "right_shoulder_yaw",
            5: "right_elbow",
            21: "right_gripper",

            8: "left_hip_yaw",
            10: "left_hip_roll",
            12: "left_hip_pitch",
            14: "left_knee",
            16: "left_ankle_pitch",
            18: "left_ankle_roll",

            7: "right_hip_yaw",
            9: "right_hip_roll",
            11: "right_hip_pitch",
            13: "right_knee",
            15:"right_ankle_pitch",
            17: "right_ankle_roll"
        }
        
        self.motors = {}
        self.sensors = {}
        self.stored_params = {}

        self.imu = None
        self.gyro = None
        self.accel = None
        self.camera = None
        self.camera_interval_step = 0
        
        self.ball_node = self.robot.getFromDef("BALL")
        self.robot_node = self.robot.getFromDef("HIRO")
        if self.robot_node is None:
            self.robot_node = self.robot.getSelf()

        self._init_motors_and_sensors()
        self._init_imu()
        # self._init_camera()

    def _init_motors_and_sensors(self):
        missing_devices = []
        count_motors = 0
        
        for joint_id, name in self.id_to_name.items():
            motor = self.robot.getDevice(name)
            if motor:
                self.motors[joint_id] = motor
                self.stored_params[joint_id] = {'max_torque': motor.getMaxTorque()}
                motor.setVelocity(motor.getMaxVelocity())
            
            sensor_name = name + "_sensor"
            sensor = self.robot.getDevice(sensor_name)
            if not sensor:
                sensor = self.robot.getDevice(name)
            if sensor:
                sensor.enable(self.timestep)
                self.sensors[joint_id] = sensor
                count_motors += 1
        
        if missing_devices:
            print(f"[DEVICE] {len(missing_devices)} devices missing: {missing_devices}")
        else:
            print(f"[DEVICE] All {count_motors} motors initialized.")

    def _init_imu(self):
        self.imu = self.robot.getDevice('imu')
        if self.imu:
            self.imu.enable(self.timestep)
        
        self.gyro = self.robot.getDevice('gyro')
        if self.gyro:
            self.gyro.enable(self.timestep)
        
        self.accel = self.robot.getDevice('accelerometer')
        if self.accel:
            self.accel.enable(self.timestep)

        status = []
        if self.imu:
            status.append("IMU")
        if self.gyro:
            status.append("GYRO")
        if self.accel:
            status.append("ACCEL")
        print(f"[SENSOR] Active: {', '.join(status)}")

    def _init_camera(self, target_fps=30.0):
        """Logic ini disalin PERSIS dari kode lama Anda"""
        
        # Cari device (Kode lama pakai "camera", kita buat robust sedikit)
        self.camera = self.robot.getDevice("camera_sensor")
            
        if self.camera:
            if target_fps > 0:
                # Rumus Kode Lama:
                # self.camera_interval_step = int(1000 / (self.target_fps * self.timestep))
                self.camera_interval_step = int(1000 / (target_fps * self.timestep))
            else:
                self.camera_interval_step = 1
            
            if self.camera_interval_step < 1: 
                self.camera_interval_step = 1
            
            # Rumus Kode Lama:
            # camera_period_ms = self.camera_interval_step * self.timestep
            camera_period_ms = self.camera_interval_step * self.timestep
            
            # Enable
            self.camera.enable(camera_period_ms)
            
            print(f"[CAMERA] Active: {target_fps} FPS. Period: {camera_period_ms}ms (Every {self.camera_interval_step} steps)", flush=True)
            return self.camera_interval_step
            
        print("[CAMERA] Device Not Found!", flush=True)
        return 0

    def set_joint_position(self, joint_id, position_rad):
        if joint_id in self.motors:
            self.motors[joint_id].setPosition(position_rad)

    def set_torque_limit(self, joint_id, enable):
        if joint_id in self.stored_params and joint_id in self.motors:
            limit = self.stored_params[joint_id]['max_torque'] if enable else 0.0
            self.motors[joint_id].setAvailableTorque(limit)

    def get_joint_position(self, joint_id):
        if joint_id in self.sensors:
            return self.sensors[joint_id].getValue()
        return 0.0