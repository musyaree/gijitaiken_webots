import os
import mujoco
import numpy as np
from mujoco import MjModel, MjData

class Simulation:
    def __init__(self, model_path: str):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Path not found: {model_path}")

        print(f"Loading model from {model_path}", flush=True)
        self.model = MjModel.from_xml_path(model_path)
        self.data = MjData(self.model)
        
        self.frame = 0

    def set_control(self, name: str, value: float):
        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id != -1:
            self.data.ctrl[actuator_id] = value

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)
        self.frame += 1

    def get_unit(self, name: str):
        try:
            return self.data.sensor(name).data.copy()
        except Exception:
            return np.array([0.0, 0.0, 0.0])
        
    def get_odometry(self):
        return self.data.qpos[0], self.data.qpos[1]
    
    def get_joint_state(self, name: str):
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)

        if joint_id == -1:
            return 0.0, 0.0
        
        qpos_adr = self.model.jnt_qposadr[joint_id]
        dof_adr = self.model.jnt_dofadr[joint_id]

        position = self.data.qpos[qpos_adr]
        velocity = self.data.qpos[dof_adr]

        return position, velocity
    
    ''' 
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.frame = 0
    '''