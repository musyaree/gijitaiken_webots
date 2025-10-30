#include "tachimawari_mini/mujoco_control_manager.hpp"
#include "tachimawari_mini/joint_id.hpp"
#include <iostream>
#include <vector>

namespace tachimawari::control
{

MujocoControlManager::MujocoControlManager(std::shared_ptr<Simulator> simulator)
: simulator_(simulator)
{
    if (!simulator_) {
        std::cerr << "MujocoControlManager Error: Simulator pointer is null!" << std::endl;
        return;
    }

    // Peta dari JointId enum ke nama string aktuator di MuJoCo
    joint_id_to_name[joint::JointId::RIGHT_HIP_YAW] = "right_hip_yaw";
    joint_id_to_name[joint::JointId::RIGHT_HIP_ROLL] = "right_hip_roll";
    joint_id_to_name[joint::JointId::RIGHT_HIP_PITCH] = "right_hip_pitch";
    joint_id_to_name[joint::JointId::RIGHT_KNEE] = "right_knee";
    joint_id_to_name[joint::JointId::RIGHT_ANKLE_PITCH] = "right_ankle_pitch";
    joint_id_to_name[joint::JointId::RIGHT_ANKLE_ROLL] = "right_ankle_roll";
    joint_id_to_name[joint::JointId::LEFT_HIP_YAW] = "left_hip_yaw";
    joint_id_to_name[joint::JointId::LEFT_HIP_ROLL] = "left_hip_roll";
    joint_id_to_name[joint::JointId::LEFT_HIP_PITCH] = "left_hip_pitch";
    joint_id_to_name[joint::JointId::LEFT_KNEE] = "left_knee";
    joint_id_to_name[joint::JointId::LEFT_ANKLE_PITCH] = "left_ankle_pitch";
    joint_id_to_name[joint::JointId::LEFT_ANKLE_ROLL] = "left_ankle_roll";
    joint_id_to_name[joint::JointId::RIGHT_SHOULDER_PITCH] = "right_shoulder_pitch";
    joint_id_to_name[joint::JointId::RIGHT_SHOULDER_ROLL] = "right_shoulder_roll";
    joint_id_to_name[joint::JointId::RIGHT_ELBOW] = "right_elbow";
    joint_id_to_name[joint::JointId::LEFT_SHOULDER_PITCH] = "left_shoulder_pitch";
    joint_id_to_name[joint::JointId::LEFT_SHOULDER_ROLL] = "left_shoulder_roll";
    joint_id_to_name[joint::JointId::LEFT_ELBOW] = "left_elbow";
    joint_id_to_name[joint::JointId::NECK_YAW] = "head_yaw";
    joint_id_to_name[joint::JointId::NECK_PITCH] = "head_pitch";
}

bool MujocoControlManager::connect() { return simulator_ != nullptr; }
void MujocoControlManager::disconnect() {}

bool MujocoControlManager::sync_write_packet(const std::vector<joint::Joint> & joints)
{
    if (!simulator_) return false;
    for (const auto & joint : joints) {
        auto it = joint_id_to_name.find(joint.get_id());
        if (it != joint_id_to_name.end()) {
            simulator_->set_control(it->second, joint.get_position().radian());
        }
    }
    return true;
}

int MujocoControlManager::read_packet(uint8_t id, uint16_t address, int data_length)
{
    return 0;
}

} // namespace tachimawari::control