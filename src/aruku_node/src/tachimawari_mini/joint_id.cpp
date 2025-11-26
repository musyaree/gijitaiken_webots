#include "tachimawari_mini/joint_id.hpp"

namespace tachimawari::joint
{

const std::map<std::string, uint8_t> JointId::by_name = {
  {"neck_yaw", NECK_YAW},
  {"neck_pitch", NECK_PITCH},
  {"left_shoulder_pitch", LEFT_SHOULDER_PITCH},
  {"left_shoulder_roll", LEFT_SHOULDER_ROLL},
  {"left_elbow", LEFT_ELBOW},
  {"right_shoulder_pitch", RIGHT_SHOULDER_PITCH},
  {"right_shoulder_roll", RIGHT_SHOULDER_ROLL},
  {"right_elbow", RIGHT_ELBOW},
  {"left_hip_yaw", LEFT_HIP_YAW},
  {"left_hip_roll", LEFT_HIP_ROLL},
  {"left_hip_pitch", LEFT_HIP_PITCH},
  {"left_knee", LEFT_KNEE},
  {"left_ankle_pitch", LEFT_ANKLE_PITCH},
  {"left_ankle_roll", LEFT_ANKLE_ROLL},
  {"right_hip_yaw", RIGHT_HIP_YAW},
  {"right_hip_roll", RIGHT_HIP_ROLL},
  {"right_hip_pitch", RIGHT_HIP_PITCH},
  {"right_knee", RIGHT_KNEE},
  {"right_ankle_pitch", RIGHT_ANKLE_PITCH},
  {"right_ankle_roll", RIGHT_ANKLE_ROLL}
};

const std::map<uint8_t, std::string> JointId::by_id = {
  {NECK_YAW, "neck_yaw"},
  {NECK_PITCH, "neck_pitch"},
  {LEFT_SHOULDER_PITCH, "left_shoulder_pitch"},
  {LEFT_SHOULDER_ROLL, "left_shoulder_roll"},
  {LEFT_ELBOW, "left_elbow"},
  {RIGHT_SHOULDER_PITCH, "right_shoulder_pitch"},
  {RIGHT_SHOULDER_ROLL, "right_shoulder_roll"},
  {RIGHT_ELBOW, "right_elbow"},
  {LEFT_HIP_YAW, "left_hip_yaw"},
  {LEFT_HIP_ROLL, "left_hip_roll"},
  {LEFT_HIP_PITCH, "left_hip_pitch"},
  {LEFT_KNEE, "left_knee"},
  {LEFT_ANKLE_PITCH, "left_ankle_pitch"},
  {LEFT_ANKLE_ROLL, "left_ankle_roll"},
  {RIGHT_HIP_YAW, "right_hip_yaw"},
  {RIGHT_HIP_ROLL, "right_hip_roll"},
  {RIGHT_HIP_PITCH, "right_hip_pitch"},
  {RIGHT_KNEE, "right_knee"},
  {RIGHT_ANKLE_PITCH, "right_ankle_pitch"},
  {RIGHT_ANKLE_ROLL, "right_ankle_roll"}
};

}  // namespace tachimawari::joint