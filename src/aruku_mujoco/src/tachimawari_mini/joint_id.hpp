#ifndef TACHIMAWARI__JOINT__MODEL__JOINT_ID_HPP_
#define TACHIMAWARI__JOINT__MODEL__JOINT_ID_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <array>

#include "keisan/angle.hpp"

namespace tachimawari::joint
{

class JointId
{
public:
  enum : uint8_t {
    // head motors
    NECK_YAW = 19,
    NECK_PITCH = 20,

    // left arm motors
    LEFT_SHOULDER_PITCH = 2,
    LEFT_SHOULDER_ROLL = 4,
    LEFT_ELBOW = 6,
    // LEFT_SHOULDER_YAW = 24,
    // LEFT_GRIPPER = 22,

    // right arm motors
    RIGHT_SHOULDER_PITCH = 1,
    RIGHT_SHOULDER_ROLL = 3,
    RIGHT_ELBOW = 5,
    // RIGHT_SHOULDER_YAW = 23,
    // RIGHT_GRIPPER = 21,

    // left leg motors
    LEFT_HIP_YAW = 8,
    LEFT_HIP_ROLL = 10,
    LEFT_HIP_PITCH = 12,
    LEFT_KNEE = 14,
    LEFT_ANKLE_PITCH = 16,
    LEFT_ANKLE_ROLL = 18,

    // right leg motors
    RIGHT_HIP_YAW = 7,
    RIGHT_HIP_ROLL = 9,
    RIGHT_HIP_PITCH = 11,
    RIGHT_KNEE = 13,
    RIGHT_ANKLE_PITCH = 15,
    RIGHT_ANKLE_ROLL = 17,
  };

  static const std::map<std::string, uint8_t> by_name;
  static const std::map<uint8_t, std::string> by_id;
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__MODEL__JOINT_ID_HPP_