#ifndef TACHIMAWARI__JOINT__MODEL__JOINT_HPP_
#define TACHIMAWARI__JOINT__MODEL__JOINT_HPP_

#include "keisan/angle.hpp"  
#include "tachimawari_mini/joint_id.hpp"

using namespace keisan::literals;

namespace tachimawari::joint
{

class Joint
{
public:
  explicit Joint(uint8_t joint_id, keisan::Angle<double> target_position = 0.0_deg)
  : id(joint_id), position(target_position)
  {
  }

  uint8_t get_id() const { return id; }

  void set_position(const keisan::Angle<double> & target_position) { position = target_position; }
  const keisan::Angle<double> & get_position() const { return position; }

private:
  uint8_t id;
  keisan::Angle<double> position; 
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__MODEL__JOINT_HPP_