#ifndef ARUKU_MINI__WALKING_MANAGER_HPP_
#define ARUKU_MINI__WALKING_MANAGER_HPP_

#include "aruku_mini/kinematic.hpp"
#include "jitsuyo/config.hpp"
#include "tachimawari_mini/joint.hpp" 

#include <memory>
#include <string>
#include <vector>

namespace tachimawari
{
  namespace joint
  {
    class Joint;
  }
}

namespace aruku::mini
{

class WalkingManager
{
public:
  WalkingManager();

  bool set_config(const nlohmann::json & kinematic_data);

  void run(double x_move, double y_move, double a_move);

  void stop();

  void update_gyro(const keisan::Vector<3> & gyro);

  bool process();

  bool is_running() const;

  const std::vector<tachimawari::joint::Joint> & get_joints() const;

private:
  Kinematic kinematic;

  std::vector<tachimawari::joint::Joint> joints;
};

}  // namespace aruku::mini

#endif  // ARUKU_MINI__WALKING_MANAGER_HPP_