#ifndef ARUKU_MINI__WALKING_MANAGER_HPP_
#define ARUKU_MINI__WALKING_MANAGER_HPP_

#include "aruku_mini/kinematic.hpp"
#include "jitsuyo/config.hpp"
#include "tachimawari_mini/control_manager.hpp"
#include "tachimawari_mini/joint.hpp"
#include <memory>
#include <string>
#include <vector>

namespace aruku::mini
{

class WalkingManager
{
public:
  // Konstruktor akan menerima pointer ke ControlManager
  explicit WalkingManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager);

  bool set_config(const nlohmann::json & kinematic_data);

  // Perintah utama untuk berjalan dan berhenti
  void run(double x_move, double y_move, double a_move);
  void stop();

  void update_gyro(const keisan::Vector<3> & gyro);

  bool process();

  bool is_running() const;

private:
  std::shared_ptr<tachimawari::control::ControlManager> control_manager;

  Kinematic kinematic;

  // Wadah untuk menyimpan hasil sudut sendi
  std::vector<tachimawari::joint::Joint> joints;
};

}  // namespace aruku::mini

#endif  // ARUKU_MINI__WALKING_MANAGER_HPP_