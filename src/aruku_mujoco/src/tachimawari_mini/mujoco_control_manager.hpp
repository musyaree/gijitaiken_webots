#ifndef TACHIMAWARI_MINI__MUJOCO_CONTROL_MANAGER_HPP_
#define TACHIMAWARI_MINI__MUJOCO_CONTROL_MANAGER_HPP_

#include "tachimawari_mini/control_manager.hpp" 
#include "simulator.hpp"                        
#include <map>
#include <string>
#include <vector>
#include <memory>

namespace tachimawari::control
{

class MujocoControlManager : public ControlManager
{
public:
  explicit MujocoControlManager(std::shared_ptr<Simulator> simulator);
  virtual ~MujocoControlManager() = default; 

  bool connect() override;
  void disconnect() override;
  bool sync_write_packet(const std::vector<joint::Joint> & joints) override;

  int read_packet(uint8_t id, uint16_t address, int data_length = 1) override;

private:
  std::shared_ptr<Simulator> simulator_;

  // Peta untuk menerjemahkan ID sendi (uint8_t) ke nama aktuator (string)
  std::map<uint8_t, std::string> joint_id_to_name;
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI_MINI__MUJOCO_CONTROL_MANAGER_HPP_