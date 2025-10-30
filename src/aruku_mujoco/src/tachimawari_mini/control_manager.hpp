#ifndef TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_
#define TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_

#include <string>
#include <vector>
#include "tachimawari_mini/joint.hpp"

namespace tachimawari::control
{

class ControlManager
{
public:
  ControlManager() = default;
  virtual ~ControlManager() = default;

  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool sync_write_packet(const std::vector<joint::Joint> & joints) = 0;
  
  virtual int read_packet(uint8_t id, uint16_t address, int data_length = 1) { return 0; }
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_