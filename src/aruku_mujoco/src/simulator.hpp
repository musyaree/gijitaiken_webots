#ifndef ARUKU_MUJOCO__SIMULATOR_HPP_
#define ARUKU_MUJOCO__SIMULATOR_HPP_

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <vector>

class Simulator
{
public:
  explicit Simulator(const std::string & model_path);
  ~Simulator();

  void step();
  void render();
  void reset();
  bool is_window_open();
  GLFWwindow* get_window();

  void set_control(const std::string & actuator_name, double value);
  std::vector<double> get_gyro();

  mjModel* model_;
  mjData* data_;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  

private:
  GLFWwindow* window_;
  mjrContext con_;
};

#endif // ARUKU_MUJOCO__SIMULATOR_HPP_