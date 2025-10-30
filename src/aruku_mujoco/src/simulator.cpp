#include "simulator.hpp"
#include "aruku_mini/aruku_mujoco_node.hpp" 
#include <iostream>

// Status mouse
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    ArukuMujocoNode* node = (ArukuMujocoNode*)glfwGetWindowUserPointer(window);
    if (!node) return;

    Simulator* sim = node->get_simulator().get();
    if (!sim) return;

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(sim->model_, sim->data_);
        mj_forward(sim->model_, sim->data_);
    }

    node->handle_keyboard(key, act);
}

void mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
{
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    
    ArukuMujocoNode* node = (ArukuMujocoNode*)glfwGetWindowUserPointer(window);
    if (!node) return;
    Simulator* sim = node->get_simulator().get();
    if (!sim) return;

    mjtMouse action;
    if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else action = mjMOUSE_ZOOM;
    
    mjv_moveCamera(sim->model_, action, dx/height, dy/height, &sim->scn_, &sim->cam_);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    ArukuMujocoNode* node = (ArukuMujocoNode*)glfwGetWindowUserPointer(window);
    if (!node) return;
    Simulator* sim = node->get_simulator().get();
    if (!sim) return;
    
    mjv_moveCamera(sim->model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &sim->scn_, &sim->cam_);
}


Simulator::Simulator(const std::string & model_path)
{
    char error[1000] = "Could not load model";
    model_ = mj_loadXML(model_path.c_str(), NULL, error, 1000);
    if (!model_) { mju_error("Load model error: %s", error); }
    data_ = mj_makeData(model_);

    if (!glfwInit()) { mju_error("Could not initialize GLFW"); }
    window_ = glfwCreateWindow(1280, 720, "Aruku Mujoco", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);
    
    glfwSetKeyCallback(window_, keyboard_callback);
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, mouse_move_callback);
    glfwSetScrollCallback(window_, scroll_callback);
}

Simulator::~Simulator()
{
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    mj_deleteData(data_);
    mj_deleteModel(model_);
    glfwTerminate();
}

void Simulator::step()
{
    double target_time = data_->time + 0.008;
    while(data_->time < target_time) {
        mj_step(model_, data_);
    }
}

void Simulator::render()
{
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    mjv_updateScene(model_, data_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void Simulator::reset()
{
  mj_resetData(model_, data_);
}

bool Simulator::is_window_open() { return !glfwWindowShouldClose(window_); }
GLFWwindow* Simulator::get_window() { return window_; }

void Simulator::set_control(const std::string & actuator_name, double value)
{
    int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id != -1) { data_->ctrl[actuator_id] = value; }
}

std::vector<double> Simulator::get_gyro()
{
    int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, "gyro");
    if (sensor_id != -1) {
        int adr = model_->sensor_adr[sensor_id];
        int dim = model_->sensor_dim[sensor_id];
        std::vector<double> gyro_data;
        gyro_data.assign(data_->sensordata + adr, data_->sensordata + adr + dim);
        return gyro_data;
    }
    return {};
}