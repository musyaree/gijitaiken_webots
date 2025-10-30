cmake_minimum_required(VERSION 3.8)
project(aruku_mujoco)

# Setup Compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Cari Dependensi ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Cari Dependensi Eksternal (MuJoCo & GLFW)
find_path(MUJOCO_INCLUDE_DIR NAMES mujoco/mujoco.h PATHS ENV MUJOCO_PATH PATH_SUFFIXES include)
find_library(MUJOCO_LIBRARY NAMES mujoco PATHS ENV MUJOCO_PATH PATH_SUFFIXES lib)
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo not found! Please set the MUJOCO_PATH environment variable.")
endif()

# Tambahkan header MuJoCo ke include path
include_directories(${MUJOCO_INCLUDE_DIR})

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# Kumpulkan semua file .cpp secara otomatis dari subfolder src
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

# Buat executable (node) dari semua file sumber
add_executable(aruku_mujoco_node ${SOURCE_FILES})

# Beritahu compiler di mana menemukan file .hpp kita
target_include_directories(aruku_mujoco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# Tautkan (link) node kita dengan semua library yang dibutuhkan
target_link_libraries(aruku_mujoco_node
  PUBLIC
    # Dependensi ROS 2
    ament_index_cpp::ament_index_cpp
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_generator_c
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp

    # Dependensi Eksternal
    ${MUJOCO_LIBRARY}
    glfw
    ${OPENGL_LIBRARIES}
)

# Aturan Instalasi
install(TARGETS aruku_mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Ament Boilerplate
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()