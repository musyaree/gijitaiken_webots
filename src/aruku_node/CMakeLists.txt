cmake_minimum_required(VERSION 3.8)
# nama projek
project(aruku_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#  package lain yg dibutuhkan
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)   

# executable file
add_executable(aruku_node  
  src/main.cpp
  src/aruku_node.cpp         

  src/aruku_mini/walking_manager.cpp
  src/aruku_mini/kinematic.cpp
  src/tachimawari_mini/joint_id.cpp
  src/keisan/angle.cpp
  src/keisan/constant.cpp
  src/keisan/matrix.cpp
  src/keisan/point_3.cpp
)

target_include_directories(aruku_node PUBLIC U
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

ament_target_dependencies(aruku_node 
  PUBLIC
    "rclcpp"
    "geometry_msgs"
    "sensor_msgs"
    "ament_index_cpp"
)


install(TARGETS aruku_node 
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()