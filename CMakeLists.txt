cmake_minimum_required(VERSION 3.8)
project(gijitaiken_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_package(ninshiki_interfaces REQUIRED)
find_package(shisen_interfaces REQUIRED)
find_package(gyakuenki_interfaces REQUIRED)

include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(vision_lib
  src/gijitaiken_webots/node/vision_node.cpp
  src/gijitaiken_webots/process/image_process.cpp
)

target_include_directories(vision_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${cv_bridge_INCLUDE_DIRS}
)

ament_target_dependencies(vision_lib
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ninshiki_interfaces
  shisen_interfaces
  gyakuenki_interfaces
)

target_link_libraries(vision_lib
  ${OpenCV_LIBRARIES}
)

add_executable(vision_node_exec src/gijitaiken_webots/vision_main.cpp)

target_link_libraries(vision_node_exec vision_lib)

ament_target_dependencies(vision_node_exec
  rclcpp
  sensor_msgs
  cv_bridge
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  gijitaiken_webots/imu_main.py
  gijitaiken_webots/logic_main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS vision_lib vision_node_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config resource worlds protos
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()