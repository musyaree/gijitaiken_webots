cmake_minimum_required(VERSION 3.8)
project(gijitaiken_webots)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================
# 1. FIND PACKAGES
# =========================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Custom Interfaces
find_package(ninshiki_interfaces REQUIRED)
find_package(shisen_interfaces REQUIRED)
find_package(gyakuenki_interfaces REQUIRED)

# =========================================
# 2. C++ BUILD (VISION FEATURE)
# =========================================
include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# A. Library (Core Logic)
# Pastikan path ini sesuai dengan tree folder Anda yang terakhir
add_library(vision_lib
  src/gijitaiken_webots/node/vision_node.cpp
  src/gijitaiken_webots/process/image_process.cpp
)

target_include_directories(vision_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${cv_bridge_INCLUDE_DIRS} # Tambahkan juga di sini
)

# [FIX] Gunakan ament_target_dependencies untuk package ROS
ament_target_dependencies(vision_lib
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  ninshiki_interfaces
  shisen_interfaces
  gyakuenki_interfaces
)

# [FIX] Gunakan target_link_libraries HANYA untuk OpenCV (External Lib)
target_link_libraries(vision_lib
  ${OpenCV_LIBRARIES}
)

# B. Executable (Main Entry)
add_executable(vision_node_exec src/gijitaiken_webots/vision_main.cpp)

# Link ke library internal kita
target_link_libraries(vision_node_exec vision_lib)

# Executable juga perlu tahu dependency ROS agar header rclcpp terbaca
ament_target_dependencies(vision_node_exec
  rclcpp
  sensor_msgs
  cv_bridge
)

# =========================================
# 3. PYTHON INSTALLATION
# =========================================
ament_python_install_package(${PROJECT_NAME})

# Install Python Executables
install(PROGRAMS
  gijitaiken_webots/imu_main.py
  gijitaiken_webots/logic_main.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================
# 4. INSTALL C++ TARGETS
# =========================================
install(TARGETS vision_lib vision_node_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# =========================================
# 5. INSTALL ASSETS
# =========================================
install(DIRECTORY launch config resource worlds protos
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()