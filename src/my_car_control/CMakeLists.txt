cmake_minimum_required(VERSION 3.8)
project(my_car_control)

# =======================
# Compiler setup
# =======================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =======================
# Dependencies
# =======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(tf2_ros REQUIRED)  # chỉ bật nếu thực sự dùng tf2_ros broadcaster

# =======================
# Executable: Stanley Controller Node
# =======================
add_executable(stanley_controller src/stanley_controller.cpp)
ament_target_dependencies(stanley_controller
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(smooth_cmd_vel src/smooth_cmd_vel.cpp)
ament_target_dependencies(smooth_cmd_vel rclcpp geometry_msgs)
install(TARGETS
  smooth_cmd_vel
  DESTINATION lib/${PROJECT_NAME})







target_include_directories(stanley_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(stanley_controller PUBLIC cxx_std_17)

# =======================
# Install binaries
# =======================
install(TARGETS stanley_controller
  DESTINATION lib/${PROJECT_NAME})

# =======================
# Install data files (CSV, launch, config)
# =======================
# 1. Cài file path.csv (để node có thể load)
install(FILES path.csv
  DESTINATION share/${PROJECT_NAME})

# 2. Nếu có thư mục launch/ hoặc config/

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# =======================
# Package export
# =======================
ament_package()
