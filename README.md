# 🛰️ ROS2 Workspace for STM32 micro-ROS Controller

This repository contains my **ROS 2 workspace (`ros2_ws`)** for a custom STM32-based controller integrating:
- 🎮 **Joystick / Stanlley input**
- 📡 **RTK-GPS (u-blox NEO-M8P)**
- 🧭 **IMU sensor (fake sensor)**
- 🔗 **micro-ROS communication**

The system enables high-precision control and localization for robotics or autonomous vehicles through STM32 running micro-ROS, communicating with ROS 2 on a host computer.

---

## ⚙️ Prerequisites

### 1. ROS 2 environment
Make sure you have ROS 2 (e.g. **Humble**, **Iron**, or **Jazzy**) installed.

Example for ROS 2 Humble:
```bash
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool git
sudo rosdep init
rosdep update


## Clone (or use your local workspace):
cd ~/ros2_ws

## Make sure your src/ folder contains all packages (e.g. micro_ros_setup, firmware, your custom packages).

colcon build
source install/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 launch my_car_control stanley_with stanley_with_teleop.launch.py 
ros2 launch my_localization fake_sensors.launch.py 
ros2 launch my_localization localization.launch.py 