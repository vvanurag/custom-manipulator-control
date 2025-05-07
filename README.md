# 🦾 custom_arm — A Minimal ROS 2 Simulated Manipulator

## 📖 Overview

This project contains a minimal, custom-built 3-link robotic arm with 1 end-effector, implemented in **ROS 2 Humble**, with full simulation support in **RViz2**.

It is built from scratch using `xacro`, and tested with `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2.

## 🚀 Features

- Pure Xacro-based URDF
- 3 revolute joints + 1 end-effector
- Interactive joint sliders
- RViz2 integration with saved config
- Built cleanly in an isolated workspace: `customArm_ws`

---

## 🧰 Requirements

- Ubuntu 22.04
- ROS 2 Humble (`sudo apt install ros-humble-desktop`)
- ✅ Avoid installing `gazebo`/`ros-gz` if using other robots (like UR10e)
- These packages must be installed:

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2

## 🧱 Build Instructions
# Clone into a ROS 2 workspace
mkdir -p ~/customArm_ws/src
cd ~/customArm_ws/src
git clone https://github.com/vvanurag/custom-manipulator-control.git

# Build
cd ~/customArm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source overlay
source install/setup.bash


## 🏗️ Launch the Simulation
ros2 launch custom_arm display.launch.py

## 🩺 Troubleshooting
- ❌ RViz shows "No TF"
- 🔍 Set Fixed Frame to base_link in the left panel

- ✅ Check that robot_state_publisher is publishing

- ❌ No robot shown
- * Make sure you moved a slider → joint_state_publisher_gui only publishes joint values when a change occurs

- ❌ No sliders in GUI
- * All joints must be of type revolute and include a <limit> tag

- ✅ Working Status
- * All TF links visible: base_link → link_1 → link_2 → link_3 → link_ee
- * Robot appears in RViz after interaction

## 📂 File Structure
custom_arm/
├── urdf/custom_arm.urdf.xacro         # Xacro robot description
├── launch/display.launch.py           # Launch file with RViz & publishers
├── rviz/view.rviz                     # Saved RViz config
├── package.xml, CMakeLists.txt        # ROS 2 package definitions

🧭 Next Goals
* Add collision/inertial simulation

* Integrate with Ignition Fortress (ros_gz)

* Add ros2_control and move the joints in sim

