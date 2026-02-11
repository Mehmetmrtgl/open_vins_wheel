# Xsens MTi ROS2 Driver

This guide provides a ROS 2 driver for Xsens MTi devices.

## Clone the Repository

Make sure to clone the correct branch (`ros2`):

```bash
mkdir ~/ros2_ws_imu/src
cd ~/ros2_ws_imu/src
git clone --branch ros2 https://github.com/xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git
```

## Installation
### 1. Install Dependencies
Replace [ROSDISTRIBUTION] with your ROS 2 distribution (e.g., humble):

```bash
sudo apt install ros-[ROSDISTRIBUTION]-nmea-msgs
sudo apt install ros-[ROSDISTRIBUTION]-mavros-msgs
```
**For example, if you are using ROS 2 Humble:**

```bash
sudo apt install ros-humble-nmea-msgs
sudo apt install ros-humble-mavros-msgs
```

### 2. Build the Package

```bash
cd ~/ros2_ws_imu
colcon build
```
### 4. Source the Workspace
After building, source the setup script:

```bash
source install/setup.bash
```
**To avoid sourcing manually every time, add it to your ~/.bashrc:**

```bash
sudo nano ~/.bashrc
```
Add this line at the end:

```bash
source /home/[USER_NAME]/ros2_ws/install/setup.bash
```
Replace [USER_NAME] with your actual username. Save and exit.


## How to Use
### Option 1: Launch the driver
```bash
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
```
### Option 2: Launch with RViz for 3D visualization
```bash
ros2 launch xsens_mti_ros2_driver display.launch.py
```