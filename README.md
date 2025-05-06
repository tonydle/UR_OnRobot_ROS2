# UR_OnRobot_ROS2
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<img src=doc/images/ur_onrobot.gif width=30%>

ROS2 package for Universal Robot e-Series mounted with OnRobot grippers.

## Features
- [ur_onrobot_description](https://github.com/tonydle/UR_OnRobot_ROS2/tree/main/ur_onrobot_description): Combined URDF into a single robot description
- [ur_onrobot_control](https://github.com/tonydle/UR_OnRobot_ROS2/tree/main/ur_onrobot_control): Combined launch file and ROS 2 controllers
- [ur_onrobot_moveit_config](https://github.com/tonydle/UR_OnRobot_ROS2/tree/main/ur_onrobot_moveit_config): MoveIt! configuration package for the combined robot and controllers

## Dependencies (all included in the installation steps below)
- [Universal Robot ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) - humble branch
- [OnRobot ROS2 Driver](https://github.com/tonydle/OnRobot_ROS2_Driver), which requires:
    - [OnRobot ROS2 Description](https://github.com/tonydle/OnRobot_ROS2_Description)
    - libnet1-dev (for Modbus TCP/Serial)
    - [Modbus](https://github.com/Mazurel/Modbus) C++ library (included as a submodule)

## Installation

1. Navigate to your ROS2 workspace and **clone the repository** into the `src` directory:
   ```sh
   git clone https://github.com/tonydle/UR_OnRobot_ROS2.git src/ur_onrobot
   ```
2. Install git dependencies using `vcs`:
   ```sh
   vcs import src --input src/ur_onrobot/required.repos --recursive
   ```
3. Install libnet (for Modbus TCP/Serial):
   ```sh
   sudo apt install libnet1-dev
   ```
4. Let rosdep install ROS 2 dependencies:
   ```sh
   rosdep install -y --from-paths src --ignore-src
   ```
4. Build using colcon with symlink install:
   ```sh
   colcon build --symlink-install
   ```
5. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## Hardware Setup
1. Connect the OnRobot Quick Changer to the Tool I/O of the UR robot.
2. On the UR Teach Pendant, install the [RS485 Daemon URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap) following the steps below:
   - Download the URCap from [Releases](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/releases)
   - Follow the [URCap Installation Guide](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/blob/master/doc/install_urcap.md)
   - Note: Currently there is a bug where if you have the robotiq_grippers URCap installed, the RS485 URCap cannot run.
    Follow the issue [here](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/issues/9) for updates.
   - Restart robot
3. Setup Tool I/O parameters (Installation -> General -> Tool I/O)

   <img src=doc/images/installation_tool_io.png width=60%>

      - Controlled by: User
      - Communication Interface:
         - Baud Rate: 1M
         - Parity: Even
         - Stop Bits: One
         - RX Idle Chars: 1.5
         - TX Idle Chars: 3.5
      - Tool Output Voltage: 24V
      - Standard Output:
         - Digital Output 0: Sinking (NPN)
         - Digital Output 1: Sinking (NPN)

## Usage
### View the URDF
   ```sh
   ros2 launch ur_onrobot_description view_robot.launch.py ur_type:=ur3e onrobot_type:=rg2
   ```

### Start robot
   ```sh
   ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=<robot_ip>
   ```
Other arguments:
- `use_fake_hardware` (default: `false`): Use mock hardware interface for testing
- `launch_rviz` (default: `true`): Launch RViz with the robot model
- `tf_prefix` (default: `""`): Prefix for all TF frames

### Start MoveIt!
   ```sh
   ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
   ```

### Get the full joint states including `finger_width` (metres)
   ```sh
   ros2 topic echo /joint_states
   ```
### Control the gripper with `finger_width_controller`(JointGroupPositionController)
   ```sh
   ros2 topic pub --once /finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
   ```

## Author
[Tony Le](https://github.com/tonydle)

## License
This software is released under the MIT License, see [LICENSE](./LICENSE).