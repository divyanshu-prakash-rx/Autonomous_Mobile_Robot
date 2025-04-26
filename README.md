# Autonomous Mobile Robots: Simulation and Control Using ROS 2

## Project Overview

This project explores the development and simulation of Autonomous Mobile Robots (AMRs) using ROS 2 Humble and the Nav2 stack. The primary goal is to design a robust simulated environment for AMR navigation and control using Gazebo and ROS 2 Control. By integrating sensors such as LiDAR, IMU, and Optical wheel encoders, we enable precise localization and obstacle avoidance capabilities.

## Features

- Autonomous navigation and obstacle avoidance
- Map creation using SLAM Toolbox
- Localization with AMCL
- Path planning and execution using Nav2 stack
- Robot control with ROS 2 Control framework
- Real-time visualization with RViz
- Custom simulated environments in Gazebo

## Requirements

### Software
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Gazebo 11
- SLAM Toolbox
- Nav2 Stack
- RViz2
- ROS 2 Control

### Hardware (for physical implementation)
- Raspberry Pi 5
- Arduino Mega
- RPLiDAR A1
- IMU Sensor
- 12V DC Motors with Quad Encoders (x2)
- Motor Driver
- 3S LiPo Battery
- 5V Regulator

## Installation

### 1. Install ROS 2 Humble

```bash
# Setup locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

### 2. Install Gazebo and Nav2 Dependencies

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-joy ros-humble-ros2-control ros-humble-ros2-controllers
```

### 3. Create a ROS 2 Workspace

```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
```

### 4. Clone the Required Repositories

```bash
git clone https://github.com/YOUR_USERNAME/amr_robot.git  # Replace with your repository
cd ~/amr_ws
colcon build --symlink-install
```

### 5. Source the Workspace

```bash
echo "source ~/amr_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Setting Up the Robot Simulation

### 1. Create Robot Description Package

If you're implementing this from scratch, create a robot description package with URDF files:

```bash
cd ~/amr_ws/src
ros2 pkg create --build-type ament_cmake amr_description --dependencies rclcpp
```

Then create the necessary URDF files under the `urdf` directory.

### 2. Launch Files Structure

Create launch files for:
- Robot state publisher
- Gazebo simulation
- SLAM & mapping
- Navigation

Example directory structure:
```
amr_description/
├── urdf/
│   ├── amr.urdf.xacro
│   ├── inertial_macros.xacro
│   └── sensors.xacro
├── config/
│   ├── nav2_params.yaml
│   ├── slam_params.yaml
│   └── controllers.yaml
├── meshes/
│   └── visual/
├── launch/
│   ├── rsp.launch.py
│   ├── gazebo.launch.py
│   ├── slam.launch.py
│   └── navigation.launch.py
└── worlds/
    └── hospital.world
```

## Running Simulations

### 1. Launch Gazebo with the Robot Model

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch amr_description gazebo.launch.py
```

This will start Gazebo with the robot model in the hospital environment.
![image](https://github.com/user-attachments/assets/8a44ae11-b156-4bd0-90a7-721f6dd65b40)


![image](https://github.com/user-attachments/assets/c23880ea-71cc-4e26-ad64-ddeedd65dc3d)

### 2. Create a Map with SLAM

```bash
# Terminal 2: Launch SLAM
ros2 launch amr_description slam.launch.py
```

To manually control the robot for mapping:

```bash
# Terminal 3: Teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

After mapping is complete, save the map:

```bash
# Terminal 4: Save the map
ros2 run nav2_map_server map_saver_cli -f ~/amr_ws/src/amr_description/maps/hospital_map
```

### 3. Run Navigation with a Pre-existing Map

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch amr_description gazebo.launch.py

# Terminal 2: Launch Navigation
ros2 launch amr_description navigation.launch.py map:=/path/to/hospital_map.yaml
```

### 4. Set Navigation Goals in RViz

1. In RViz, click the "2D Pose Estimate" button and set the initial pose of the robot
2. Click the "Nav2 Goal" button and set the goal pose for the robot
3. The robot will plan and execute a path to the goal
![image](https://github.com/user-attachments/assets/540ca319-7969-4f39-8214-33cfc189df1c)

## Hardware Setup (For Physical Implementation)

### Raspberry Pi 5 Configuration

1. Flash Raspberry Pi OS (64-bit) using Raspberry Pi Imager
2. Configure SSH and WiFi credentials during imaging
3. Install ROS 2 on Raspberry Pi following the standard installation instructions

### LiDAR A1 Configuration
![image](https://github.com/user-attachments/assets/41826bba-d5be-44ca-aa30-f103397ed22f)

1. Connect RPLiDAR A1 to Raspberry Pi:
   - VCC → 5V
   - GND → GND
   - TX → GPIO RX (UART)

2. Install RPLiDAR ROS 2 package:
```bash
cd ~/amr_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git -b ros2
cd ~/amr_ws
colcon build --symlink-install --packages-select rplidar_ros
```

3. Create udev rule for persistent device naming:
```bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Arduino-ROS 2 Communication

1. Install Arduino IDE and required libraries:
   - rosserial_arduino
   - Encoder library
   - MPU6050 library (for IMU)

2. Program Arduino Mega with the proper firmware for motor control and sensor reading

3. Configure serial communication between Arduino and Raspberry Pi:
```bash
cd ~/amr_ws/src
git clone https://github.com/ros-drivers/rosserial.git -b ros2
cd ~/amr_ws
colcon build --symlink-install --packages-select rosserial
```

## Torque Calculation

For reference, the calculated torque requirement for the motors:
- Robot mass: 1.3489 kg
- Force: 13.23 N
- Torque: 0.6615 Nm = 6.74 kg.cm

Ensure your selected motors can provide sufficient torque with a safety margin.

## Running on Physical Hardware

### 1. Launch the Base Nodes

```bash
# Terminal 1: Launch hardware interface
ros2 launch amr_bringup hardware.launch.py
```

### 2. Launch SLAM for Mapping (First Run)

```bash
# Terminal 2: Launch SLAM
ros2 launch amr_bringup slam.launch.py
```

### 3. Launch Navigation with Existing Map (Regular Operation)

```bash
# Terminal 2: Launch Navigation
ros2 launch amr_bringup navigation.launch.py map:=/path/to/hospital_map.yaml
```

## Troubleshooting

### Common Issues and Solutions

1. **LiDAR not detected**
   - Check USB connections
   - Verify udev rules are properly set
   - Ensure proper permissions: `sudo chmod 666 /dev/ttyUSB0`

2. **Navigation failures**
   - Verify map is correctly loaded
   - Check AMCL parameters
   - Ensure the robot's initial pose is properly set in RViz

3. **Motor control issues**
   - Check Arduino connections
   - Verify PWM settings and motor driver configurations
   - Ensure power supply is adequate

4. **ROS 2 communication issues**
   - Check network configuration
   - Ensure all nodes are in the same ROS domain: `export ROS_DOMAIN_ID=<same_number>`

## References

1. ROS 2 Documentation: https://docs.ros.org/en/humble/
2. Nav2 Documentation: https://navigation.ros.org/
3. SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
4. Gazebo: https://gazebosim.org/docs
5. RPLiDAR ROS 2: https://github.com/Slamtec/rplidar_ros/tree/ros2
