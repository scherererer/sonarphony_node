# SonarPhony Node

ROS2 node for the sonarphony project.

Requires `ros_acoustic_msgs`.

## Dependencies

Install ROS2 https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

```
sudo apt install qt6-serialport-dev
sudo apt install ros-jazzy-marine-acoustic-msgs
```

Install native messages

```
git clone https://github.com/scherererer/sonarphony_msgs.git
cd sonarphony_msgs
colcon build
. ./install/setup.bash
```

## Installation

```
git clone ...
cd sonarphony_node
git submodule update --init
colcon build
```

## Running

```
. install/setup.bash
ros2 run sonarphony_node sonarphony_node
```
