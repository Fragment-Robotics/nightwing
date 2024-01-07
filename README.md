# Nightwing
Repository for an industrial-focused autonomous drone companion's code.

## Setup

We leverage [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/index.html) with its packages and docker images for development and on-bot
container runtime. Steps for setting up the development environment:

1. Follow [NVIDIA Isaac ROS Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)
2. Follow camera sensor specific Isaac ROS setup
    - [Realsense](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/realsense_setup.html)
    - [ZED 2](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/zed_setup.html)
    - [Hawk](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/hawk_setup.html)
3. Install other host dependencies

```bash
$ sudo apt install ros-humble-foxglove-bridge
```

4. Install XRCE-DDS Agent for PX4-Jetson ROS2 Communication. We will build it as part of our ROS2 workspace.

```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git 
```
5. Install `Nightwing` repo

```bash
$ git clone https://github.com/Fragment-Robotics/nightwing.git
```

6. Drop into development docker containers and build.

```bash
$ cd src/isaac_ros_common
$ ./scripts/run_dev.sh ${ISAAC_ROS_WS}
$ colcon build --cmake-args '-DBUILD_TOOLS=ON'
```

7. Run an example launch to verify installation

```bash
$ cd ~/ros2_ws/ && source install/local_setup.bash
$ ros2 launch nightwing_example_package example_launch.py
```