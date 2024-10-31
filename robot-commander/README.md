
# `robot-commander`

`robot-commander` is a companion software for mobile robot that commands the robot based on high-level inputs such as computer vision.

## Environment setup
This software requires **Ubuntu 22.04 LTS**. Python scripts are run using default `python3` provided by Ubuntu 22.04.
```bash
$ python3 --version
Python 3.10.12
```
It is **discouraged** to use virtual environments for python development as ROS libraries are not installable through virtual environment. All dependencies need to be installed globally.

### ROS2 Humble setup
Refer to this link: [Ubuntu (deb packages) — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

> For development environment, choose desktop installation.
> For deployment, choose bare bones installation.

After installing ROS, add following line to `.bashrc`:
```bash
$ source /opt/ros/humble/setup.bash
```

### CUDA and cuDNN setup
TODO

### OpenCV setup
TODO

## Running ROS nodes
Execute python scripts as usual:
```bash
$ python3 ros_node.py
```

To subscribe to a topic via CLI:
```bash
$ ros2 topic echo /cmd_vel
```