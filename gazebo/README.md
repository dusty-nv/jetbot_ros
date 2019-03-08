# JetBot Model for Gazebo Robotics Simulator

<img src="https://github.com/dusty-nv/jetbot_ros/raw/master/gazebo/jetbot_gazebo_0.png" width="700">

### Installation

To setup the model, first install and run Gazebo once, and then run the following script:

```bash
$ cd jetbot_ros/gazebo   # substitute where you have jetbot_ros repo on your machine
$ ./install_jetbot_model.sh
```

The `jetbot` model will then be symbolically linked to `~/.gazebo/models` and can then be loaded with Gazebo.

> **note**:  to uninstall the model, run `rm ~/.gazebo/models/jetbot`
