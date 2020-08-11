# jetbot_ros
ROS nodes and Gazebo model for NVIDIA JetBot with Jetson Nano


## System Configuration

It is assumed that your Nano's SD card was flashed with NVIDIA's JetPack image - see the [Getting Started](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) guide.

> **Note**:  the process below will likely exceed the disk capacity of a 16GB filesystem,  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; so a larger SD card should be used.  If using the 'Etcher' method with JetPack-L4T image,  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; the APP partition will automatically be resized to fill the SD card upon first booting the system.  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Otherwise flash with L4T using the -S option (example given for 64GB SD card):  
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `sudo ./flash.sh -S 58GiB jetson-nano-sd mmcblk0p1`  


### Install ROS Melodic

```bash
# enable all Ubuntu packages:
$ sudo apt-add-repository universe
$ sudo apt-add-repository multiverse
$ sudo apt-add-repository restricted

# add ROS repository to apt sources
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ROS Base
$ sudo apt-get update
$ sudo apt-get install ros-melodic-ros-base

# add ROS paths to environment
sudo sh -c 'echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc'
```

Close and restart the terminal.


### Install Adafruit Libraries

These Python libraries from Adafruit support the TB6612/PCA9685 motor drivers and the SSD1306 debug OLED:

```bash
# pip should be installed
$ sudo apt-get install python-pip

# install Adafruit libraries
$ pip install Adafruit-MotorHAT
$ pip install Adafruit-SSD1306
```

Grant your user access to the i2c bus:

```bash
$ sudo usermod -aG i2c $USER
```

Reboot the system for the changes to take effect.

### Create catkin workspace

Create a ROS Catkin workspace to contain our ROS packages:

```bash
# create the catkin workspace
$ mkdir -p ~/workspace/catkin_ws/src
$ cd ~/workspace/catkin_ws
$ catkin_make

# add catkin_ws path to bashrc
$ sudo sh -c 'echo "source ~/workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc'

```
> Note:  out of personal preference, my catkin_ws is created as a subdirectory under ~/workspace

Close and open a new terminal window.
Verify that your catkin_ws is visible to ROS:
```bash
$ echo $ROS_PACKAGE_PATH 
/home/nvidia/workspace/catkin_ws/src:/opt/ros/melodic/share
```

### Build jetson-inference

Clone and build the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) repo:

```bash
# git and cmake should be installed
sudo apt-get install git cmake

# clone the repo and submodules
cd ~/workspace
git clone https://github.com/dusty-nv/jetson-inference
cd jetson-inference
git submodule update --init

# build from source
mkdir build
cd build
cmake ../
make

# install libraries
sudo make install
```

### Build ros_deep_learning

Clone and build the [`ros_deep_learning`](https://github.com/dusty-nv/ros_deep_learning) repo:

```bash
# install dependencies
sudo apt-get install ros-melodic-vision-msgs ros-melodic-image-transport ros-melodic-image-publisher

# clone the repo
cd ~/workspace/catkin_ws/src
git clone https://github.com/dusty-nv/ros_deep_learning

# make ros_deep_learning
cd ../    # cd ~/workspace/catkin_ws
catkin_make

# confirm that the package can be found
$ rospack find ros_deep_learning
/home/nvidia/workspace/catkin_ws/src/ros_deep_learning
```

### Build jetbot_ros

Clone and build the [`jetbot_ros`](https://github.com/dusty-nv/jetbot_ros) repo:

```bash
# clone the repo
$ cd ~/workspace/catkin_ws/src
$ git clone https://github.com/dusty-nv/jetbot_ros

# build the package
$ cd ../    # cd ~/workspace/catkin_ws
$ catkin_make

# confirm that jetbot_ros package can be found
$ rospack find jetbot_ros
/home/nvidia/workspace/catkin_ws/src/jetbot_ros
```

## Testing JetBot

Next, let's check that the different components of the robot are working under ROS.

First open a new terminal, and start `roscore`
```bash
$ roscore
```

### Running the Motors

Open a new terminal, and start the `jetbot_motors` node:
```bash
$ rosrun jetbot_ros jetbot_motors.py
```

The `jetbot_motors` node will listen on the following topics:
* `/jetbot_motors/cmd_dir`     relative heading (degree `[-180.0, 180.0]`, speed `[-1.0, 1.0]`)
* `/jetbot_motors/cmd_raw`     raw L/R motor commands  (speed `[-1.0, 1.0]`, speed `[-1.0, 1.0]`)
* `/jetbot_motors/cmd_str`     simple string commands (left/right/forward/backward/stop)

> Note:  currently only `cmd_str` method is implemented.


#### Test Motor Commands
Open a new terminal, and run some test commands:

```bash
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "forward"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "backward"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "left"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "right"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "stop"
```
(it is recommended to initially test with JetBot up on blocks, wheels not touching the ground)  


### Using the Debug OLED

If you have an SSD1306 debug OLED on your JetBot, you can run the `jetbot_oled` node to display system information and user-defined text:

```bash
$ rosrun jetbot_ros jetbot_oled.py
```

By default, `jetbot_oled` will refresh the display every second with the latest memory usage, disk space, and IP addresses.

The node will also listen on the `/jetbot_oled/user_text` topic to recieve string messages from the user that it will display:

```bash
rostopic pub /jetbot_oled/user_text std_msgs/String --once "HELLO!"
```

### Using the Camera

To begin streaming the JetBot camera, start the `jetbot_camera` node:

```bash
$ rosrun jetbot_ros jetbot_camera
```

The video frames will be published to the `/jetbot_camera/raw` topic as [`sensor_msgs::Image`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) messages with BGR8 encoding.  To test the camera feed, install the [`image_view`](http://wiki.ros.org/image_view?distro=melodic) package and then subscribe to `/jetbot_camera/raw` from a new terminal:

```bash
# first open a new terminal
$ sudo apt-get install ros-melodic-image-view
$ rosrun image_view image_view image:=/jetbot_camera/raw
```

A window should then open displaying the live video from the camera.  By default, the window may appear smaller than the video feed.  Click on the terminal or maximize button on the window to enlarge the window to show the entire frame.


## JetBot Model for Gazebo Robotics Simulator

<img src="https://github.com/dusty-nv/jetbot_ros/raw/master/gazebo/jetbot_gazebo_0.png" width="700">

See the [`gazebo`](gazebo) directory of the repo for instructions on loading the JetBot simulator model for Gazebo.

