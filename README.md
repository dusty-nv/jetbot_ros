# jetbot_ros
ROS nodes for NVIDIA JetBot with Jetson Nano


## System Configuration

It is assumed that the Nano has been setup with JetPack 4.2 and that CUDA, cuDNN, and TensorRT have been installed.

> **Note**:  the process below will likely exceed the disk capacity of the default 16GB filesystem,  
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
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

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
git clone -b onnx https://github.com/dusty-nv/jetson-inference
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

> Note:  as of 2/22/19, only `cmd_str` method is implemented.  Other methods coming soon.


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

This node will listen on the `/jetbot_oled/user_text` topic to recieve string messages that it will display:

```bash
rostopic pub /jetbot_oled/user_text std_msgs/String --once "HELLO!"
```

