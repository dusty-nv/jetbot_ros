# jetbot_ros
ROS nodes for NVIDIA JetBot with Jetson Nano


## System Configuration

It is assumed that the Nano has been setup with JetPack 4.2 and that CUDA, cuDNN, and TensorRT have been installed.

> **Note**:  the process below will likely exceed the disk capacity of the default 16GB filesystem,
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; so a larger SD card should be used.  If using the 'Etcher' method with
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; pre-build JetPack-L4T image, the APP partition will automatically be resized to fill the SD card.
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Otherwise flash with L4T using the -S option (example given for 64GB SD card)
> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`sudo ./flash.sh -S 58GiB jetson-nano-sd mmcblk0p1`


### Install ROS Melodic

Enable all Ubuntu packages:
```bash
$ sudo apt-add-repository universe
$ sudo apt-add-repository multiverse
$ sudo apt-add-repository restricted
```

Add ROS repository to apt sources:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
```

Install ROS Base:
```bash
$ sudo apt-get update
$ sudo apt-get install ros-melodic-ros-base
```

Add ROS paths to environment:
```bash
sudo sh -c 'echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc'
```

Close and restart the terminal.


### Install Adafruit Libraries

These Python libraries from Adafruit support the TB6612/PCA9685 motor drivers and the SSD1306 OLED debug display:

```bash
$ sudo apt-get install python-pip
$ pip install Adafruit-MotorHAT
$ pip install Adafruit-SSD1306
```

Grant your user access to the i2c bus:

```bash
$ sudo usermod -aG i2c $USER
```

### Create catkin workspace

Create a ROS Catkin workspace to contain our ROS packages:

```bash
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

Clone and build [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) repo:

```bash
cd ~/workspace
sudo apt-get install git cmake
git clone -b onnx https://github.com/dusty-nv/jetson-inference
cd jetson-inference
git submodule update --init
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build ros_deep_learning

Clone and build [`ros_deep_learning`](https://github.com/dusty-nv/ros_deep_learning) repo:

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

### Running the Nodes

Open a new terminal, and start `roscore`
```bash
$ roscore
```

Open a new terminal, and start the `jetbot_motors` node:
```bash
$ rosrun jetbot_ros jetbot_motors.py
```

The `jetbot_motors` node will listen on the following topics:
* `/jetbot_motors/cmd_dir`     relative heading (degree `[-180.0, 180.0]`, speed `[-1.0, 1.0]`)
* `/jetbot_motors/cmd_raw`     raw L/R motor commands  (speed `[-1.0, 1.0]`, speed `[-1.0, 1.0]`)
* `/jetbot_motors/cmd_str`     simple string commands (left/right/forward/backward/stop)

> Note:  as of 2/22/19, only `cmd_str` method is implemented.  Other methods coming soon.


### Testing the Motors

Open a new terminal, and run some test commands:
(it is recommended to initially test with JetBot up on blocks, wheels not touching the ground)

```bash
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "forward"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "backward"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "left"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "right"
$ rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "stop"
```


