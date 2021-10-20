# jetbot_ros
ROS2 nodes and Gazebo model for NVIDIA JetBot with Jetson Nano

> note:  if you want to use ROS Melodic, see the `melodic` branch

### Start the JetBot ROS2 Foxy container

``` bash
git clone -b dev https://github.com/dusty-nv/jetbot_ros
cd jetbot_ros
docker/run.sh
```
 
### Run JetBot

If you have a real JetBot, you can start the camera / motors like so:

``` bash
ros2 launch jetbot_ros jetbot_nvidia.launch.py
```

Otherwise, see the [`Launch Gazebo`](#launch-gazebo) section below to run the simulator.

### Launch Gazebo

``` bash
ros2 launch jetbot_ros gazebo_world.launch.py
```

Then to run the following commands, launch a new terminal session into the container:

``` bash
sudo docker exec -it jetbot_ros /bin/bash
```

### Test Teleop

``` bash
ros2 launch jetbot_ros teleop_keyboard.launch.py
```

The keyboard controls are as follows:

```
w/x:  increase/decrease linear velocity
a/d:  increase/decrease angular velocity

space key, s:  force stop
```

Press Ctrl+C to quit.

### Data Collection

``` bash
ros2 launch jetbot_ros data_collection.launch.py
```

It's recommended to view the camera feed in Gazebo by going to `Window -> Topic Visualization -> gazebo.msgs.ImageStamped` and selecting the `/gazebo/default/jetbot/camera_link/camera/image` topic.

Then drive the robot and press the `C` key to capture an image.  Then annotate that image in the pop-up window by clicking the center point of the path.  Repeat this all the way around the track.  It's important to also collect data of when the robot gets off-course (i.e. near the edges of the track, or completely off the track).  This way, the JetBot will know how to get back on track.

Press Ctrl+C when you're done collecting data.

### Train Navigation Model

Run this from inside the container, substituting the path of the dataset that you collected (by default, it will be in a timestamped folder under `/workspace/src/jetbot_ros/data/datasets/`)

``` bash
cd /workspace/src/jetbot_ros/jetbot_ros/dnn
python3 train.py --data /workspace/src/jetbot_ros/data/datasets/20211018-160950/
```

### Run Navigation Model

After the model has finished training, run the command below to have the JetBot navigate autonomously around the track.  Substitute the path to your model below:

``` bash
ros2 launch jetbot_ros nav_model.launch.py model:=/workspace/src/jetbot_ros/data/models/202106282129/model_best.pth
```

> note:  to reset the position of the robot in the Gazebo environment, press `Ctrl+R`

<a href="https://youtu.be/gok9pvUzZeY" target="_blank"><img src=https://github.com/dusty-nv/jetbot_ros/raw/dev/docs/images/jetbot_gazebo_sim.jpg width="750"></a>

