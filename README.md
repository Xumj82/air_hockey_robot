# Installation
## 1.[Install ROS](http://wiki.ros.org/cn/noetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```
## 2.[Install Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)

```
curl -sSL http://get.gazebosim.org | sh

```
## 3.[Creating a workspace for catkin](http://wiki.ros.org/cn/catkin/Tutorials/create_a_workspace)

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
## 4.Clone this repo under ~/catkin_ws/src and run catkin_make:
```
cd ~/catkin_ws/
catkin_make
```

# Air hockey robot
## 1.View robot in RViz
```
roslaunch hockey_robot_description hockey_robot_rviz.launch
```
Robot structure is built in hockey_robot_description/urdf/hockey_robot.xacro.Please refer [Tutorial: Using a URDF in Gazebo](http://gazebosim.org/tutorials/?tut=ros_urdf) to understand how to build robot with xml file. </br>

## 2.View and control robot in Gazebo
```
roslaunch hockey_robot_gazebo air_hockey.launch
```
Open a new terminal
```
rqt
```
Then click <strong>Plugins->Image View</strong>

## 3.Control robot with simlpe python
[Controller_1](/hockey_robot_gazebo/scripts/controller_1.py)
```
rosrun hockey_robot_gazebo controller_1.py
```
[Controller_2](/hockey_robot_gazebo/scripts/controller_2.py)
```
rosrun hockey_robot_gazebo controller_2.py
```

## 4.Some useful commands
List topics:
```
rostopic list
```
Result should like below when run robot in gazebo.</br>
/hockey_robot/joint1_position_controller/command : topic to control track forward/backward</br>
/hockey_robot/joint2_position_controller/command : topic to control track left/right</br>
/hockey_robot/joint3_position_controller/command : topic to control track forward/backward</br>
/hockey_robot/joint4_position_controller/command : topic to control track left/right</br>

```
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/hockey_robot/camera1/camera_info
/hockey_robot/camera1/image_raw
/hockey_robot/camera1/image_raw/compressed
/hockey_robot/camera1/image_raw/compressed/parameter_descriptions
/hockey_robot/camera1/image_raw/compressed/parameter_updates
/hockey_robot/camera1/image_raw/compressedDepth
/hockey_robot/camera1/image_raw/compressedDepth/parameter_descriptions
/hockey_robot/camera1/image_raw/compressedDepth/parameter_updates
/hockey_robot/camera1/image_raw/theora
/hockey_robot/camera1/image_raw/theora/parameter_descriptions
/hockey_robot/camera1/image_raw/theora/parameter_updates
/hockey_robot/camera1/parameter_descriptions
/hockey_robot/camera1/parameter_updates
/hockey_robot/joint1_position_controller/command
/hockey_robot/joint1_position_controller/pid/parameter_descriptions
/hockey_robot/joint1_position_controller/pid/parameter_updates
/hockey_robot/joint1_position_controller/state
/hockey_robot/joint2_position_controller/command
/hockey_robot/joint2_position_controller/pid/parameter_descriptions
/hockey_robot/joint2_position_controller/pid/parameter_updates
/hockey_robot/joint2_position_controller/state
/hockey_robot/joint3_position_controller/command
/hockey_robot/joint3_position_controller/pid/parameter_descriptions
/hockey_robot/joint3_position_controller/pid/parameter_updates
/hockey_robot/joint3_position_controller/state
/hockey_robot/joint4_position_controller/command
/hockey_robot/joint4_position_controller/pid/parameter_descriptions
/hockey_robot/joint4_position_controller/pid/parameter_updates
/hockey_robot/joint4_position_controller/state
/hockey_robot/joint_states
/rosout
/rosout_agg
/tf
/tf_static

```
Get topic message type:
```
rostopic type [topic name]
```
