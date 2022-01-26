# Installation
## 1.Install ROS
Please refer to http://wiki.ros.org/cn/noetic/Installation/Ubuntu
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```
## 2.Install Gazebo
http://gazebosim.org/tutorials?tut=install_ubuntu
```
curl -sSL http://get.gazebosim.org | sh

```
## 3.Creating a workspace for catkin
http://wiki.ros.org/cn/catkin/Tutorials/create_a_workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
## 4.Clone this repo under ~/catkin_ws/src and run command below:
```
cd ~/catkin_ws/
catkin_make
```

# For air hockey pusher
```
cd ~/catkin_ws/src/air_hockey_robot
roslaunch air_hockey_robot gazebo.launch model:=urdf/pusher.urdf.xacro
```
# Urdf_sim_tutorial
See the tutorials over at http://wiki.ros.org/urdf_tutorial<br>
**Replace package name with 'air_hockey_robot' when run command**

