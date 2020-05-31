# RoboND-MapMyWorld

![SLAM Database](/images/rtabmap_1.png)

SLAM Database

![Mapping Result](/images/rtabmap_map.png)

Mapping Result

![Transformation Tree](/images/frames_with_rgbd.png)

Transformation Tree

## Overview  
In this project you will create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the RTAB-Map package.

RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Most importantly, the quality of the documentation on [ROS Wiki](http://wiki.ros.org/rtabmap_ros) is very high. Being able to leverage RTAB-Map with your own robots will lead to a solid foundation for mapping and localization well beyond this Nanodegree program.

For this project we will be using the rtabmap_ros package, which is a ROS wrapper (API) for interacting with RTAB-Map. Keep this in mind when looking at the relative documentation.

* You will develop your own package to interface with the rtabmap_ros package.

* You will build upon your localization project to make the necessary changes to interface the robot with RTAB-Map. An example of this is the addition of an RGB-D camera.

* You will ensure that all files are in the appropriate places, all links are properly connected, naming is properly setup and topics are correctly mapped. Furthermore you will need to generate the appropriate launch files to launch the robot and map its surrounding environment.

* When your robot is launched you will teleop around the room to generate a proper map of the environment.

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS packages
```
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-amcl
sudo apt-get install ros-kinetic-rtabmap-ros
```

* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Run the project  
* Clone this repository into /home/workspace/catkin_ws/src
```
cd /home/workspace/catkin_ws/src
git clone https://github.com/fazlurnu/RoboND-MapMyWorld.git
```
* Open the repository and make  
```
cd /home/workspace/catkin_ws/
catkin_make
```
* Launch world.launch in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* Open a new terminal and run teleop node 
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```  
* Launch the mapping launch file
```
roslaunch my_robot mapping.launch
```  
* Testing  
Navigate the robot around by giving command from the teleop terminal. When all areas have been visited, terminate the mapping nodes in its terminal

## Result

To explore the SLAM database that I've created, please click [here](https://drive.google.com/drive/folders/1i2-h3-f1lZ0diFISClwkBGCUIgoYhxid?usp=sharing).
