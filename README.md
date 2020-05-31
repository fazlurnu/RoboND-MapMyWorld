# RoboND-MapMyWorld

![Localization_gif](/images/ezgif-6-8911f85b5f6d.gif)

Localized Robot

![Transformation Tree](/images/frames.png)

Transformation Tree

## Overview  
In this project you'll utilize ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments. Here are the steps to learn several aspects of robotic software engineering with a focus on ROS:  
* Create a ROS package that launches a custom robot model in a custom Gazebo world  
* Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot  
* Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results  

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
