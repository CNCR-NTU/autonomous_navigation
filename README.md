# Summit_XL Autonomous Navigation

This repository hosts all source code for providing autonomous navigation functionalities to the Summit_XL robot.

## Summit_XL ROS Offical Repository

http://wiki.ros.org/Robots/SummitXL

## Contributors

* Josh Paveley <joshpaveley@gmail.com>

* Pedro Machado <pedro.baptistamachado@ntu.ac.uk>


## Step 1: Install ROS Kinetic and Configure Summit_XL

For help installing ROS Kinetic consult the [ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu).

For help configuring your Summit_XL with your computer consult the tutorial on this wiki.

## Step 2: Ensure all dependencies are installed
```
$ sudo apt-get ros-kinetic-summit-xl*
$ sudo apt-get ros-kinetic-robotnik*
```
## Step 3: Clone this repository to your /src/ folder
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/CNCR-NTU/autonomous_navigation
$ cd ~/catkin_ws
$ catkin_make
```
## Step 4: Setup Summit_XL
Ensure both your Summit_XL **and** the robot's internal computer are turned on.

Connect to the Summit's wifi network.

Export ROS master using hostname from /etc/hosts file you configured in setup.
```
$ export ROS_MASTER_URI=http://{SUMMIT-HOSTNAME}:11311
```

## Step 5: Run
Source your workspace
```
$ source ~/catkin_ws/devel/setup.bash
```
Launch file
```
$ roslaunch autonomous_navigation get_laser.launch
```

2018 (c) Computational Neurosciences and Cognitive Robotics Lab - Nottingham Trent University
