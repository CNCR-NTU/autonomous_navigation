# Summit_XL Autonomous Navigation

This repository hosts all source code for providing autonomous navigation functionalities to the Summit_XL robot.

## Summit_XL ROS Offical Repository

http://wiki.ros.org/Robots/SummitXL

## Contributors

* Josh Paveley <joshpaveley@gmail.com>

* Pedro Machado <pedro.baptistamachado@ntu.ac.uk>


## Step 1: Install ROS Kinetic and Configure Summit_XL

For help installing ROS Kinetic consult the [ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu).

For help configuring your Summit_XL with your computer consult the tutorial on the [CNCR Summit Wiki](https://github.com/CNCR-NTU/summitxl).

## Step 2: Install Object Classifier on Summit_XL
Ensure you have configured your system with the as described in Step 1.

Connect to Wifi of Summit.

SSH into the Summit: `$ ssh summit@summit`

Clone package into catkin_ws:
```
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/CNCR-NTU/object_detection.git

Follow instructions from the README.md file as detailed below.
```
### Step 2.1 Install OpenCV 4
Run the install script:
```
$ cd object_detection/
$ ./install_opencv4-0-1.sh
```
Test the installation.
```
$ python 3
> import cv2
> cv2.__version__
```
**NOTE:** Open a new teminal if it fails.

Installation procedure retrieved from [here](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/)

### 2.2 Install the librealsense 2 
Follow the steps described [here](https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md)

### 2.3 Install the pyrealsense 2
Follow the steps described [here](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python)

### 2.4 Add classifer to Summit launch files
```
$ roscd summit_xl_bringup/launch/
$ nano summit_xl_complete.launch
```
Add the line: 
```
<node pkg="object_detection" type="objectClassification.py" name="objectClassifier" output="screen" />
```
at the end of the file before `</launch>`

Rebuild the catkin_ws: 
```
$ cd ~/catkin_ws
$ catkin_make
```

## Step 3: Ensure all dependencies are installed on your computer
### Summit Dependancies
```
$ sudo apt-get ros-kinetic-summit-xl*
$ sudo apt-get ros-kinetic-robotnik*
```
## Step 4: Clone this repository to your /src/ folder
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/CNCR-NTU/autonomous_navigation
$ cd ~/catkin_ws
$ catkin_make
```
## Step 5: Setup Summit_XL
Ensure both your Summit_XL **and** the robot's internal computer are turned on.

Connect to the Summit's wifi network.

Export ROS master using hostname from /etc/hosts file you configured in setup.

```
$ echo "export ROS_MASTER_URI=http://{SUMMIT-HOSTNAME}:11311" >> ~/.bashrc
```

## Step 6: Run
Ensure your workspace is sourced:
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Launch files
### Complete.Launch
Launches all packages.
```
$ roslaunch autonomous_navigation Complete.launch
```
### Autonomous_Navigation.Launch
Launches packages that enable Autonomous_Navigation functionalities in the robot.
```
$ roslaunch autonomous_navigation Autonomous_Navigation.launch
```
### Monitor_System.Launch
Launches packages that monitor the stage of the system.
```
$ roslaunch autonomous_navigation Monitor_System.launch
```
2018 (c) Computational Neurosciences and Cognitive Robotics Lab - Nottingham Trent University
