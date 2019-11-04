# ROS Beginner Tutorials
This project mirrors the ROS beginner tutorials. Specifically, it consists of a simple publisher and subscriber that sends/receives String messages from a single topic.

## Dependencies
The system must run Ubuntu 16.04 and have C++11 and ROS Kinetic installed.

## Build Instructions
Create a new directory on the local system to designate as the Catkin workspace and create another directory called "src" within it (eg. catkin_ws/src). Clone the project into the src directory:
```
git clone https://github.com/kdglider/beginner_tutorials.git
```
From the catkin_ws directory, build the code and run the setup script:
```
catkin_make
source devel/setup.bash
```

## Run Instructions
Open one more terminal windows. In window 1, run:
```
roslaunch beginner_tutorials talkerListener.launch pubRate:=X
```
where X is a positive integer for the publishing rate in Hertz.
In window 2, run:
```
rosservice call /printString name
```
where name is any desired string.
