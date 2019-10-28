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
Open two more terminal windows. In window 1, run:
```roscore```
In window 2, run:
```rosrun beginner_tutorials talker```
In window 3, run:
```rosrun beginner_tutorials listener```
