# ROS Beginner Tutorials
This project mirrors some of the ROS tutorials. It consists of a simple publisher (talker.cpp) and subscriber (listener.cpp) that sends/receives String messages from a single topic.

The talker file also hosts a tf broadcaster that sends a static frame and a ROS service that changes the published String message with input from the user.


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


## Run Unit Tests
To run unit tests (optional), execute:
```
catkin_make run_tests
```
during the build. Alternatively, run:
```
rostest beginner_tutorials serviceTest.test
```
after building with catkin_make.


## Run Demonstration
In one terminal window, run:
```
roslaunch beginner_tutorials talkerListener.launch pubRate:=X record:=Y
```
where X is a positive integer for the publishing rate in Hz and Y is set to 'true' or 'false' to indicate whether rosbag recording should take place. If recording is set to true, all topics will be recorded for 15s and the output file will be saved to the results directory. These settings can be changed in the launch file. The bag file can be inspected using 'rosbag info'. The bag file can be played back using 'rosbag play' and the listener can now be run independently without the talker.

To use the printString service in the talker, open a new terminal window and execute:
```
rosservice call /printString name
```
where name is any desired string.

To inspect the broadcasted tf frame from the talker, use any of 'tf_echo', 'rqt_tf_tree' or 'view_frames' (consult http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf). The frame will be named 'talk' with parent 'world'.
