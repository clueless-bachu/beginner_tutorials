# beginner_tutorials

## Description

A project implementing the beginner ROS tutorials mentioned on the ROS tutorials page. This includes creating a new package, writing a publisher and subscriber node.

## Dependencies

This package requires ROS Melodic. The instructions to install ROS Melodic is in the link below

http://wiki.ros.org/melodic/Installation/Ubuntu

We use catkin to build packages. Learn about its installation form http://wiki.ros.org/catkin

Notice the CMakeLists has roscpp, tf, std_msgs, message_generation dependencies

## Download and Install

First, create a workspace, go to src files and clone this package in the src file
```
$ mkdir -p <workspace_name>/src
$ cd <workspace_name>/src
$ git clone https://github.com/clueless-bachu/beginner_tutorials.git
```

To install, go to the workspace and run
```
$ cd ..
```

## Run ROS nodes

For each new terminal and source the files
```
$ source devel/setup.bash
```

In two terminals run the commands separately
```
$ rosrun beginner_tutorials talker
and 
$ rosrun beginner_tutorials listener
```

## Run ROS launch file

For each new terminal, source the files and run the launch command. Here ROSbag will automatically record and frequency is set to 1. if you don't want to record then set toRecord argument as false
```
$ source devel/setup.bash
$ roslaunch beginner_tutorials node.launch freq:=<rate of publishing> toRecord:=<true or false>
```

## Run ROS service

With the talker node or the node.launch running, in a new terminal
```
$ source devel/setup.bash
$ rosservice call /AddNums "A:<num1> B:<num2>"
```
specify num1 and num2 appropriately

## Inspect the TF frames

First run the talker node in a new terminal
```
cd <workspace_name>
source devel/setup.bash
rosrun beginner_tutorials talker <frequency value>
```
For checking the transfrom open a new terminal and run the following commands
```
cd <workspace_name>
source devel/setup.bash
rosrun tf tf_echo /world /talk
rosrun tf view_frames
evince frames.pdf
rosrun rqt_tf_tree rqt_tf_tree
```

The output frames.pdf is included in the results folder

## Running ROSbag play

After recording, you can verify using the playback.
Open 2 terminals and run these commands for each

Terminal 1
```
cd <workspace_name>/src/beginner_tutorials/results
rosbag play rec.bag
```
Terminal 2
```
rosrun beginner_tutorials listener
```
You should see messages being printed in the by listener node

## Tests

You can run the tests by the runnin the following commands on a new terminal
```
cd <workspace_name>
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
``` 
You should see all the test cases passing.