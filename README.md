# beginner_tutorials

## Description

A project implementing the beginner ROS tutorials mentioned on the ROS tutorials page. This includes creating a new package, writing a publisher and subscriber node.

## Dependencies

This package requires ROS Melodic. The instructions to install ROS Melodic is in the link below

http://wiki.ros.org/melodic/Installation/Ubuntu

We use catkin to build packages. Leanr about its installation form http://wiki.ros.org/catkin

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
$ source devel/setup/bash
```

In two terminals run the commands separately
```
$ rosrun beginner_tutorials talker
and 
$ rosrun beginner_tutorials listener
```