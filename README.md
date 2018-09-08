# Selfie F1/10 Project

[![Build Status](https://travis-ci.com/Goldob/selfie_f1tenth.svg?token=PJsofUnpisMA7tRvqt4p&branch=master)](https://travis-ci.com/Goldob/selfie_f1tenth)
[![codecov](https://codecov.io/gh/Goldob/selfie_f1tenth/branch/master/graph/badge.svg?token=BL6hzoYe9L)](https://codecov.io/gh/Goldob/selfie_f1tenth)

This repository contains a collection of ROS packages specific to implementation of the Selfie Autonomous Car for [F1/10](http://f1tenth.org) October 2018 competition. It is targetting [ROS Kinetic Kame](http://wiki.ros.org/kinetic) distribution.

## Workspace setup

In order to build this project, you need to have a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) setup in a convenient location. It will be further assumed that the workspace is configured at `~/catkin_ws`, but any other place will be fine, provided write permissions. When evaluating the shell commands below, change the location if necessary.

The repository should be placed in the [source space](http://wiki.ros.org/catkin/workspaces#Source_Space) of your workspace. It can be done as follows.

```bash
cd ~/catkin_ws/src
git clone https://github.com/Goldob/selfie_f1tenth
```


### Building packages

```bash
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make
catkin_make install
```
