# asv_framework
## Introduction
Proposed ROS framework for the operation of a dynamic positioning capable ASV.

Package contains many (now defunct) nodes and scripts, so pay attention.

Use asv_launch as the main launching platform.  Separate launch files can be in other packages, just use the include tag.

## Specifics
Make sure you have installed the full desktop version if you want to visualise the simulation.

Make sure to install the pid package: `sudo apt-get install ros-indigo-pid`

Make sure to execute `catkin_make` in the workspace directory (to build the packages).

For a dynamic positioning simulation (calm water), run the following:

`roslaunch asv_launch sim.launch`

In the visualisation panel, use the "2D Nav Goal" tool to click and drag out a position + orientation for the ASV to align itself to.  Use the "2D Pose Estimate" tool to click and drag out a position + orientation for the ASV to teleport to.

The following nodes will help you understand the underlying process:
Position Hold Controller:  `rosed asv_control hold_position.py`
Thrust Allocation:         `rosed asv_control thruster_allocator.py`
Simulator:                 `rosed asv_control nav_sim.py`
