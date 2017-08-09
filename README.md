# asv_framework

Proposed ROS framework for the operation of a dynamic positioning capable ASV.

Package contains many (now defunct) nodes and scripts, so pay attention.

Use asv_launch as the main launching platform.  Separate launch files can be in other packages, just use the include tag.

For a dynamic positioning simulation (calm water), run the following:

`roslaunch asv_launch sim.launch`

In the visualisation panel, use the "2D Nav Goal" tool to click and drag out a position + orientation for the ASV to align itself to.  Use the "2D Pose Estimate" tool to click and drag out a position + orientation for the ASV to teleport to.

Position Hold Controller:  `rosed asv_control hold_position.py`
Thrust Allocation:         `rosed asv_control thruster_allocator.py`
