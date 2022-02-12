# deltax_ros_public

## Prerequisites:
- A 64bit Windows OS (x64 architecture is needed for .dll libraries)
- Install [ROS on Windows](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html)

## Remarks
Unlike others ROS-I driver packages, each HIWIN Robot's driver node does not communicate with a ROS Controller Node inside 
the robot's controller. Instead, the control is done by the driver node itself through COM Port.

Although the current version has been structured as described above, it is possible that future versions will support a more 
traditional ROS-Industrial protocol, connecting to a ROS Controller node on the Robot's controller through Simple Message.
