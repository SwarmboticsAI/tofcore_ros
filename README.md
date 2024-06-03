# tofcore_ros
Sample nodes for interacting with PreAct ToF sensors in Ros1 and Ros2 

# ToFCore ROS support

Project to build ROS packages for the sensors that work with libtofcore


## Building Ros2

### Prerequisits

- A release of ROS2 installed and active. So far all testing has been done with
  with the Galactic release but things should work with any recent ROS2 release.


### Building

Once the prerequisits are installed and actived (the case of ROS) building should be as easy as: 
```
make ros2
source ./install/setup.bash
```