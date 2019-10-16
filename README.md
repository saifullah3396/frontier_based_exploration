# Frontier based Exploration in 3D for UAVs

A frontier based exploration algorithm for navigating through unknown environments and generating a map based on octomaps.

## Installation
- The build is only yet tested on ROS melodic but it should work on other distributions as well. Make a local cakin workspace:
```
mkdir -p ~/catkin_ws/src
```
- Build and source the workspace:
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
- Clone the repository to your workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/saifullah3396/frontier_based_exploration.git
```
- Install dependencies for the package with your ROS distribution:
```
rosdep install --from-paths src --ignore-src --rosdistro <ros-distro> -y.
```
- Build the package:
```
cd ~/catkin_ws
catkin_make
```
## Usage
- Run test launch file for demonstration:
```
roslaunch frontier_based_exploration frontier_based_exploration.launch
```