#include <ros/ros.h>
#include "mavros_navigation/frontier_based_exploration.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_navigation_node");
    mavros_navigation::FrontierBasedExploration3D fbe3d;
    auto rate = ros::Rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}