#include <ros/ros.h>
#include "frontier_based_exploration/frontier_based_exploration.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_based_exploration_node");
    frontier_based_exploration::FrontierBasedExploration3D fbe3d;
    auto rate = ros::Rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}