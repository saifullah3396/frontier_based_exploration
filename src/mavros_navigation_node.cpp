#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavros_navigation_node");
    ros::spin();
    return 0;
}