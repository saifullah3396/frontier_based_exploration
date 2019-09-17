#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <boost/pointer_cast.hpp>
#include <vector>
#include <stdio.h>
#include <fstream>

using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;

namespace mavros_navigation
{

class FrontierBasedExploration3D
{
public:
	FrontierBasedExploration3D();	
	~FrontierBasedExploration3D();

private:
	void octomapCb(const octomap_msgs::Octomap::ConstPtr& octomap_msg);

	ros::NodeHandle nh_;
	ros::Subscriber octomap_sub_;
	octomap::OcTree* oc_tree_;
};

}