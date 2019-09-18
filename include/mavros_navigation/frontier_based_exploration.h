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

#include <moveit_msgs/PlanningScene.h>

#include <boost/pointer_cast.hpp>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <Eigen/Dense>

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
	typedef std::tr1::unordered_map<OcTreeKey, vector<OcTreeKey>, OcTreeKey::KeyHash> NodeNeighborMap;
	void planningSceneCb(const moveit_msgs::PlanningSceneConstPtr& planning_scene);
	void findFrontiers();
	void findClusters();
	void neighborRecursion(vector<OcTreeKey>& neighbors, Eigen::Vector3i& center, int& c_size);

	ros::NodeHandle nh_;
	ros::Subscriber planning_scene_sub_;
	octomap::OcTree* oc_tree_;
	NodeNeighborMap frontiers;
	vector<OcTreeKey> clusterCenters;
	Eigen::Array<int, Eigen::Dynamic, 3> neighbor_table;
};

}