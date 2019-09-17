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

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <Eigen/Dense>
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
	/**
	 *  (N)orth: positive X   (S)outh:  negative X
	 *  (W)est : positive Y   (E)ast:   negative Y
	 *  (T)op  : positive Z   (B)ottom: negative Z
	 */
	typedef enum {
		W = 0, E, N, S , T , B,                         // face neighbors
		SW, NW, SE, NE, TW, BW, TE, BE, TN, TS, BN, BS, // edge neighbors
		TNW, TSW, TNE, TSE, BNW, BSW, BNE, BSE,          // vertex neighbors
		N_NEIGHBORS // total neighbors
	} NeighborDirection;
};

}