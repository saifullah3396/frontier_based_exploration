#pragma once

#include <moveit_msgs/PlanningScene.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/pointer_cast.hpp>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <Eigen/Dense>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/algorithm.h>
#include <CGAL/Side_of_triangle_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
typedef K::Point_3                                              Point_3;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron_3> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef CGAL::Side_of_triangle_mesh<Polyhedron_3, K> Point_inside;

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
	typedef std::tr1::unordered_map<OcTreeKey, bool, OcTreeKey::KeyHash> NodeSearchMap;
	typedef std::tr1::unordered_map<OcTreeKey, vector<OcTreeKey>, OcTreeKey::KeyHash> NodeNeighborMap;
	void planningSceneCb(const moveit_msgs::PlanningSceneConstPtr& planning_scene);
	void publishOctree();
	void publishFrontiers();
	void publishVoids();
	void publishHull();
	void findFrontiers();
	void findClusters();
	void neighborRecursion(vector<OcTreeKey>& neighbors, octomap::point3d& center, int& c_size);

	ros::NodeHandle nh_;
	bool publish_frontiers_ = {true};
	ros::Publisher frontiers_pub_;
	bool publish_voids_ = {true};
	ros::Publisher voids_pub_;
	bool publish_clusters_ = {true};
	ros::Publisher filtered_oc_tree_pub_;
	bool publish_filtered_oc_tree_ = {false};
	ros::Publisher clusters_pub_;
	ros::Publisher hull_pub_;
	bool publish_hull_ = {true};

	ros::Subscriber planning_scene_sub_;

	octomap::OcTree* oc_tree_;
	NodeNeighborMap frontiers_;
	NodeSearchMap frontiers_search_;
	vector<octomap::point3d> voids_;
	vector<octomap::point3d> cluster_centers_;
	vector<vector<OcTreeKey>> clusters_;
	Eigen::Array<int, Eigen::Dynamic, 3> neighbor_table_;
	Polyhedron_3 convex_hull_;
	vector<Point_3> hull_points_;
};

}