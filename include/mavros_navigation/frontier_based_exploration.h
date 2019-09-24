#pragma once

#include <moveit_msgs/PlanningScene.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_server/OctomapServer.h>
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

#include "mavros_navigation/utils.h"

#include <CGAL/min_max_n.h>
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

#include "mavros_navigation/dkm/dkm.hpp"

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

struct FrontierCluster {
  FrontierCluster() {}

	void setup() {
		Eigen::Vector3d fit_line;
		utils::best_line_from_points(frontiers_, center_, fit_line);
		normal_[0] = -fit_line[1];
		normal_[1] = fit_line[0];
		double min_dist = 1000;
		// assign closest frontier to center as center
		Eigen::Vector3d* best_f;
		for (auto& f: frontiers_) {
			auto dist = (f - center_).norm();
			if (dist <= min_dist) {
				min_dist = dist;
				best_f = &f;
			}
		}
		center_ = *best_f;
	}

  vector<Eigen::Vector3d> frontiers_; // All the octocells in the cluster
  Eigen::Vector3d center_; // Cluster center
	Eigen::Vector3d normal_;
	bool searched_ = {false}; // Whether this cluster has been navigated to or not
};

class FrontierBasedExploration3D : public octomap_server::OctomapServer
{
public:
	FrontierBasedExploration3D();	
	~FrontierBasedExploration3D();

private:
	typedef std::tr1::unordered_map<OcTreeKey, bool, OcTreeKey::KeyHash> NodeSearchMap;
	typedef std::tr1::unordered_map<OcTreeKey, vector<OcTreeKey>, OcTreeKey::KeyHash> NodeNeighborMap;

	// publisher functions
	void publishOctree();
	visualization_msgs::MarkerArray toMarkers(
	  const NodeNeighborMap& cells,
	  visualization_msgs::Marker& marker,
	  const std_msgs::ColorRGBA& color);
	template <typename Point>
	visualization_msgs::MarkerArray toMarkers(
	  const std::vector<Point>& cells,
	  visualization_msgs::Marker& marker,
	  const std_msgs::ColorRGBA& color);
	visualization_msgs::MarkerArray toArrowMarkers(
		const std::vector<Eigen::Vector3d>& cells,
		const std::vector<Eigen::Vector3d>& directions,
		visualization_msgs::Marker& marker,
		const std_msgs::ColorRGBA& color);
	template <typename VisType>
	void publishVisCells(
	  const std::string& name, 
	  const VisType& vis_type,
	  const Eigen::Vector3f& color);
	template <typename VisTypeU, typename VisTypeV>
	void publishVisCellsWithDirections(
	  const std::string& name, 
	  const VisTypeU& vis_type,
		const VisTypeV& vis_dir_type,
	  const Eigen::Vector3f& color);
	template <typename VisType>
	void publishVisPoints(
	  const std::string& name, 
	  const VisType& vis_type,
	  const Eigen::Vector3f& color);

	// derived from octomap server
	void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

	// fbe3d functions
	void update();
	void findFrontiers();
	void refreshFrontiers();
	void findVoids();
	void findFrontierClusters();
	void findVoidClusters();
	void neighborRecursion(vector<OcTreeKey>& neighbors);

	// ros
	ros::NodeHandle nh_; // ros node handle
	tf::TransformListener tf_listener; // tf listener

	map<std::string, ros::Publisher> pubs_; // ros publisher
	vector<std::string> publisher_names_ { // names of the publishers 
		"vis_octree",
		"vis_frontiers",
		"vis_voids",
		"vis_f_clusters",
		"vis_v_clusters",
		"vis_hull",
		"vis_rand_sample"
	};
	visualization_msgs::Marker cell_marker_; // base marker for cells
	visualization_msgs::Marker point_marker_; // base marker for points
	visualization_msgs::Marker arrow_marker_; // base marker for arrows
	ros::Subscriber octomap_sub_; // octomap subscriber
	ros::Subscriber planning_scene_sub_; // moveit planning scene subscriber
	// ros params
	std::string frame_id_; // octomap frame id
	std::string base_frame_id_; // sensor frame id
	tf::StampedTransform sensor_tf_; // sensor frame tf
	bool debug_; // whether to debug or not
	double octomap_resolution_; // octomap resolution
	double frontier_search_min_z_; // minimum search height 
	double frontier_search_max_z_; // maximum search height 
	int min_un_neighbor_count_ = {1}; // minimum number of unknown neighbors for frontier
	int min_f_cluster_size_; // minimum number of elements in cluster frontier
	double sensor_max_range_; // maximum sensor range
	double frontier_exp_range_; // range within which a frontier is considered to be explored
	double sensor_horizontal_fov_; // field of view of sensor in horizontal
	double vis_alpha_; // opacity of visualization markers
	double vis_duration_; // duration of visualization markers

	// fbe3d
	octomap::OcTree* oc_tree_; // main octree
	NodeNeighborMap frontiers_; // detected frontiers
	NodeSearchMap frontiers_search_; // boolean map used for setting frontier as searched/unsearched during clustering
	vector<FrontierCluster> f_clusters_; // detected fronter clusters
	vector<vector<octomap::point3d>> v_clusters_; // detected void clusters
	vector<octomap::point3d> voids_; // detected voids
	Eigen::Array<int, Eigen::Dynamic, 3> neighbor_table_; // lookup table for searching close neighbors
	Polyhedron_3 convex_hull_; // convex hull of the octree
	vector<Point_3> hull_points_; // vertices of convex hull
	vector<Point_3> hull_sampled_points_;
};

}