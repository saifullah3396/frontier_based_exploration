#include "mavros_navigation/frontier_based_exploration.h"
#include "mavros_navigation/octomap_utils.h"

namespace mavros_navigation
{

FrontierBasedExploration3D::FrontierBasedExploration3D() 
{
  ros::NodeHandle p_nh("~");
  double octomap_resolution;
  std::string planning_scene_topic;
  p_nh.getParam("planning_scene_topic", planning_scene_topic);
  p_nh.getParam("octomap_resolution", octomap_resolution);
  oc_tree_ = new OcTree(octomap_resolution);
  oc_tree_->enableChangeDetection(true);
  planning_scene_sub_ = nh_.subscribe<moveit_msgs::PlanningScene>(planning_scene_topic, 10, &FrontierBasedExploration3D::planningSceneCb, this);
  frontiers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_frontiers", 10);
  clusters_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_clusters", 10);
  neighbor_table = octomap_utils::createNeighborLUT();
}

FrontierBasedExploration3D::~FrontierBasedExploration3D() 
{
  if (oc_tree_) {
    delete oc_tree_;
    oc_tree_ = nullptr;
  }
}

void FrontierBasedExploration3D::planningSceneCb(const moveit_msgs::PlanningSceneConstPtr& planning_scene) {
  // Get the OcTree from the incoming moveit planning scene
  const auto& octomap = planning_scene->world.octomap.octomap;
  if (octomap.data.size() != 0) {
    octomap::OcTree* new_oc_tree_ = 
      boost::static_pointer_cast<OcTree>(octomap_msgs::fullMsgToMap(planning_scene->world.octomap.octomap));
    for (auto iter = new_oc_tree_->begin(); iter != oc_tree_->end_leafs(); ++iter) { // find changed nodes between previous and new tree
      auto node = new_oc_tree_->search(iter.getKey());
      oc_tree_->updateNode(iter.getCoordinate(), new_oc_tree_->isNodeOccupied(node));
    }
    findFrontiers();
    findClusters();
  }
}

void FrontierBasedExploration3D::findFrontiers() {
  ros::WallTime start_time = ros::WallTime::now();
  frontiers.clear();
  for (auto iter = oc_tree_->changedKeysBegin(); iter != oc_tree_->changedKeysEnd(); ++iter) {
    const auto& key = iter->first;
    auto node = oc_tree_->search(iter->first);
    auto occupied = oc_tree_->isNodeOccupied(node);
    if (!occupied) {
      bool is_frontier = false;
      bool free_neighbor_exists = false;
      bool occupied_neighbor_exists = false;
      vector<OcTreeKey> neighbor_keys;
      for (int i = 0; i < octomap_utils::N_NEIGHBORS; ++i) {
        auto n_key = OcTreeKey(
          key.k[0] + neighbor_table(i, 0),
          key.k[1] + neighbor_table(i, 1),
          key.k[2] + neighbor_table(i, 2));
        auto n_node = oc_tree_->search(n_key);
        if (!is_frontier) {
          if (!n_node) { // If neighbor is free or unknown
            free_neighbor_exists = true;
          } else {
            // if at least one neighbor is occupied 
            if (oc_tree_->isNodeOccupied(n_node)) {
              occupied_neighbor_exists = true;
            } else {
              free_neighbor_exists = true;
            }
          }
          if (free_neighbor_exists && occupied_neighbor_exists) {
            is_frontier = true;
          }
        } else {
          if (!n_node || !oc_tree_->isNodeOccupied(n_node)) {
            // only free or unoccupied cells can be frontiers so add only those neighbors
            neighbor_keys.push_back(n_key);
          }
        }
      }
      if (is_frontier) {
        frontiers[key] = neighbor_keys;
      }
    }
  }
  ROS_INFO_STREAM("frontiers detected: " << frontiers.size());

  if (publish_frontiers_) {
    visualization_msgs::MarkerArray frontiers_vis;
    int id = 0;
    auto size = oc_tree_->getNodeSize(oc_tree_->getTreeDepth());
    for (const auto& f: frontiers) {
      visualization_msgs::Marker f_m;
      f_m.header.frame_id = "map";
      f_m.header.stamp = ros::Time();
      f_m.ns = "fbe3d";
      f_m.id = id++;
      f_m.type = visualization_msgs::Marker::CUBE;
      f_m.action = visualization_msgs::Marker::MODIFY;
      auto coord = oc_tree_->keyToCoord(f.first);
      f_m.pose.position.x = coord.x();
      f_m.pose.position.y = coord.y();
      f_m.pose.position.z = coord.z();
      f_m.scale.x = size;
      f_m.scale.y = size;
      f_m.scale.z = size;
      f_m.color.a = 0.85;
      f_m.color.r = 1.0;
      f_m.color.g = 1.0;
      f_m.color.b = 0.0;
      frontiers_vis.markers.push_back(f_m);
    }
    frontiers_pub_.publish(frontiers_vis);
  }

  oc_tree_->resetChangeDetection();
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_STREAM("find_frontier used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findClusters()
{
  clusterCenters.clear();
  for (auto& f : frontiers) {
    int c_size = 0;
    auto center = oc_tree_->keyToCoord(f.first);
    ROS_INFO_STREAM("center:" << center);
    auto nn = f.second;
    frontiers.erase(f.first);
    neighborRecursion(f.second, center, c_size); // one cluster is finished
    center /= c_size; // find the cluster center
    clusterCenters.push_back(center);
  }
  ROS_INFO_STREAM("clusters detected: " << clusterCenters.size());

  if (publish_clusters_) {
    visualization_msgs::MarkerArray clusters_vis;
    int id = 0;
    auto size = oc_tree_->getNodeSize(oc_tree_->getTreeDepth());
    for (const auto& c: clusterCenters) {
      visualization_msgs::Marker f_m;
      f_m.header.frame_id = "map";
      f_m.header.stamp = ros::Time();
      f_m.ns = "fbe3d";
      f_m.id = id++;
      f_m.type = visualization_msgs::Marker::CUBE;
      f_m.action = visualization_msgs::Marker::MODIFY;
      auto coord = oc_tree_->keyToCoord(c);
      f_m.pose.position.x = coord.x();
      f_m.pose.position.y = coord.y();
      f_m.pose.position.z = coord.z();
      f_m.scale.x = size;
      f_m.scale.y = size;
      f_m.scale.z = size;
      f_m.color.a = 0.85;
      f_m.color.r = 1.0;
      f_m.color.g = 0.0;
      f_m.color.b = 0.0;
      clusters_vis.markers.push_back(f_m);
    }
    clusters_pub_.publish(clusters_vis);
  }
  while (true);
}

void FrontierBasedExploration3D::neighborRecursion(vector<OcTreeKey>& neighbors, Eigen::Vector3i& center, int& c_size) {
  for (auto& n : neighbors) { // all neighbors of f
    if (frontiers.find(n) == frontiers.end()) { // if neighbor is also a frontier
      center += Eigen::Vector3i(n.k[0], n.k[1], n.k[2]);
      c_size++;
      ROS_INFO_STREAM("center:" << center);
      ROS_INFO_STREAM("c_size:" << c_size);
      auto nn = frontiers[n]; // get second neighbors
      frontiers.erase(n); // remove from frontier
      neighborRecursion(nn, center, c_size); // do it again
    }
  }
}

}