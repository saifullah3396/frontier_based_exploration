#include "mavros_navigation/frontier_based_exploration.h"

namespace mavros_navigation
{

FrontierBasedExploration3D::FrontierBasedExploration3D() 
{
  ros::NodeHandle p_nh("~");
  std::string octomap_topic;
  p_nh.getParam("octomap_topic", octomap_topic);
  octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>(octomap_topic, 10, FrontierBasedExploration3D::octomapCb);
  neighbor_table = octomap_utils::createNeighborLUT();
}

FrontierBasedExploration3D::~FrontierBasedExploration3D() 
{
}

void FrontierBasedExploration3D::octomapCb(const octomap_msgs::Octomap::ConstPtr& octomap_msg) {
  // Make an OcTree from the incoming octomap msg
  oc_tree_ = boost::static_pointer_cast<OcTree>(octomap_msgs::fullMsgToMap(*octomap_msg));
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
        frontiers.insert(
          std::pair<OcTreeKey, vector<OcTreeKey>>(key, neighbor_keys));
      }
    }
  }
  oc_tree_->resetChangeDetection();
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_DEBUG_STREAM("find_frontier used total " << total_time << " sec");
}

}