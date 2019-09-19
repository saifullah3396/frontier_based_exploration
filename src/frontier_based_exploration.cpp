#include "mavros_navigation/frontier_based_exploration.h"
#include "mavros_navigation/octomap_utils.h"
#include "mavros_navigation/cgal_utils.h"

namespace mavros_navigation
{

bool point_inside(Polyhedron_3& polyhedron, Point_3& query) {
  // Construct AABB tree with a KdTree
  Tree tree(faces(polyhedron).first, faces(polyhedron).second, polyhedron);
  tree.accelerate_distance_queries();
  // Initialize the point-in-polyhedron tester
  Point_inside inside_tester(tree);

  // Determine the side and return true if inside!
  return inside_tester(query) == CGAL::ON_BOUNDED_SIDE;
}

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
  if (publish_frontiers_)
    frontiers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_frontiers", 10);
  if (publish_voids_)
    voids_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_voids", 10);
  if (publish_clusters_)
    clusters_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_clusters", 10);
  if (publish_filtered_oc_tree_)
    filtered_oc_tree_pub_ = nh_.advertise<octomap_msgs::Octomap>("f_oct", 10);
  if (publish_hull_)
    hull_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("oct_hull", 10);
  neighbor_table_ = octomap_utils::createNeighborLUT();
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
      if (iter.getCoordinate().z() > 0.0 && iter.getCoordinate().z() < 2.5)
        oc_tree_->updateNode(iter.getCoordinate(), new_oc_tree_->isNodeOccupied(node));
    }
    publishOctree();
    findFrontiers();
    //findFrontiers();
    //findClusters();
  }
}

void FrontierBasedExploration3D::publishOctree()
{
  if (publish_filtered_oc_tree_) {
    octomap_msgs::Octomap msg;
    msg.header.frame_id = "map";
    octomap_msgs::fullMapToMsg(*oc_tree_, msg);
    filtered_oc_tree_pub_.publish(msg);
  }
}

void FrontierBasedExploration3D::publishFrontiers() {
  if (publish_frontiers_) {
    visualization_msgs::MarkerArray frontiers_vis;
    int id = 0;
    auto size = oc_tree_->getNodeSize(oc_tree_->getTreeDepth());
    for (const auto& f: frontiers_) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time();
      m.ns = "fbe3d";
      m.id = id++;
      m.type = visualization_msgs::Marker::CUBE;
      m.action = visualization_msgs::Marker::MODIFY;
      auto coord = oc_tree_->keyToCoord(f.first);
      m.pose.position.x = coord.x();
      m.pose.position.y = coord.y();
      m.pose.position.z = coord.z();
      m.scale.x = size;
      m.scale.y = size;
      m.scale.z = size;
      m.color.a = 0.85;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.lifetime = ros::Duration(0.1);
      frontiers_vis.markers.push_back(m);
    }
    frontiers_pub_.publish(frontiers_vis);
  }
}

void FrontierBasedExploration3D::publishVoids() {
  if (publish_voids_) {
    visualization_msgs::MarkerArray voids_vis;
    int id = 0;
    auto size = oc_tree_->getNodeSize(oc_tree_->getTreeDepth());
    for (const auto& v: voids_) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time();
      m.ns = "fbe3d";
      m.id = id++;
      m.type = visualization_msgs::Marker::CUBE;
      m.action = visualization_msgs::Marker::MODIFY;
      m.pose.position.x = v.x();
      m.pose.position.y = v.y();
      m.pose.position.z = v.z();
      m.scale.x = size;
      m.scale.y = size;
      m.scale.z = size;
      m.color.a = 0.85;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.lifetime = ros::Duration(0.5);
      voids_vis.markers.push_back(m);
    }
    voids_pub_.publish(voids_vis);
  }
}

void FrontierBasedExploration3D::publishHull() {
  if (publish_hull_) {
    visualization_msgs::MarkerArray hull_vis;
    int id = 0;
    for (const auto& p : hull_points_) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time();
      m.ns = "fbe3d";
      m.id = id++;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::MODIFY;
      m.pose.position.x = p.x();
      m.pose.position.y = p.y();
      m.pose.position.z = p.z();
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.color.a = 0.85;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.lifetime = ros::Duration(0.1);
      hull_vis.markers.push_back(m);
    }

    vector<Point_3> points;
    CGAL::Random_points_in_triangle_mesh_3<Polyhedron_3> g(convex_hull_);
    CGAL::cpp11::copy_n(g, 1000, std::back_inserter(points));
    for (const auto& p : points) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time();
      m.ns = "fbe3d";
      m.id = id++;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::MODIFY;
      m.pose.position.x = p.x();
      m.pose.position.y = p.y();
      m.pose.position.z = p.z();
      m.scale.x = 0.05;
      m.scale.y = 0.05;
      m.scale.z = 0.05;
      m.color.a = 0.85;
      m.color.r = 0.5;
      m.color.g = 1.0;
      m.color.b = 0.5;
      m.lifetime = ros::Duration(0.1);
      hull_vis.markers.push_back(m);
    }
    hull_pub_.publish(hull_vis);
  }
}

void FrontierBasedExploration3D::findFrontiers() {
  ros::WallTime start_time = ros::WallTime::now();
  frontiers_.clear();
  frontiers_search_.clear();
  
  std::vector<Point_3> input_points;
  for (auto iter = oc_tree_->changedKeysBegin(); iter != oc_tree_->changedKeysEnd(); ++iter) {
    const auto& key = iter->first;
    auto coord = oc_tree_->keyToCoord(key);
    input_points.push_back(Point_3(coord.x(), coord.y(), coord.z()));
    auto node = oc_tree_->search(key);
    auto occupied = oc_tree_->isNodeOccupied(node);
    if (!occupied) {
      bool is_frontier = false;
      bool free_neighbor_exists = false;
      bool occupied_neighbor_exists = false;
      vector<OcTreeKey> neighbor_keys;
      for (int i = 0; i < octomap_utils::N_NEIGHBORS; ++i) {
        auto n_key = OcTreeKey(
          key.k[0] + neighbor_table_(i, 0),
          key.k[1] + neighbor_table_(i, 1),
          key.k[2] + neighbor_table_(i, 2));
        auto n_node = oc_tree_->search(n_key);
        if (!is_frontier) {
          if (!n_node) { // If neighbor is free or unknown
            free_neighbor_exists = true;
          } else {
            // if at least one neighbor is occupied 
            //if (oc_tree_->isNodeOccupied(n_node)) {
            //  occupied_neighbor_exists = true;
            //} else {
            //if (!oc_tree_->isNodeOccupied(n_node)) {
              // only unoccupied cells can be frontiers so add only those neighbors
              //neighbor_keys.push_back(n_key);
              //free_neighbor_exists = true;
            //}
          }
          if (free_neighbor_exists) { // && occupied_neighbor_exists) {
            is_frontier = true;
          }
        } else {
          if (n_node && !oc_tree_->isNodeOccupied(n_node)) {
            // only unoccupied cells can be frontiers so add only those neighbors
            neighbor_keys.push_back(n_key);
          }
        }
      }
      if (is_frontier) {
        frontiers_[key] = neighbor_keys;
        frontiers_search_[key] = false;
      }
    }
  }
  hull_points_.insert(hull_points_.begin(), input_points.begin(), input_points.end());
  // Find the convex hull from updated points
  CGAL::convex_hull_3(hull_points_.begin(), hull_points_.end(), convex_hull_);
  hull_points_ = vector<Point_3>(convex_hull_.points_begin(), convex_hull_.points_end());
  vector<Point_3> rand_points;
  CGAL::Random_points_in_triangle_mesh_3<Polyhedron_3> g(convex_hull_);
  CGAL::cpp11::copy_n(g, 1000, std::back_inserter(rand_points));
  voids_.clear();
  for (auto iter = rand_points.begin(); iter != rand_points.end(); ++iter) {
    auto coord = octomap::point3d(iter->x(), iter->y(), iter->z());
    auto n_node = oc_tree_->search(oc_tree_->coordToKey(coord));
    if (!n_node)
      voids_.push_back(coord);
  } 
  oc_tree_->resetChangeDetection();
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_STREAM("findFrontiers used total " << total_time << " sec");
  publishFrontiers();
  publishVoids();
  publishHull();
}

void FrontierBasedExploration3D::findClusters()
{
  cluster_centers_.clear();
  clusters_.clear();
  for (auto& f : frontiers_) {
    if (!frontiers_search_[f.first]) {
      int c_size = 1;
      if (publish_clusters_) {
        clusters_.push_back(vector<OcTreeKey>());
      }
      auto center = oc_tree_->keyToCoord(f.first);
      auto nn = f.second;
      frontiers_search_[f.first] = true;
      neighborRecursion(nn, center, c_size); // one cluster is finished
      center /= c_size; // find the cluster center
      cluster_centers_.push_back(center);
    }
  }
  ROS_INFO_STREAM("clusters detected: " << cluster_centers_.size());

  if (publish_clusters_) {
    visualization_msgs::MarkerArray clusters_vis;
    int id = 0;
    auto size = oc_tree_->getNodeSize(oc_tree_->getTreeDepth());
    for (const auto& c: cluster_centers_) {
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = ros::Time();
      m.ns = "fbe3d";
      m.id = id++;
      m.type = visualization_msgs::Marker::CUBE;
      m.action = visualization_msgs::Marker::MODIFY;
      m.pose.position.x = c.x();
      m.pose.position.y = c.y();
      m.pose.position.z = c.z();
      m.scale.x = size;
      m.scale.y = size;
      m.scale.z = size;
      m.color.a = 0.85;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.lifetime = ros::Duration(0.1);
      clusters_vis.markers.push_back(m);
    }
    for (const auto& cluster: clusters_) {
      std_msgs::ColorRGBA color;
      color.a = 0.5;
      color.r = ((double) rand() / (RAND_MAX));
      color.g = ((double) rand() / (RAND_MAX));
      color.b = ((double) rand() / (RAND_MAX));
      for (const auto& c: cluster) {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time();
        m.ns = "fbe3d";
        m.id = id++;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::MODIFY;
        auto coord = oc_tree_->keyToCoord(c);
        m.pose.position.x = coord.x();
        m.pose.position.y = coord.y();
        m.pose.position.z = coord.z();
        m.scale.x = size;
        m.scale.y = size;
        m.scale.z = size;
        m.color = color;
        m.lifetime = ros::Duration(0.5);
        clusters_vis.markers.push_back(m); 
      }
    }
    clusters_pub_.publish(clusters_vis);
  }
}

void FrontierBasedExploration3D::neighborRecursion(vector<OcTreeKey>& neighbors, octomap::point3d& center, int& c_size) {
  for (auto& n : neighbors) { // all neighbors of f
    if (frontiers_.find(n) != frontiers_.end() && // if neighbor is also a frontier
        !frontiers_search_[n]) // not already searched
    { 
      center += oc_tree_->keyToCoord(n);
      if (publish_clusters_) 
        clusters_.back().push_back(n);
      c_size++;
      auto nn = frontiers_[n]; // get second neighbors
      frontiers_search_[n] = true;
      neighborRecursion(nn, center, c_size); // do it again
    }
  }
}

}