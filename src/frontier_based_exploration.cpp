#include "mavros_navigation/frontier_based_exploration.h"

namespace mavros_navigation
{

FrontierBasedExploration3D::FrontierBasedExploration3D() 
{
  ros::NodeHandle p_nh("~");

  // initialize parameters
  std::string planning_scene_topic;
  p_nh.getParam("frame_id", frame_id_);
  p_nh.getParam("octomap_resolution", octomap_resolution_);
  p_nh.getParam("planning_scene_topic", planning_scene_topic);
  p_nh.getParam("frontier_search_min_z", frontier_search_min_z_);
  p_nh.getParam("frontier_search_max_z", frontier_search_max_z_);

  // initialize publishers
  for (int i = 0; i < publisher_names_.size(); ++i) { 
    bool publish = false;
    p_nh.getParam(publisher_names_[i], publish);
    if (publish) {
      if (publisher_names_[i] == "vis_octree")
        pubs_[publisher_names_[i]] = nh_.advertise<octomap_msgs::Octomap>(publisher_names_[i], 10);
      else 
        pubs_[publisher_names_[i]] = nh_.advertise<visualization_msgs::MarkerArray>(publisher_names_[i], 10);
    }
  }

  // initialize subscribers
  oc_tree_ = new OcTree(octomap_resolution_);
  oc_tree_->enableChangeDetection(true);
  planning_scene_sub_ = nh_.subscribe<moveit_msgs::PlanningScene>(planning_scene_topic, 10, &FrontierBasedExploration3D::planningSceneCb, this);

  // create neighbor
  neighbor_table_ = octomap_utils::createNeighborLUT();

  // setup visualization marker settings
  cell_marker_.header.frame_id = frame_id_;
  cell_marker_.ns = "fbe3d";
  cell_marker_.type = visualization_msgs::Marker::CUBE;
  cell_marker_.action = visualization_msgs::Marker::MODIFY;
  cell_marker_.color.a = 0.5;
  cell_marker_.lifetime = ros::Duration(0.5);
  cell_marker_.scale.x = octomap_resolution_;
  cell_marker_.scale.y = octomap_resolution_;
  cell_marker_.scale.z = octomap_resolution_;
  point_marker_ = cell_marker_;
  double points_size = 0.1;
  point_marker_.type = visualization_msgs::Marker::SPHERE;
  point_marker_.scale.x = points_size;
  point_marker_.scale.y = points_size;
  point_marker_.scale.z = points_size;
}

FrontierBasedExploration3D::~FrontierBasedExploration3D() 
{
  if (oc_tree_) {
    delete oc_tree_;
    oc_tree_ = nullptr;
  }
}

void FrontierBasedExploration3D::planningSceneCb(const moveit_msgs::PlanningSceneConstPtr& planning_scene) {
  // get the OcTree from the incoming moveit planning scene
  const auto& octomap = planning_scene->world.octomap.octomap;
  if (octomap.data.size() != 0) {
    octomap::OcTree* new_oc_tree_ = 
      boost::static_pointer_cast<OcTree>(octomap_msgs::fullMsgToMap(planning_scene->world.octomap.octomap));

    // update octree from incoming octomap
    for (auto iter = new_oc_tree_->begin(); iter != oc_tree_->end_leafs(); ++iter) { // find changed nodes between previous and new tree
      auto node = new_oc_tree_->search(iter.getKey());
      if (iter.getCoordinate().z() > frontier_search_min_z_ && iter.getCoordinate().z() < frontier_search_max_z_)
        oc_tree_->updateNode(iter.getCoordinate(), new_oc_tree_->isNodeOccupied(node));
    }

    // update exploration
    findFrontiers();
    findVoids();
    findFrontierClusters();
    findVoidClusters();

    // publish visuals 
    publishOctree();
    publishVisCells("vis_frontiers", frontiers_, Eigen::Vector3f(1.0, 0.0, 0.0));
    publishVisCells("vis_voids", voids_, Eigen::Vector3f(1.0, 0.0, 1.0));
    publishVisCells("vis_clusters", f_cluster_centers_, Eigen::Vector3f(0.0, 0.0, 1.0));
    for (const auto& cluster: f_clusters_) {
      Eigen::Vector3f color(
        rand() / (RAND_MAX), rand() / (RAND_MAX), rand() / (RAND_MAX));
      publishVisCells("vis_clusters", cluster, color);
    }
    publishVisPoints("vis_hull", hull_points_, Eigen::Vector3f(0.0, 0.0, 0.0));
    publishVisPoints("vis_rand_sample", hull_sampled_points_, Eigen::Vector3f(0.5, 1.0, 0.5));
  }
}

void FrontierBasedExploration3D::publishOctree()
{
  if (pubs_["vis_octree"]) {
    octomap_msgs::Octomap msg;
    msg.header.frame_id = "map";
    octomap_msgs::fullMapToMsg(*oc_tree_, msg);
    pubs_["vis_octree"].publish(msg);
  }
}

visualization_msgs::MarkerArray FrontierBasedExploration3D::toMarkers(
  const NodeNeighborMap& cells,
  visualization_msgs::Marker& marker,
  const std_msgs::ColorRGBA& color) 
{
  marker.color = color;
  int id = 0;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = id++;  
    auto coord = oc_tree_->keyToCoord(c.first);
    marker.pose.position.x = coord.x();
    marker.pose.position.y = coord.y();
    marker.pose.position.z = coord.z();
    markers.markers.push_back(marker);
  }
  return markers;
}

template <typename Point>
visualization_msgs::MarkerArray FrontierBasedExploration3D::toMarkers(
  const std::vector<Point>& cells,
  visualization_msgs::Marker& marker,
  const std_msgs::ColorRGBA& color) 
{
  marker.color = color;
  int id = 0;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = id++;
    marker.pose.position.x = c.x();
    marker.pose.position.y = c.y();
    marker.pose.position.z = c.z();
    markers.markers.push_back(marker);
  }
  return markers;
}

template <>
visualization_msgs::MarkerArray FrontierBasedExploration3D::toMarkers(
  const std::vector<OcTreeKey>& cells,
  visualization_msgs::Marker& marker,
  const std_msgs::ColorRGBA& color) 
{
  marker.color = color;
  int id = 0;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = id++;
    auto coord = oc_tree_->keyToCoord(c);
    marker.pose.position.x = coord.x();
    marker.pose.position.y = coord.y();
    marker.pose.position.z = coord.z();
    markers.markers.push_back(marker);
  }
  return markers;
}

template <typename VisType>
void FrontierBasedExploration3D::publishVisCells(
  const std::string& name, 
  const VisType& vis_type,
  const Eigen::Vector3f& color) 
{
  if (pubs_[name]) {
    std_msgs::ColorRGBA c;
    c.r = color[0];
    c.g = color[1];clusters_
    c.b = color[2];
    auto markers = toMarkers(vis_type, cell_marker_, c);
    pubs_[name].publish(markers);
  }
}

template <typename VisType>
void FrontierBasedExploration3D::publishVisPoints(
  const std::string& name, 
  const VisType& vis_type,
  const Eigen::Vector3f& color) 
{
  if (pubs_[name]) {
    std_msgs::ColorRGBA c;
    c.r = color[0];
    c.g = color[1];
    c.b = color[2];
    auto markers = toMarkers(vis_type, point_marker_, c);
    pubs_[name].publish(markers);
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
  oc_tree_->resetChangeDetection();
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_STREAM("findFrontiers() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findVoids() {
  ros::WallTime start_time = ros::WallTime::now();
  // Find the convex hull from updated points
  CGAL::convex_hull_3(hull_points_.begin(), hull_points_.end(), convex_hull_);
  hull_points_ = vector<Point_3>(convex_hull_.points_begin(), convex_hull_.points_end());
  vector<Point_3> hull_sampled_points_;
  CGAL::Random_points_in_triangle_mesh_3<Polyhedron_3> g(convex_hull_);
  CGAL::cpp11::copy_n(g, 1000, std::back_inserter(hull_sampled_points_));
  voids_.clear();
  for (auto iter = hull_sampled_points_.begin(); iter != hull_sampled_points_.end(); ++iter) {
    auto coord = octomap::point3d(iter->x(), iter->y(), iter->z());
    auto n_node = oc_tree_->search(oc_tree_->coordToKey(coord));
    if (!n_node)
      voids_.push_back(coord);
  }
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_STREAM("findVoids() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findFrontierClusters()
{
  ros::WallTime start_time = ros::WallTime::now();
  f_cluster_centers_.clear();
  f_clusters_.clear();
  for (auto& f : frontiers_) {
    if (!frontiers_search_[f.first]) {
      int c_size = 1;
      f_clusters_.push_back(vector<OcTreeKey>());
      auto center = oc_tree_->keyToCoord(f.first);
      auto nn = f.second;
      frontiers_search_[f.first] = true;
      neighborRecursion(nn, center, c_size); // one f_cluster is finished
      center /= c_size; // find the f_cluster center
      f_cluster_centers_.push_back(center);
    }
  }
  ROS_INFO_STREAM("f_clusters detected: " << f_cluster_centers_.size());
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_STREAM("findFrontierClusters() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findVoidClusters()
{
  ros::WallTime start_time = ros::WallTime::now();
  v_clusters_.clear();
  std::vector<std::array<double, 3>> kmeans_input;
  for (auto& v : voids_) {
    kmeans_input.push_back(std::array<double, 3>{v.x(), v.y(), v.z()});
  }
  auto v_cluster_data = dkm::kmeans_lloyd(kmeans_input, 3);
  std::cout << "Means:" << std::endl;
  for (const auto& mean : std::get<0>(v_cluster_data)) {
    std::cout << "\t(" << mean[0] << "," << mean[1] << ")" << std::endl;
  }
  double total_time = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_STREAM("findVoidClusters() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::neighborRecursion(vector<OcTreeKey>& neighbors, octomap::point3d& center, int& c_size) {
  for (auto& n : neighbors) { // all neighbors of f
    if (frontiers_.find(n) != frontiers_.end() && // if neighbor is also a frontier
        !frontiers_search_[n]) // not already searched
    { 
      center += oc_tree_->keyToCoord(n);
      f_clusters_.back().push_back(n);
      c_size++;
      auto nn = frontiers_[n]; // get second neighbors
      frontiers_search_[n] = true;
      neighborRecursion(nn, center, c_size); // do it again
    }
  }
}

}