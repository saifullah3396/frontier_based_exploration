#include <frontier_based_exploration/frontier_based_exploration.h>
#include <frontier_based_exploration/frontier_cluster.h>
#include <frontier_based_exploration/frontier.h>

namespace frontier_based_exploration
{

FrontierBasedExploration3D::FrontierBasedExploration3D() 
{
  ros::NodeHandle p_nh("~");

  // initialize parameters
  std::string planning_scene_topic, sensor_topic;
  p_nh.getParam("frame_id", frame_id_);
  p_nh.getParam("base_frame_id", base_frame_id_);
  p_nh.getParam("debug", debug_);
  p_nh.getParam("resolution", octomap_resolution_);
  p_nh.getParam("planning_scene_topic", planning_scene_topic);
  p_nh.getParam("frontier_search_min_z", frontier_search_min_z_);
  p_nh.getParam("frontier_search_max_z", frontier_search_max_z_);
  p_nh.getParam("min_un_neighbor_count", min_un_neighbor_count_);
  p_nh.getParam("min_f_cluster_size", min_f_cluster_size_);
  p_nh.getParam("sensor_model/max_range", sensor_max_range_);
  p_nh.getParam("frontier_exp_range", frontier_exp_range_);
  p_nh.getParam("sensor_horizontal_fov", sensor_horizontal_fov_);
  p_nh.getParam("vis_alpha", vis_alpha_);
  p_nh.getParam("vis_duration", vis_duration_);

  // initialize publishers
  goal_frontier_pub_ = nh_.advertise<geometry_msgs::PointStamped>("goal_frontier", 10);
  if (debug_) {
    for (int i = 0; i < vis_pub_names_.size(); ++i) { 
      bool publish = false;
      p_nh.getParam(vis_pub_names_[i], publish);
      if (publish) {
        vis_pubs_[vis_pub_names_[i]] = nh_.advertise<visualization_msgs::MarkerArray>(vis_pub_names_[i], 10);
      }
    }
  }

  // initialize subscribers
  oc_tree_ = this->m_octree;
  oc_tree_->enableChangeDetection(true);

  // create neighbor
  neighbor_table_ = utils::createNeighborLUT();

  // setup visualization marker settings
  cell_marker_.header.frame_id = frame_id_;
  cell_marker_.ns = "fbe3d";
  cell_marker_.type = visualization_msgs::Marker::CUBE;
  cell_marker_.action = visualization_msgs::Marker::MODIFY;
  cell_marker_.color.a = vis_alpha_;
  cell_marker_.lifetime = ros::Duration(vis_duration_);
  cell_marker_.scale.x = octomap_resolution_;
  cell_marker_.scale.y = octomap_resolution_;
  cell_marker_.scale.z = octomap_resolution_;
  point_marker_ = cell_marker_;
  double points_size = 0.1;
  point_marker_.type = visualization_msgs::Marker::SPHERE;
  point_marker_.scale.x = points_size;
  point_marker_.scale.y = points_size;
  point_marker_.scale.z = points_size;
  arrow_marker_ = point_marker_;
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  auto arrows_size = 0.1;
  arrow_marker_.scale.x = 0.05;
  arrow_marker_.scale.y = 0.1;
  arrow_marker_.scale.z = 0.05;

  frontiers_.reset();
  frontiers_history_ = boost::make_shared<FrontierMap>();
}

FrontierBasedExploration3D::~FrontierBasedExploration3D() 
{
}

void FrontierBasedExploration3D::insertScan(const tf::Point & sensorOrigin, const PCLPointCloud & ground, const PCLPointCloud & nonground) {
  OctomapServer::insertScan(sensorOrigin, ground, nonground);
  update();
}

visualization_msgs::MarkerArray FrontierBasedExploration3D::toMarkers(
  const FrontierMap& cells,
  visualization_msgs::Marker& marker,
  const std_msgs::ColorRGBA& color) 
{
  marker.color = color;
  marker.color.a = vis_alpha_;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = marker_id_++;  
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
  marker.color.a = vis_alpha_;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = marker_id_++;
    marker.pose.position.x = c.x();
    marker.pose.position.y = c.y();
    marker.pose.position.z = c.z();
    markers.markers.push_back(marker);
  }
  return markers;
}

template <>
visualization_msgs::MarkerArray FrontierBasedExploration3D::toMarkers(
  const std::vector<Eigen::Vector3d>& cells,
  visualization_msgs::Marker& marker,
  const std_msgs::ColorRGBA& color) 
{
  marker.color = color;
  marker.color.a = vis_alpha_;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = marker_id_++;
    marker.pose.position.x = c[0];
    marker.pose.position.y = c[1];
    marker.pose.position.z = c[2];
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
  marker.color.a = vis_alpha_;
  visualization_msgs::MarkerArray markers;
  for (const auto& c: cells) {
    marker.header.stamp = ros::Time();
    marker.id = marker_id_++;
    auto coord = oc_tree_->keyToCoord(c);
    marker.pose.position.x = coord.x();
    marker.pose.position.y = coord.y();
    marker.pose.position.z = coord.z();
    markers.markers.push_back(marker);
  }
  return markers;
}

visualization_msgs::MarkerArray FrontierBasedExploration3D::toArrowMarkers(
  const std::vector<Eigen::Vector3d>& cells,
  const std::vector<Eigen::Vector3d>& directions,
  visualization_msgs::Marker& marker,
  const std_msgs::ColorRGBA& color) 
{
  marker.color = color;
  marker.color.a = vis_alpha_;
  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < cells.size(); ++i) {
    marker.points.clear();
    marker.header.stamp = ros::Time();
    marker.id = marker_id_++;
    geometry_msgs::Point start, end;
    start.x = cells[i][0];
    start.y = cells[i][1];
    start.z = cells[i][2];
    end.x = cells[i][0] + directions[i][0] * 0.5;
    end.y = cells[i][1] + directions[i][1] * 0.5;
    end.z = cells[i][2] + directions[i][2] * 0.5;
    marker.points.push_back(start);
    marker.points.push_back(end);
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
  if (vis_pubs_[name]) {
    std_msgs::ColorRGBA c;
    c.r = color[0];
    c.g = color[1];
    c.b = color[2];
    c.a = vis_alpha_;
    auto markers = toMarkers(vis_type, cell_marker_, c);
    vis_pubs_[name].publish(markers);
  }
}

template <typename VisTypeU, typename VisTypeV>
void FrontierBasedExploration3D::publishVisCellsWithDirections(
  const std::string& name, 
  const VisTypeU& vis_type,
  const VisTypeV& vis_dir_type,
  const Eigen::Vector3f& color) 
{
  if (vis_pubs_[name]) {
    std_msgs::ColorRGBA c;
    c.r = color[0];
    c.g = color[1];
    c.b = color[2];
    c.a = vis_alpha_;
    auto cell_markers = toMarkers(vis_type, cell_marker_, c);
    auto arrow_markers = toArrowMarkers(vis_type, vis_dir_type, arrow_marker_, c);
    vis_pubs_[name].publish(cell_markers);
    vis_pubs_[name].publish(arrow_markers);
  }
}

template <typename VisType>
void FrontierBasedExploration3D::publishVisPoints(
  const std::string& name, 
  const VisType& vis_type,
  const Eigen::Vector3f& color) 
{
  if (vis_pubs_[name]) {
    std_msgs::ColorRGBA c;
    c.r = color[0];
    c.g = color[1];
    c.b = color[2];
    c.a = vis_alpha_;
    auto markers = toMarkers(vis_type, point_marker_, c);
    vis_pubs_[name].publish(markers);
  }
}

void FrontierBasedExploration3D::update() {
  if (base_frame_id_ == "") {
    ROS_ERROR("base_frame_id_ not set! See if sensor information callback is fine.");
    return;
  }

  try {
    // map frame to sensor frame
    tf_listener.lookupTransform(frame_id_, base_frame_id_, ros::Time(0), sensor_tf_);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // update exploration
  //auto start_time = ros::Time::now();
  refreshFrontiers();
  findFrontiers();
  //findVoids();
  findFrontierClusters();
  findGoalFrontier();
  //double total_time = (ros::Time::now() - start_time).toSec();
  //ROS_INFO_STREAM("fbe3d used total " << total_time << " sec");

  //ROS_DEBUG_STREAM("Frontiers found:" << frontiers_->size());
  //ROS_DEBUG_STREAM("Total frontiers in history:" << frontiers_history_->size());
  //ROS_DEBUG_STREAM("Total frontier clusters:" << f_clusters_.size());
  //findVoidClusters();

  // publish visuals 
  if (debug_) {
    marker_id_ = 0;
    publishVisCells("vis_frontiers", *frontiers_, Eigen::Vector3f(1.0, 0.0, 0.0));
    publishVisCells("vis_frontiers", *frontiers_history_, Eigen::Vector3f(0.5, 0.0, 0.0));
    //publishVisCells("vis_voids", voids_, Eigen::Vector3f(1.0, 0.0, 1.0));
    vector<octomap::point3d> f_cluster_centers;
    vector<Eigen::Vector3d> f_cluster_normals;
    for (const auto& cluster: f_clusters_) {
      Eigen::Vector3f color(
        (float)rand() / (float)(RAND_MAX), 
        (float)rand() / (float)(RAND_MAX),
        (float)rand() / (float)(RAND_MAX));
      //publishVisCells("vis_f_clusters", cluster->frontiers_, color);
      f_cluster_centers.push_back(cluster->getCenter()->coord_);
      //f_cluster_normals.push_back(cluster.normal_);
    }
    publishVisCells("vis_f_clusters", f_cluster_centers, Eigen::Vector3f(0.0, 0.0, 1.0));
    //publishVisPoints("vis_hull", hull_points_, Eigen::Vector3f(0.0, 0.0, 0.0));
    //publishVisPoints("vis_rand_sample", hull_sampled_points_, Eigen::Vector3f(0.5, 1.0, 0.5));
  }
}

void FrontierBasedExploration3D::refreshFrontiers()
{
  //auto start_time = ros::Time::now();
  // Mark clusters as searched if they are in robot field of view and sensor range
  // and have neighbors updated
  double s_roll, s_pitch, s_yaw;
  tf::Matrix3x3(sensor_tf_.getRotation()).getRPY(s_roll, s_pitch, s_yaw);
  for (auto& c : f_clusters_) {
    auto center = c->getCenter();
    // Frontier center in map frame
    auto c_in_map = tf::Vector3(center->coord_.x(), center->coord_.y(), center->coord_.z());
    // Frontier center in sensor frame
    auto c_in_sensor = sensor_tf_.inverse() * c_in_map;
    if (fabsf(atan2(c_in_sensor.y(), c_in_sensor.x())) <= sensor_horizontal_fov_) {
      // Frontier center is within field of view
      auto dist = sqrt(
        c_in_sensor.x() * c_in_sensor.x() + 
        c_in_sensor.y() * c_in_sensor.y() +
        c_in_sensor.z() * c_in_sensor.z());
      if (dist <= frontier_exp_range_) {
        bool explored = true;
        // Frontier center is within sensor range
        auto center_key = 
          oc_tree_->coordToKey(center->coord_);
        // Check all neighbors of center to see if a surrounding space is updated
        for (int i = 0; i < utils::N_NEIGHBORS; ++i) {
          auto n_key = OcTreeKey(
            center_key.k[0] + neighbor_table_(i, 0),
            center_key.k[1] + neighbor_table_(i, 1),
            center_key.k[2] + neighbor_table_(i, 2));
          auto n_node = oc_tree_->search(n_key);
          if (!n_node) { // If any neighbor is unknown
            explored = false;
          }
        }
        c->setExplored(explored);
      }
    }
  }

  // Remove clusters if already explored
  auto iter = f_clusters_.begin();
  while (iter != f_clusters_.end()) {
    if ((*iter)->getExplored()) {
      // erase all frontiers from history map
      for (const auto& f: (*iter)->getFrontiers()) {
        frontiers_history_->erase(f->key_);
      }
      // erase the cluster
      iter = f_clusters_.erase(iter);
    } else {
      iter++;
    }
  }

  // cleanup frontiers that are not assigned to any cluster
  for (const auto& f: (*frontiers_history_)) {
    if (f.second->cluster_.expired()) {
      frontiers_history_->erase(f.first);
    }
  }
  //double total_time = (ros::Time::now() - start_time).toSec();
  //ROS_DEBUG_STREAM("refreshFrontiers() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findFrontiers() {
  //ros::WallTime start_time = ros::WallTime::now();
  if (frontiers_)
    frontiers_history_->insert(frontiers_->begin(), frontiers_->end());
  frontiers_ = boost::make_shared<FrontierMap>();
  //std::vector<Point_3> input_points;
  auto sensor_origin = sensor_tf_.getOrigin();
  for (auto iter = oc_tree_->changedKeysBegin(); iter != oc_tree_->changedKeysEnd(); ++iter) {
    const auto& key = iter->first;
    auto coord = oc_tree_->keyToCoord(key);
    //input_points.push_back(Point_3(coord.x(), coord.y(), coord.z()));
    auto node = oc_tree_->search(key);
    if (!node)
      continue;
    auto occupied = oc_tree_->isNodeOccupied(node);
    if (!occupied) {
      // check if node is at max range from sensor and only get those as frontiers
      octomap::point3d diff;
      diff.x() = sensor_origin.x() - coord.x();
      diff.y() = sensor_origin.y() - coord.y();
      diff.z() = sensor_origin.z() - coord.z();
      if (diff.norm() < sensor_max_range_)
        continue;

      bool is_frontier = false;
      vector<OcTreeKey> neighbor_keys;
      int un_count = 0;
      for (int i = 0; i < utils::N_NEIGHBORS; ++i) {
        auto n_key = OcTreeKey(
          key.k[0] + neighbor_table_(i, 0),
          key.k[1] + neighbor_table_(i, 1),
          key.k[2] + neighbor_table_(i, 2));
        auto n_node = oc_tree_->search(n_key);
        if (!is_frontier) {
          if (!n_node) { // If neighbor is unknown
            un_count++;
            if (un_count >= min_un_neighbor_count_)
              is_frontier = true;
          } else {
            // if neighbor is unoccupied
            if (!oc_tree_->isNodeOccupied(n_node)) {
              // only unoccupied cells can be frontiers so add only those neighbors
              neighbor_keys.push_back(n_key);
            }
          }
        } else {
          if (n_node && !oc_tree_->isNodeOccupied(n_node)) {
            // only unoccupied cells can be frontiers so add only those neighbors
            neighbor_keys.push_back(n_key);
          }
        }
      }
      if (is_frontier) {
        (*frontiers_)[key] = boost::make_shared<Frontier>(key, coord, neighbor_keys, false);
      }
    }
  }
  oc_tree_->resetChangeDetection();
  //hull_points_.insert(hull_points_.begin(), input_points.begin(), input_points.end());
  //double total_time = (ros::WallTime::now() - start_time).toSec();
  //ROS_DEBUG_STREAM("findFrontiers() used total " << total_time << " sec");
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
  //ROS_DEBUG_STREAM("findVoids() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findFrontierClusters()
{
  //ros::WallTime start_time = ros::WallTime::now();
  for (auto& f : *frontiers_) {
    auto frontier = (*frontiers_)[f.first];
    if (!frontier->searched_) {
      auto cluster = boost::make_shared<FrontierCluster>(); // make a new cluster
      cluster->addFrontier(frontier); // add the first frontier
      neighborRecursion(frontier->neighbors_, cluster); // one f_cluster is finished
      if (cluster->size() < min_f_cluster_size_) {
        cluster.reset();
        continue;
      }
      cluster->setup();
      f_clusters_.push_back(cluster);
    }
  }
  //double total_time = (ros::WallTime::now() - start_time).toSec();
  //ROS_DEBUG_STREAM("findFrontierClusters() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::neighborRecursion(
  vector<OcTreeKey>& neighbors,
  const FrontierClusterSharedPtr& cluster) 
{
  for (auto& n : neighbors) { // all neighbors of f
    if (frontiers_->count(n) && 
        !(*frontiers_)[n]->searched_) // if neighbor is also a frontier and not already searched
    {
      auto frontier = (*frontiers_)[n]; // get frontier
      cluster->addFrontier(frontier);
      neighborRecursion(frontier->neighbors_, cluster); // repeat
    } else if (frontiers_history_ != nullptr) {
      if (frontiers_history_->count(n)) { // if neighbor was a frontier in last run
        auto frontier = (*frontiers_history_)[n];
        if (!frontier->cluster_.expired() && frontier->cluster_.lock() != cluster) { // cluster still exists
          auto prev_cluster = frontier->cluster_.lock();
          cluster->join(prev_cluster);
          f_clusters_.erase(std::remove(f_clusters_.begin(), f_clusters_.end(), prev_cluster), f_clusters_.end());
        }
      }
    }
  }
}

void FrontierBasedExploration3D::findVoidClusters()
{
  //ros::WallTime start_time = ros::WallTime::now();
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
  //double total_time = (ros::WallTime::now() - start_time).toSec();
  //ROS_DEBUG_STREAM("findVoidClusters() used total " << total_time << " sec");
}

void FrontierBasedExploration3D::findGoalFrontier()
{
  // find all frontier cluster centers within robot view and closest of them
  double min_dist = 1000;
  FrontierClusterSharedPtr closest_c;
  closest_c.reset();
  double s_roll, s_pitch, s_yaw;
  tf::Matrix3x3(sensor_tf_.getRotation()).getRPY(s_roll, s_pitch, s_yaw);
  for (auto& c : f_clusters_) { // clusters may have changed from refresh time so find center distances again
    auto center = c->getCenter();
    // Frontier center in map frame
    auto c_in_map = tf::Vector3(center->coord_.x(), center->coord_.y(), center->coord_.z());
    // Frontier center in sensor frame
    auto c_in_sensor = sensor_tf_.inverse() * c_in_map;
    if (fabsf(atan2(c_in_sensor.y(), c_in_sensor.x())) <= sensor_horizontal_fov_) {
      // Frontier center is within field of view
      auto dist = sqrt(
        c_in_sensor.x() * c_in_sensor.x() + 
        c_in_sensor.y() * c_in_sensor.y() +
        c_in_sensor.z() * c_in_sensor.z());
      if (dist <= min_dist) {
        min_dist = dist;
        closest_c = c;
      }
    }
  }
  // publish closest frontier to robot in the vicinity as the goal frontier
  if (closest_c) {
    geometry_msgs::PointStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.point.x = closest_c->getCenter()->coord_.x();
    goal.point.y = closest_c->getCenter()->coord_.y();
    goal.point.z = closest_c->getCenter()->coord_.z();
    goal_frontier_pub_.publish(goal);
  }
}

}