#include <mavros_navigation/3DFrontierBasedExploration.h>

FrontierBasedExploration3D::FrontierBasedExploration3D() 
{
  ros::NodeHandle p_nh("~");
  std::string octomap_topic;
  p_nh.getParam("octomap_topic", octomap_topic);
  octomap_sub_ = nh_.subscribe<octomap_msgs::Octomap>(octomap_topic, 10, FrontierBasedExploration3D::octomapCb);
}

FrontierBasedExploration3D::~FrontierBasedExploration3D() 
{
}

void FrontierBasedExploration3D::octomapCb(const octomap_msgs::Octomap::ConstPtr& octomap_msg) {
  // Make an OcTree from the incoming octomap msg
  oc_tree_ = boost::static_pointer_cast<OcTree>(octomap_msgs::fullMsgToMap(*octomap_msg));
}