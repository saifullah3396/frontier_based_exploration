#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <vector>
#include <Eigen/Dense>
#include <mavros_navigation/utils.h>

using namespace octomap;

namespace mavros_navigation 
{

class Frontier;
class FrontierCluster;

class FrontierCluster : public boost::enable_shared_from_this<FrontierCluster>{
public:
  // Constructor
  FrontierCluster() {}
  ~FrontierCluster() {}

  // Getters
  size_t size() const { return frontiers_.size(); }
  std::vector<boost::shared_ptr<Frontier>> getFrontiers() const { return frontiers_; }
  boost::shared_ptr<Frontier> getCenter() const { return center_f; }
	bool getExplored() const {	return explored_; }

  // Setters
	void setExplored(const bool& explored) { explored_ = explored; }
  
	void addFrontier(const boost::shared_ptr<Frontier>& frontier); // Adds a frontier to cluster
	void join(const boost::shared_ptr<FrontierCluster>& other); // unionize two clusters
	void setup(); // setup center and cluster normal

private:
  boost::shared_ptr<FrontierCluster> get() 
    { return boost::shared_ptr<FrontierCluster>(this); }
  std::vector<boost::shared_ptr<Frontier>> frontiers_; // All the octocells in the cluster
  boost::shared_ptr<Frontier> center_f; // Cluster center
	Eigen::Vector3d normal_;
	bool explored_ = {false}; // Whether this cluster has been navigated to or not
};

}