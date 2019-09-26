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
  std::vector<boost::weak_ptr<Frontier>> getFrontiers() const { return frontiers_; }
  boost::weak_ptr<Frontier> getCenter() const { return center_f; }
	bool getSearched() const {	return searched_; }

  // Setters
	void setSearched(const bool& searched) { searched_ = searched; }
  
	void addFrontier(const boost::weak_ptr<Frontier>& frontier); // Adds a frontier to cluster
	void join(const boost::shared_ptr<FrontierCluster>& other); // unionize two clusters
	void setup(); // setup center and cluster normal

private:
  boost::shared_ptr<FrontierCluster> get() 
    { return boost::shared_ptr<FrontierCluster>(this); }
  std::vector<boost::weak_ptr<Frontier>> frontiers_; // All the octocells in the cluster
  boost::weak_ptr<Frontier> center_f; // Cluster center
	Eigen::Vector3d normal_;
	bool searched_ = {false}; // Whether this cluster has been navigated to or not
};

}