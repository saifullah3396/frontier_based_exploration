#include <vector>
#include <Eigen/Dense>
#include <mavros_navigation/utils.h>

using namespace octomap;

namespace mavros_navigation 
{

class Frontier;
class FrontierCluster;

using FrontierPtr = Frontier*;
using FrontierClusterPtr = FrontierCluster*;

class FrontierCluster {
public:
  // Constructor
  FrontierCluster() {}
  ~FrontierCluster() {}

  // Getters
  size_t size() const { return frontiers_.size(); }
  std::vector<FrontierPtr> getFrontiers() const { return frontiers_; }
  FrontierPtr getCenter() const { return center_f; }
	bool getSearched() const {	return searched_; }

  // Setters
	void setSearched(const bool& searched) { searched_ = searched; }
  
	void addFrontier(const FrontierPtr& frontier); // Adds a frontier to cluster
	void join(const FrontierClusterPtr& other); // unionize two clusters
	void setup(); // setup center and cluster normal

private:
  std::vector<FrontierPtr> frontiers_; // All the octocells in the cluster
  FrontierPtr center_f; // Cluster center
	Eigen::Vector3d normal_;
	bool searched_ = {false}; // Whether this cluster has been navigated to or not
};

}