#include <vector>
#include <Eigen/Dense>
#include <mavros_navigation/utils.h>

using namespace octomap;

namespace mavros_navigation 
{

class Frontier;

class FrontierCluster {
public:
  // Constructor
  FrontierCluster() {}

  // Getters
  const size_t size() const { return frontiers_.size(); }
  const std::vector<Frontier*> getFrontiers() const { return frontiers_; }
  const Frontier* getCenter() const { return center_f; }
	const bool& getSearched() const {	return searched_; }

  // Setters
	void setSearched(const bool& searched) { searched_ = searched; }
  
	void addFrontier(Frontier* frontier); // Adds a frontier to cluster
	void join(FrontierCluster* other); // unionize two clusters
	void setup(); // setup center and cluster normal

private:
  std::vector<Frontier*> frontiers_; // All the octocells in the cluster
  Frontier* center_f; // Cluster center
	Eigen::Vector3d normal_;
	bool searched_ = {false}; // Whether this cluster has been navigated to or not
};

}