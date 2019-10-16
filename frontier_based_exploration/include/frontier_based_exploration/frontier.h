#include <boost/weak_ptr.hpp>
#include <vector>
#include <Eigen/Dense>
#include <frontier_based_exploration/utils.h>

using namespace octomap;

namespace frontier_based_exploration 
{

class FrontierCluster;

struct Frontier {
	Frontier(
		const octomap::OcTreeKey& key,
		const octomap::point3d& coord,
		const std::vector<OcTreeKey>& neighbors, 
		const bool& searched);
	~Frontier() {}

	octomap::OcTreeKey key_;
	octomap::point3d coord_;
	std::vector<OcTreeKey> neighbors_;
	bool searched_ = {false};
	boost::weak_ptr<FrontierCluster> cluster_;
};

}