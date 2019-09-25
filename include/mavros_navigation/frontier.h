#include <vector>
#include <Eigen/Dense>
#include <mavros_navigation/utils.h>

using namespace octomap;

namespace mavros_navigation 
{

struct FrontierCluster;

struct Frontier {
	Frontier(
		const octomap::point3d& coord,
		const std::vector<OcTreeKey>& neighbors, 
		const bool& searched);
	~Frontier() {}

	octomap::point3d coord_;
	std::vector<OcTreeKey> neighbors_;
	bool searched_ = {false};
	FrontierCluster* cluster_;
};

}