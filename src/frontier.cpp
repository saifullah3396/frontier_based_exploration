#include <frontier_based_exploration/frontier.h>

namespace frontier_based_exploration 
{

Frontier::Frontier(
  const octomap::OcTreeKey& key,
  const octomap::point3d& coord,
  const std::vector<OcTreeKey>& neighbors, 
  const bool& searched) :
  key_(key),
  coord_(coord),
  neighbors_(neighbors),
  searched_(searched)
{
  cluster_.reset();
}

}