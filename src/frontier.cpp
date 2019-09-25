#include <mavros_navigation/frontier.h>

namespace mavros_navigation 
{

Frontier::Frontier(
  const octomap::point3d& coord,
  const std::vector<OcTreeKey>& neighbors, 
  const bool& searched) :
  coord_(coord),
  neighbors_(neighbors),
  searched_(searched)
{
}

}