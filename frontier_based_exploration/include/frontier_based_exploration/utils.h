#ifndef OCTOMAP_UTILS_H
#define OCTOMAP_UTILS_H

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <Eigen/Dense>

namespace frontier_based_exploration 
{
namespace utils 
{
  /**
   *  
	 *  (N)orth: positive X   (S)outh:  negative X
	 *  (W)est : positive Y   (E)ast:   negative Y
	 *  (T)op  : positive Z   (B)ottom: negative Z
	 */
	typedef enum {
		W = 0, E, N, S , T , B,                         // face neighbors
		SW, NW, SE, NE, TW, BW, TE, BE, TN, TS, BN, BS, // edge neighbors
		TNW, TSW, TNE, TSE, BNW, BSW, BNE, BSE,          // vertex neighbors
		N_NEIGHBORS // total neighbors
	} NeighborDirection;

  /**
   * Returns an eigen matrix based lookup table for finding direct neighbors 
   * to an octree cell
   * 
   * @return Eigen::Array<int, N_NEIGHBORS, 3>
   */ 
  Eigen::Array<int, Eigen::Dynamic, 3> createNeighborLUT();

	template <typename EigenVector3>
  void best_line_from_points(const std::vector<EigenVector3> & c, EigenVector3& origin, EigenVector3& axis);
	template <typename EigenVector3>
	void best_line_2d_from_points(const std::vector<EigenVector3> & c, EigenVector3& origin, EigenVector3& axis);

	Eigen::Vector3d toEigen(const octomap::point3d& p);
}
}
#endif