#pragma once

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <Eigen/Dense>

namespace mavros_navigation 
{
namespace octomap_utils 
{
    /**
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

}
}