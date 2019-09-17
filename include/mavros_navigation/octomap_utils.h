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

  Eigen::Array<int, Eigen::Dynamic, 3> createNeighborLUT() {
		Eigen::Array<int, Eigen::Dynamic, 3> table;
		table.resize(N_NEIGHBORS, 3);
		// Initiailize face neighbors
		table.row(W) <<  0,  1,  0;
		table.row(E) <<  0, -1,  0;
		table.row(N) <<  1,  0,  0;
		table.row(S) << -1,  0,  0;
    table.row(T) <<  0,  0,  1;
    table.row(B) <<  0,  0, -1;

    // Initiailize edge neighbors
    // SW, NW, SE, NE, TW, BW, TE, BE, TN, TS, BN, BS
    table.row(SW) = table.row(S) + table.row(W);
    table.row(NW) = table.row(N) + table.row(W);
    table.row(SE) = table.row(S) + table.row(E);
    table.row(NE) = table.row(N) + table.row(E);
    table.row(TW) = table.row(T) + table.row(W);
    table.row(BW) = table.row(B) + table.row(W);
    table.row(TE) = table.row(T) + table.row(E);
    table.row(BE) = table.row(B) + table.row(E);
    table.row(TN) = table.row(T) + table.row(N);
    table.row(TS) = table.row(T) + table.row(S);
    table.row(BN) = table.row(B) + table.row(N);
    table.row(BS) = table.row(B) + table.row(S);

    // Initiailize vertex neighbors
    // TNW, TSW, TNE, TSE, BNW, BSW, BNE, BSE
    table.row(TNW) = table.row(T) + table.row(NW);
    table.row(TSW) = table.row(T) + table.row(SW);
    table.row(TNE) = table.row(TN) + table.row(E);
    table.row(TSW) = table.row(TS) + table.row(E);
    table.row(BNW) = table.row(BN) + table.row(W);
    table.row(BSW) = table.row(BS) + table.row(W);
    table.row(BNE) = table.row(BN) + table.row(E);
    table.row(BSE) = table.row(BS) + table.row(E);
  }

}
}