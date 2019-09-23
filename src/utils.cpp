
#include "mavros_navigation/utils.h"

namespace mavros_navigation 
{
namespace utils 
{
  Eigen::Array<int, Eigen::Dynamic, 3> createNeighborLUT() 
  {
		Eigen::Array<int, Eigen::Dynamic, 3> table;
		table.resize(N_NEIGHBORS, 3);
    table.setZero();
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
    table.row(TSE) = table.row(TS) + table.row(E);
    table.row(BNW) = table.row(BN) + table.row(W);
    table.row(BSW) = table.row(BS) + table.row(W);
    table.row(BNE) = table.row(BN) + table.row(E);
    table.row(BSE) = table.row(BS) + table.row(E); 
    return table;
  }

  template <typename EigenVector3>
  void best_line_from_points(const std::vector<EigenVector3> & c, EigenVector3& origin, EigenVector3& axis)
  {
    // copy coordinates to  matrix in Eigen format
    size_t points = c.size();
    Eigen::Matrix<typename EigenVector3::Scalar, Eigen::Dynamic, Eigen::Dynamic> centers(points, 3);
    for (size_t i = 0; i < points; ++i) 
      centers.row(i) << c[i].x(), c[i].y(), c[i].z();

    origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    axis = eig.eigenvectors().col(2).normalized();
  }
  template void best_line_from_points<Eigen::Vector3d>(const std::vector<Eigen::Vector3d> & c, Eigen::Vector3d& origin, Eigen::Vector3d& axis);

  template <typename EigenVector3>
  void best_line_2d_from_points(const std::vector<EigenVector3> & c, EigenVector3& origin, EigenVector3& axis)
  {
    // copy coordinates to  matrix in Eigen format
    size_t points = c.size();
    Eigen::Matrix<typename EigenVector3::Scalar, Eigen::Dynamic, Eigen::Dynamic> centers(points, 3);
    for (size_t i = 0; i < points; ++i) 
      centers.row(i) << c[i].x(), c[i].y(), 0.0;

    origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    axis = eig.eigenvectors().col(2).normalized();
  }
  template void best_line_2d_from_points<Eigen::Vector3d>(const std::vector<Eigen::Vector3d> & c, Eigen::Vector3d& origin, Eigen::Vector3d& axis);

  Eigen::Vector3d toEigen(const octomap::point3d& p) {
		return Eigen::Vector3d(p.x(), p.y(), p.z());
	}
}
}