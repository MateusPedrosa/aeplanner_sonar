#include <aeplanner/frontier_detector.h>
#include <eigen3/Eigen/Eigenvalues>
#include <algorithm>

namespace aeplanner
{

std::vector<FrontierCluster> detectFrontierClusters(
    const std::vector<ClassifiedVoxel>& frontier_voxels,
    float R_cluster)
{
  const size_t N = frontier_voxels.size();
  if (N == 0) return {};

  std::vector<bool> visited(N, false);
  std::vector<FrontierCluster> clusters;

  for (size_t seed = 0; seed < N; ++seed)
  {
    if (visited[seed]) continue;

    // Collect all voxels within R_cluster of this seed
    FrontierCluster cluster;
    cluster.member_indices.push_back(seed);
    visited[seed] = true;

    const Eigen::Vector3d& seed_pos = frontier_voxels[seed].pos;
    for (size_t j = seed + 1; j < N; ++j)
    {
      if (visited[j]) continue;
      if ((frontier_voxels[j].pos - seed_pos).squaredNorm() <= R_cluster * R_cluster)
      {
        cluster.member_indices.push_back(j);
        visited[j] = true;
      }
    }

    const size_t M = cluster.member_indices.size();

    // Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (size_t idx : cluster.member_indices)
      centroid += frontier_voxels[idx].pos;
    centroid /= (double)M;
    cluster.centroid = centroid;
    cluster.density  = (float)M;

    // E_OCC if any member touches an occupied voxel, otherwise E_FREE
    cluster.type = TargetType::E_FREE;
    for (size_t idx : cluster.member_indices)
    {
      if (frontier_voxels[idx].type == TargetType::E_OCC)
      { cluster.type = TargetType::E_OCC; break; }
    }

    if (M < 3)
    {
      // Not enough points for reliable PCA; mark as unreliable
      cluster.normal   = Eigen::Vector3d::UnitZ();
      cluster.w_normal = 0.0f;
      clusters.push_back(cluster);
      continue;
    }

    // Build 3x3 covariance matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (size_t idx : cluster.member_indices)
    {
      Eigen::Vector3d d = frontier_voxels[idx].pos - centroid;
      cov += d * d.transpose();
    }
    cov /= (double)(M - 1);

    // Eigendecomposition (sorted ascending by eigenvalue)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    // eigenvalues() returns them in ascending order: λ0 ≤ λ1 ≤ λ2
    Eigen::Vector3d evals  = solver.eigenvalues();
    Eigen::Matrix3d evecs  = solver.eigenvectors();

    // Normal = eigenvector of smallest eigenvalue (surface normal direction)
    Eigen::Vector3d normal = evecs.col(0);
    normal.normalize();

    // w_normal = (λ1 - λ0) / (λ1 + ε) using the two smallest eigenvalues
    float lam0 = (float)evals[0];
    float lam1 = (float)evals[1];
    float w_normal = (lam1 - lam0) / (lam1 + 1e-6f);

    cluster.normal   = normal;
    cluster.w_normal = std::max(0.0f, std::min(1.0f, w_normal));
    clusters.push_back(cluster);
  }

  return clusters;
}

}  // namespace aeplanner
