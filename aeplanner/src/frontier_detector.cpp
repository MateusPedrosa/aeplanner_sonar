#include <aeplanner/frontier_detector.h>
#include <eigen3/Eigen/Eigenvalues>
#include <algorithm>
#include <unordered_map>

namespace aeplanner
{

std::vector<FrontierCluster> detectFrontierClusters(
    const std::vector<ClassifiedVoxel>& frontier_voxels,
    float R_cluster)
{
  const size_t N = frontier_voxels.size();
  if (N == 0) return {};

  // Grid-based O(N) clustering: bucket voxels into cells of side R_cluster.
  // Any voxel within R_cluster of a seed must lie in one of the 27 surrounding
  // cells, so the distance check only runs against those candidates.
  const float inv_R = 1.0f / R_cluster;
  const float R2    = R_cluster * R_cluster;

  // Pack (ix, iy, iz) cell coordinates into a single int64_t.
  // 20 bits per axis with a fixed offset to handle negatives.
  // Supports ±2^19 cells per axis = ±500 000 cells = ±1000 km at R=2 m.
  auto cell_key = [](int ix, int iy, int iz) -> int64_t {
    constexpr int OFF = 1 << 19;
    return (static_cast<int64_t>(ix + OFF) & 0xFFFFF)
        | ((static_cast<int64_t>(iy + OFF) & 0xFFFFF) << 20)
        | ((static_cast<int64_t>(iz + OFF) & 0xFFFFF) << 40);
  };

  // Build grid: cell key → list of voxel indices in that cell.
  std::unordered_map<int64_t, std::vector<size_t>> grid;
  grid.reserve(N);
  for (size_t i = 0; i < N; ++i)
  {
    const Eigen::Vector3f p = frontier_voxels[i].pos.cast<float>();
    int ix = static_cast<int>(std::floor(p.x() * inv_R));
    int iy = static_cast<int>(std::floor(p.y() * inv_R));
    int iz = static_cast<int>(std::floor(p.z() * inv_R));
    grid[cell_key(ix, iy, iz)].push_back(i);
  }

  std::vector<bool> visited(N, false);
  std::vector<FrontierCluster> clusters;

  for (size_t seed = 0; seed < N; ++seed)
  {
    if (visited[seed]) continue;

    FrontierCluster cluster;
    cluster.member_indices.push_back(seed);
    visited[seed] = true;

    const Eigen::Vector3d& seed_pos = frontier_voxels[seed].pos;
    const Eigen::Vector3f  sp       = seed_pos.cast<float>();
    int sx = static_cast<int>(std::floor(sp.x() * inv_R));
    int sy = static_cast<int>(std::floor(sp.y() * inv_R));
    int sz = static_cast<int>(std::floor(sp.z() * inv_R));

    // Check all 27 neighbouring cells (3³ cube around seed's cell).
    for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
    for (int dz = -1; dz <= 1; ++dz)
    {
      auto it = grid.find(cell_key(sx + dx, sy + dy, sz + dz));
      if (it == grid.end()) continue;
      for (size_t j : it->second)
      {
        if (visited[j]) continue;
        if ((frontier_voxels[j].pos - seed_pos).squaredNorm() <= R2)
        {
          cluster.member_indices.push_back(j);
          visited[j] = true;
        }
      }
    }

    const size_t M = cluster.member_indices.size();

    // Centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (size_t idx : cluster.member_indices)
      centroid += frontier_voxels[idx].pos;
    centroid /= static_cast<double>(M);
    cluster.centroid = centroid;
    cluster.density  = static_cast<float>(M);

    // E_OCC if any member touches an occupied voxel, otherwise E_FREE
    cluster.type = TargetType::E_FREE;
    for (size_t idx : cluster.member_indices)
    {
      if (frontier_voxels[idx].type == TargetType::E_OCC)
      { cluster.type = TargetType::E_OCC; break; }
    }

    if (M < 3)
    {
      cluster.normal   = Eigen::Vector3d::UnitZ();
      cluster.w_normal = 0.0f;
      clusters.push_back(cluster);
      continue;
    }

    // PCA: build 3×3 covariance, eigendecompose
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (size_t idx : cluster.member_indices)
    {
      Eigen::Vector3d d = frontier_voxels[idx].pos - centroid;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(M - 1);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d evals = solver.eigenvalues();
    Eigen::Matrix3d evecs = solver.eigenvectors();

    Eigen::Vector3d normal = evecs.col(0);
    normal.normalize();

    float lam0    = static_cast<float>(evals[0]);
    float lam1    = static_cast<float>(evals[1]);
    float w_normal = (lam1 - lam0) / (lam1 + 1e-6f);

    cluster.normal   = normal;
    cluster.w_normal = std::max(0.0f, std::min(1.0f, w_normal));
    clusters.push_back(cluster);
  }

  return clusters;
}

}  // namespace aeplanner
