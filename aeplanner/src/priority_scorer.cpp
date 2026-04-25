#include <aeplanner/priority_scorer.h>
#include <algorithm>

namespace aeplanner
{

std::vector<ScoredTarget> scoreTargets(
    const std::vector<FrontierCluster>& u_clusters,
    const std::vector<ClassifiedVoxel>& u_voxels,
    const std::vector<FrontierCluster>& e_clusters,
    const ScorerParams& params)
{
  std::vector<ScoredTarget> result;
  result.reserve(u_clusters.size() + e_clusters.size());

  // U-target clusters: priority = mean Var_beta of member voxels
  for (const FrontierCluster& uc : u_clusters)
  {
    float sum_var = 0.0f;
    for (size_t idx : uc.member_indices)
      sum_var += u_voxels[idx].var_beta;
    float mean_var = sum_var / (float)uc.member_indices.size();

    result.push_back({
      uc.centroid,
      TargetType::U_TARGET,
      mean_var,
      uc.w_normal >= 0.1f ? uc.normal : Eigen::Vector3d::UnitX(),
      uc.w_normal,
      0
    });
  }

  // E-targets: one ScoredTarget per frontier cluster representative
  for (const FrontierCluster& c : e_clusters)
  {
    // Cluster representative voxel type — we don't know here, use E_OCC as default.
    // The TPM sets the actual type after classification.
    float density_norm = c.density / (params.cluster_norm + 1e-6f);
    density_norm = std::min(1.0f, density_norm);

    // prior Var_beta for unknown voxels is near max (Beta(prior_A, prior_B))
    // We use 0 here as placeholder; TPM fills it from the representative voxel.
    float priority = params.w_frontier * density_norm;

    result.push_back({
      c.centroid,
      c.type,
      priority,
      c.normal,
      c.w_normal,
      0
    });
  }

  // Sort descending by priority
  std::sort(result.begin(), result.end(),
    [](const ScoredTarget& a, const ScoredTarget& b) {
      return a.priority > b.priority;
    });

  return result;
}

}  // namespace aeplanner
