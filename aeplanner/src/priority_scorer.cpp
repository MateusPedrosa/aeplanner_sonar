#include <aeplanner/priority_scorer.h>
#include <algorithm>

namespace aeplanner
{

std::vector<ScoredTarget> scoreTargets(
    const std::vector<ClassifiedVoxel>& u_targets,
    const std::vector<FrontierCluster>& e_clusters,
    const ScorerParams& params)
{
  std::vector<ScoredTarget> result;
  result.reserve(u_targets.size() + e_clusters.size());

  // U-targets: priority = Var_beta
  for (const ClassifiedVoxel& v : u_targets)
  {
    // v_weak defaults to horizontal plane ⊥ zero; will be filled by caller
    // from info-matrix eigenstruct when available.
    result.push_back({
      v.pos,
      TargetType::U_TARGET,
      v.var_beta,
      Eigen::Vector3d::UnitX(),  // placeholder; overwritten by TPM using eigenstruct
      1.0f,
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
