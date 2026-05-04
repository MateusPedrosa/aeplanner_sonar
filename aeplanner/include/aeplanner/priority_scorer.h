#ifndef AEPLANNER_PRIORITY_SCORER_H
#define AEPLANNER_PRIORITY_SCORER_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <aeplanner/voxel_classifier.h>
#include <aeplanner/frontier_detector.h>

namespace aeplanner
{

struct ScoredTarget
{
  Eigen::Vector3d pos;
  TargetType      type;
  float           priority;
  Eigen::Vector3d normal;   // v_weak (U-target) or frontier normal (E-target)
  float           w_normal; // 1.0 for U-targets; PCA anisotropy for E-targets
  int             fail_count;
};

struct ScorerParams
{
  float          w_frontier;   // frontier bonus weight (default 0.6)
  float          cluster_norm; // normalisation for cluster density (default 100.0)
  double         lambda_dist;  // exponential distance penalty (default 0.2)
  Eigen::Vector3d robot_pos;   // current robot position
};

// Merges U-target clusters and frontier clusters into a single scored, sorted list.
// u_voxels is needed to look up var_beta for each cluster member via member_indices.
std::vector<ScoredTarget> scoreTargets(
    const std::vector<FrontierCluster>& u_clusters,
    const std::vector<ClassifiedVoxel>& u_voxels,
    const std::vector<FrontierCluster>& e_clusters,
    const ScorerParams& params);

}  // namespace aeplanner

#endif  // AEPLANNER_PRIORITY_SCORER_H
