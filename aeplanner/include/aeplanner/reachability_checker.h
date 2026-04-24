#ifndef AEPLANNER_REACHABILITY_CHECKER_H
#define AEPLANNER_REACHABILITY_CHECKER_H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <bgkloctomap/bgkloctomap.h>

namespace aeplanner
{

// Lightweight reachability check: samples N_ray points along the straight line
// from robot_pos to target_pos and counts occupied voxels. Returns false if
// the occupancy fraction exceeds occ_thresh (default 0.3), indicating a likely
// obstruction that would make RRT planning futile.
bool isLikelyReachable(
    const Eigen::Vector3d&                     robot_pos,
    const Eigen::Vector3d&                     target_pos,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    int    N_ray    = 20,
    float  occ_thresh = 0.3f);

}  // namespace aeplanner

#endif  // AEPLANNER_REACHABILITY_CHECKER_H
