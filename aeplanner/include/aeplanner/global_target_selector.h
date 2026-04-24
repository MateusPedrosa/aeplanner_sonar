#ifndef AEPLANNER_GLOBAL_TARGET_SELECTOR_H
#define AEPLANNER_GLOBAL_TARGET_SELECTOR_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <aeplanner/priority_scorer.h>

namespace aeplanner
{

// Returns the highest-priority target that lies outside the local volume
// (distance > local_radius from robot_pos). Returns nullptr if none found.
const ScoredTarget* selectGlobalTarget(
    const std::vector<ScoredTarget>& targets,
    const Eigen::Vector3d&           robot_pos,
    double                           local_radius);

}  // namespace aeplanner

#endif  // AEPLANNER_GLOBAL_TARGET_SELECTOR_H
