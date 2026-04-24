#include <aeplanner/global_target_selector.h>

namespace aeplanner
{

const ScoredTarget* selectGlobalTarget(
    const std::vector<ScoredTarget>& targets,
    const Eigen::Vector3d&           robot_pos,
    double                           local_radius)
{
  const double local_r2 = local_radius * local_radius;
  const ScoredTarget* best = nullptr;

  for (const ScoredTarget& t : targets)
  {
    if ((t.pos - robot_pos).squaredNorm() <= local_r2) continue;  // local — skip
    if (!best || t.priority > best->priority)
      best = &t;
  }
  return best;
}

}  // namespace aeplanner
