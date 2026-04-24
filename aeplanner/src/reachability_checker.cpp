#include <aeplanner/reachability_checker.h>

namespace aeplanner
{

bool isLikelyReachable(
    const Eigen::Vector3d&                     robot_pos,
    const Eigen::Vector3d&                     target_pos,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    int   N_ray,
    float occ_thresh)
{
  Eigen::Vector3d dir = target_pos - robot_pos;
  int occ_count = 0;

  for (int i = 1; i <= N_ray; ++i)
  {
    Eigen::Vector3d p = robot_pos + (double(i) / double(N_ray)) * dir;
    la3dm::OcTreeNode node = map->search((float)p.x(), (float)p.y(), (float)p.z());
    if (node.get_state() == la3dm::State::OCCUPIED) ++occ_count;
  }

  return ((float)occ_count / (float)N_ray) < occ_thresh;
}

}  // namespace aeplanner
