#include <aeplanner/voxel_classifier.h>

namespace aeplanner
{

// 6-connected neighbour offsets (±1 voxel in each axis)
static const int kNeighbourOffsets[6][3] = {
  { 1, 0, 0}, {-1, 0, 0},
  { 0, 1, 0}, { 0,-1, 0},
  { 0, 0, 1}, { 0, 0,-1}
};

std::vector<ClassifiedVoxel> classifyVoxels(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max,
    float sigma2_thresh)
{
  std::vector<ClassifiedVoxel> result;
  result.reserve(4096);

  la3dm::point3f robot_p3f((float)robot_pos.x(),
                            (float)robot_pos.y(),
                            (float)robot_pos.z());

  for (auto it = map->begin_leaf_in_sphere(robot_p3f, r_max);
       it != map->end_leaf(); ++it)
  {
    la3dm::OcTreeNode& node = it.get_node();
    la3dm::point3f loc      = it.get_loc();
    la3dm::State   s        = node.get_state();
    const float    res      = it.get_size();  // actual leaf size, not fine resolution

    // --- U_TARGET: observed occupied voxel with sufficient variance ---
    if (node.classified && s == la3dm::State::OCCUPIED)
    {
      float var = node.get_var();
      if (var >= sigma2_thresh)
      {
        result.push_back({
          Eigen::Vector3d(loc.x(), loc.y(), loc.z()),
          TargetType::U_TARGET,
          var
        });
      }
      continue;
    }

    // --- E-targets: unknown voxels adjacent to known voxels ---
    if (s != la3dm::State::UNKNOWN) continue;

    bool adj_occ  = false;
    bool adj_free = false;
    for (int i = 0; i < 6; ++i)
    {
      la3dm::OcTreeNode nb = map->search(
          loc.x() + kNeighbourOffsets[i][0] * res,
          loc.y() + kNeighbourOffsets[i][1] * res,
          loc.z() + kNeighbourOffsets[i][2] * res);
      la3dm::State nb_state = nb.get_state();
      if (nb_state == la3dm::State::OCCUPIED)  adj_occ  = true;
      if (nb_state == la3dm::State::FREE)       adj_free = true;
      if (adj_occ && adj_free) break;
    }

    if (!adj_occ && !adj_free) continue;

    TargetType type = adj_occ ? TargetType::E_OCC : TargetType::E_FREE;
    result.push_back({
      Eigen::Vector3d(loc.x(), loc.y(), loc.z()),
      type,
      node.get_var()  // prior variance for unknown voxels
    });
  }

  return result;
}

}  // namespace aeplanner
