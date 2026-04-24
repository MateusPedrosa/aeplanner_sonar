#ifndef AEPLANNER_VOXEL_CLASSIFIER_H
#define AEPLANNER_VOXEL_CLASSIFIER_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <bgkloctomap/bgkloctomap.h>

namespace aeplanner
{

enum class TargetType : uint8_t
{
  U_TARGET = 0,  // occupied + high Beta variance
  E_OCC    = 1,  // unknown voxel adjacent to occupied (occ/unk frontier)
  E_FREE   = 2   // unknown voxel adjacent to free (free/unk frontier)
};

struct ClassifiedVoxel
{
  Eigen::Vector3d pos;
  TargetType      type;
  float           var_beta;
};

// Classifies all voxels within r_max of robot_pos into U_TARGET and frontier
// (E_OCC / E_FREE) categories. sigma2_thresh is the minimum Var_beta for a
// U_TARGET; voxels below it are skipped.
std::vector<ClassifiedVoxel> classifyVoxels(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max,
    float sigma2_thresh);

}  // namespace aeplanner

#endif  // AEPLANNER_VOXEL_CLASSIFIER_H
