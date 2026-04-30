#ifndef AEPLANNER_VOXEL_CLASSIFIER_H
#define AEPLANNER_VOXEL_CLASSIFIER_H

#include <vector>
#include <shared_mutex>
#include <cmath>
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

// Raw leaf data copied from the map under shared_lock.
// No search() calls — safe to collect in a tight lock window.
struct LeafEntry {
  Eigen::Vector3f pos;
  la3dm::State    state;
  float           var;
  float           size;
};

// Phase 1: copy leaf data from the map (caller must hold shared_lock on the map).
// No map->search() calls inside — O(N_leaves_in_sphere) sequential copy only.
std::vector<LeafEntry> extractLeaves(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max);

// Phase 2: classify leaves using a local spatial hash — no map access, no lock.
std::vector<ClassifiedVoxel> classifyExtracted(
    const std::vector<LeafEntry>& leaves,
    float sigma2_thresh);

// Legacy all-in-one (holds caller's lock for both phases; prefer the two-phase
// API from new callers).
std::vector<ClassifiedVoxel> classifyVoxels(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max,
    float sigma2_thresh);

}  // namespace aeplanner

#endif  // AEPLANNER_VOXEL_CLASSIFIER_H
