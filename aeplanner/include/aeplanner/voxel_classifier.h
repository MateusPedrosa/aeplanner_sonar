#ifndef AEPLANNER_VOXEL_CLASSIFIER_H
#define AEPLANNER_VOXEL_CLASSIFIER_H

#include <vector>
#include <unordered_map>
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

// Raw leaf data copied from the map — no search() calls, safe to collect while
// holding shared_lock for a minimal window.
struct LeafEntry {
  Eigen::Vector3f pos;
  la3dm::State    state;
  float           var;
  float           size;
  bool            classified;
};

// Incremental index of voxels near the robot, updated by cloudCallback
// after each commit so that TPM never needs to hold ot_mutex_.
// Only scan-point positions and their 6-connected neighbours are stored,
// which covers all voxels relevant to frontier and U-target detection
// without iterating millions of FREE voxels in the octree.
struct OccupiedUnknownIndex {
  struct VoxelData {
    Eigen::Vector3f pos;
    la3dm::State    state;
    float           var;
    float           size;
    bool            classified;
  };

  std::unordered_map<int64_t, VoxelData> cells;
  mutable std::shared_mutex              mtx;
  float                                  resolution = 0.1f;

  // 21 bits per axis, offset so negative coordinates map cleanly.
  static int64_t packKey(float x, float y, float z, float res)
  {
    constexpr int32_t OFF = 1 << 20;
    uint32_t ux = static_cast<uint32_t>(static_cast<int32_t>(std::floor(x / res)) + OFF) & 0x1FFFFFu;
    uint32_t uy = static_cast<uint32_t>(static_cast<int32_t>(std::floor(y / res)) + OFF) & 0x1FFFFFu;
    uint32_t uz = static_cast<uint32_t>(static_cast<int32_t>(std::floor(z / res)) + OFF) & 0x1FFFFFu;
    return (static_cast<int64_t>(ux) << 42) |
           (static_cast<int64_t>(uy) << 21) |
            static_cast<int64_t>(uz);
  }
};

// Phase 1: copy leaf data from the map (caller must hold shared_lock).
// No map->search() calls inside — O(N_leaves) sequential copy only.
std::vector<LeafEntry> extractLeaves(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max);

// Phase 1 (incremental variant): read leaf data from OccupiedUnknownIndex
// instead of iterating the octree.  No ot_mutex_ needed — reads only
// OccupiedUnknownIndex::mtx (brief shared_lock).
std::vector<LeafEntry> extractLeavesFromIndex(
    const OccupiedUnknownIndex& idx,
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
