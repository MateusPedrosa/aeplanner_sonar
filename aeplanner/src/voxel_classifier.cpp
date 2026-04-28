#include <aeplanner/voxel_classifier.h>
#include <unordered_map>
#include <cmath>

namespace aeplanner
{

// 6-connected neighbour offsets (±1 voxel in each axis)
static const int kNeighbourOffsets[6][3] = {
  { 1, 0, 0}, {-1, 0, 0},
  { 0, 1, 0}, { 0,-1, 0},
  { 0, 0, 1}, { 0, 0,-1}
};

// Pack quantized leaf coordinate into a single int64 key.
// Uses the same 21-bit-per-axis scheme as OccupiedSnapshot.
static int64_t packLeafKey(float x, float y, float z, float res)
{
  constexpr int32_t OFF = 1 << 20;
  uint32_t ux = static_cast<uint32_t>(static_cast<int32_t>(std::floor(x / res)) + OFF) & 0x1FFFFFu;
  uint32_t uy = static_cast<uint32_t>(static_cast<int32_t>(std::floor(y / res)) + OFF) & 0x1FFFFFu;
  uint32_t uz = static_cast<uint32_t>(static_cast<int32_t>(std::floor(z / res)) + OFF) & 0x1FFFFFu;
  return (static_cast<int64_t>(ux) << 42) |
         (static_cast<int64_t>(uy) << 21) |
          static_cast<int64_t>(uz);
}

std::vector<LeafEntry> extractLeaves(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max)
{
  // Caller must hold shared_lock on the map for this call.
  // We do NOT call map->search() here — only read each leaf's own data.
  std::vector<LeafEntry> leaves;
  leaves.reserve(8192);

  la3dm::point3f robot_p3f((float)robot_pos.x(),
                            (float)robot_pos.y(),
                            (float)robot_pos.z());

  for (auto it = map->begin_leaf_in_sphere(robot_p3f, r_max);
       it != map->end_leaf(); ++it)
  {
    const la3dm::OcTreeNode& node = it.get_node();
    la3dm::point3f loc = it.get_loc();
    LeafEntry e;
    e.pos        = Eigen::Vector3f(loc.x(), loc.y(), loc.z());
    e.state      = node.get_state();
    e.var        = node.get_var();
    e.size       = it.get_size();
    e.classified = node.classified;
    leaves.push_back(e);
  }

  return leaves;
}

std::vector<ClassifiedVoxel> classifyExtracted(
    const std::vector<LeafEntry>& leaves,
    float sigma2_thresh)
{
  if (leaves.empty()) return {};

  // Use the size of the first leaf as the common resolution for key packing.
  const float res = leaves[0].size;

  // Build a spatial hash: packed coordinate → leaf state.
  // This replaces all map->search() calls that were previously under shared_lock.
  std::unordered_map<int64_t, la3dm::State> state_index;
  state_index.reserve(leaves.size() * 2);
  for (const LeafEntry& e : leaves)
    state_index.emplace(packLeafKey(e.pos.x(), e.pos.y(), e.pos.z(), e.size),
                        e.state);

  // Returns the state of a neighbour, defaulting to UNKNOWN if not in index.
  auto neighborState = [&](float x, float y, float z) -> la3dm::State {
    auto it = state_index.find(packLeafKey(x, y, z, res));
    return (it != state_index.end()) ? it->second : la3dm::State::UNKNOWN;
  };

  std::vector<ClassifiedVoxel> result;
  result.reserve(1024);

  for (const LeafEntry& e : leaves)
  {
    const float x = e.pos.x(), y = e.pos.y(), z = e.pos.z();

    // --- U_TARGET: observed occupied surface voxel with sufficient variance ---
    if (e.classified && e.state == la3dm::State::OCCUPIED)
    {
      if (e.var < sigma2_thresh) continue;
      bool adj_free = false;
      for (int i = 0; i < 6 && !adj_free; ++i)
      {
        if (neighborState(x + kNeighbourOffsets[i][0] * e.size,
                          y + kNeighbourOffsets[i][1] * e.size,
                          z + kNeighbourOffsets[i][2] * e.size) == la3dm::State::FREE)
          adj_free = true;
      }
      if (adj_free)
        result.push_back({Eigen::Vector3d(x, y, z), TargetType::U_TARGET, e.var});
      continue;
    }

    // --- E-targets: unknown voxels adjacent to known voxels ---
    if (e.state != la3dm::State::UNKNOWN) continue;

    bool adj_occ = false, adj_free = false;
    for (int i = 0; i < 6; ++i)
    {
      la3dm::State ns = neighborState(x + kNeighbourOffsets[i][0] * e.size,
                                      y + kNeighbourOffsets[i][1] * e.size,
                                      z + kNeighbourOffsets[i][2] * e.size);
      if (ns == la3dm::State::OCCUPIED) adj_occ  = true;
      if (ns == la3dm::State::FREE)     adj_free = true;
      if (adj_occ && adj_free) break;
    }

    if (!adj_occ && !adj_free) continue;

    TargetType type = adj_occ ? TargetType::E_OCC : TargetType::E_FREE;
    result.push_back({Eigen::Vector3d(x, y, z), type, e.var});
  }

  return result;
}

std::vector<LeafEntry> extractLeavesFromIndex(
    const OccupiedUnknownIndex& idx,
    const Eigen::Vector3d& robot_pos,
    float r_max)
{
  std::shared_lock<std::shared_mutex> lk(idx.mtx);

  std::vector<LeafEntry> leaves;
  leaves.reserve(std::min(idx.cells.size(), size_t(32768)));

  const float r2 = r_max * r_max;
  const Eigen::Vector3f rp(robot_pos.x(), robot_pos.y(), robot_pos.z());

  for (const auto& kv : idx.cells)
  {
    const OccupiedUnknownIndex::VoxelData& vd = kv.second;
    if ((vd.pos - rp).squaredNorm() > r2) continue;
    LeafEntry e;
    e.pos        = vd.pos;
    e.state      = vd.state;
    e.var        = vd.var;
    e.size       = vd.size;
    e.classified = vd.classified;
    leaves.push_back(e);
  }

  return leaves;
}

std::vector<LeafEntry> extractAllLeavesFromIndex(const OccupiedUnknownIndex& idx)
{
  std::shared_lock<std::shared_mutex> lk(idx.mtx);

  std::vector<LeafEntry> leaves;
  leaves.reserve(idx.cells.size());

  for (const auto& kv : idx.cells)
  {
    const OccupiedUnknownIndex::VoxelData& vd = kv.second;
    LeafEntry e;
    e.pos        = vd.pos;
    e.state      = vd.state;
    e.var        = vd.var;
    e.size       = vd.size;
    e.classified = vd.classified;
    leaves.push_back(e);
  }

  return leaves;
}

std::vector<ClassifiedVoxel> classifyVoxels(
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    const Eigen::Vector3d& robot_pos,
    float r_max,
    float sigma2_thresh)
{
  // Legacy single-phase wrapper — caller must hold shared_lock for the full call.
  // Prefer the two-phase extractLeaves/classifyExtracted API for new callers
  // so the lock is released before the O(N) classification work.
  auto leaves = extractLeaves(map, robot_pos, r_max);
  return classifyExtracted(leaves, sigma2_thresh);
}

}  // namespace aeplanner
