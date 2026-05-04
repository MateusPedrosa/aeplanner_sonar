#include <aeplanner/voxel_classifier.h>
#include <unordered_map>
#include <cmath>
#include <future>
#include <thread>

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

  // Classify in parallel: state_index is read-only after construction, safe for concurrent reads.
  const size_t N      = leaves.size();
  const size_t n_jobs = std::min<size_t>(std::thread::hardware_concurrency(), N);
  const size_t chunk  = (N + n_jobs - 1) / n_jobs;

  std::vector<std::vector<ClassifiedVoxel>> partial(n_jobs);
  std::vector<std::future<void>> futures;
  futures.reserve(n_jobs);

  for (size_t job = 0; job < n_jobs; ++job)
  {
    const size_t lo = job * chunk;
    const size_t hi = std::min(lo + chunk, N);
    futures.push_back(std::async(std::launch::async, [&, lo, hi, job]()
    {
      std::vector<ClassifiedVoxel>& out = partial[job];
      for (size_t i = lo; i < hi; ++i)
      {
        const LeafEntry& e = leaves[i];
        const float x = e.pos.x(), y = e.pos.y(), z = e.pos.z();

        if (e.state == la3dm::State::OCCUPIED)
        {
          if (e.var < sigma2_thresh) continue;
          bool adj_free = false;
          for (int k = 0; k < 6 && !adj_free; ++k)
          {
            if (neighborState(x + kNeighbourOffsets[k][0] * e.size,
                              y + kNeighbourOffsets[k][1] * e.size,
                              z + kNeighbourOffsets[k][2] * e.size) == la3dm::State::FREE)
              adj_free = true;
          }
          if (adj_free)
            out.push_back({Eigen::Vector3d(x, y, z), TargetType::U_TARGET, e.var});
          continue;
        }

        if (e.state != la3dm::State::UNKNOWN) continue;

        bool adj_occ = false, adj_free = false;
        for (int k = 0; k < 6; ++k)
        {
          la3dm::State ns = neighborState(x + kNeighbourOffsets[k][0] * e.size,
                                          y + kNeighbourOffsets[k][1] * e.size,
                                          z + kNeighbourOffsets[k][2] * e.size);
          if (ns == la3dm::State::OCCUPIED) adj_occ  = true;
          if (ns == la3dm::State::FREE)     adj_free = true;
          if (adj_occ && adj_free) break;
        }

        if (!adj_occ && !adj_free) continue;

        TargetType type = adj_occ ? TargetType::E_OCC : TargetType::E_FREE;
        out.push_back({Eigen::Vector3d(x, y, z), type, e.var});
      }
    }));
  }

  std::vector<ClassifiedVoxel> result;
  result.reserve(N / 4);
  for (size_t job = 0; job < n_jobs; ++job)
  {
    futures[job].wait();
    auto& p = partial[job];
    result.insert(result.end(), p.begin(), p.end());
  }

  return result;
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
