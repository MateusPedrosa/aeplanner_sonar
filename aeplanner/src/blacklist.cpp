#include <aeplanner/blacklist.h>
#include <cmath>

namespace aeplanner
{

Blacklist::Blacklist(int n_fail, double t_cooldown)
  : n_fail_(n_fail), t_cooldown_(t_cooldown)
{}

uint64_t Blacklist::makeKey(const Eigen::Vector3d& pos, double resolution) const
{
  // Snap position to voxel grid and encode as a 64-bit integer.
  // Uses a simple cantor-style packing with limited range assumptions.
  const int64_t OFFSET = 1 << 20;  // supports ±1M voxels per axis
  int64_t ix = (int64_t)std::round(pos.x() / resolution) + OFFSET;
  int64_t iy = (int64_t)std::round(pos.y() / resolution) + OFFSET;
  int64_t iz = (int64_t)std::round(pos.z() / resolution) + OFFSET;
  return (uint64_t)(ix ^ (iy << 21) ^ (iz << 42));
}

void Blacklist::recordFailure(const Eigen::Vector3d& pos, double resolution)
{
  uint64_t key = makeKey(pos, resolution);
  Entry&   e   = entries_[key];
  e.fail_count++;
  if (e.fail_count >= n_fail_)
    e.cooldown_until = ros::Time::now() + ros::Duration(t_cooldown_);
}

void Blacklist::recordSuccess(const Eigen::Vector3d& pos, double resolution)
{
  uint64_t key = makeKey(pos, resolution);
  auto it = entries_.find(key);
  if (it != entries_.end())
  {
    it->second.fail_count    = 0;
    it->second.cooldown_until = ros::Time(0);
  }
}

bool Blacklist::isBlacklisted(const Eigen::Vector3d& pos, double resolution) const
{
  uint64_t key = makeKey(pos, resolution);
  auto it = entries_.find(key);
  if (it == entries_.end()) return false;
  if (it->second.cooldown_until.isZero()) return false;
  return ros::Time::now() < it->second.cooldown_until;
}

int Blacklist::getFailCount(const Eigen::Vector3d& pos, double resolution) const
{
  uint64_t key = makeKey(pos, resolution);
  auto it = entries_.find(key);
  return (it == entries_.end()) ? 0 : it->second.fail_count;
}

void Blacklist::pruneExpired()
{
  ros::Time now = ros::Time::now();
  for (auto it = entries_.begin(); it != entries_.end(); )
  {
    if (it->second.fail_count == 0 &&
        (it->second.cooldown_until.isZero() || now >= it->second.cooldown_until))
      it = entries_.erase(it);
    else
      ++it;
  }
}

}  // namespace aeplanner
