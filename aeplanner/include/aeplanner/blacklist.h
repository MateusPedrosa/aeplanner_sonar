#ifndef AEPLANNER_BLACKLIST_H
#define AEPLANNER_BLACKLIST_H

#include <unordered_map>
#include <cstdint>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

namespace aeplanner
{

class Blacklist
{
public:
  // n_fail: number of consecutive failures before a target is blacklisted
  // t_cooldown: duration (seconds) the target stays suppressed
  Blacklist(int n_fail, double t_cooldown);

  // Increment fail counter for a target at position pos (snapped to grid key).
  // If fail count reaches n_fail, starts the cooldown timer.
  void recordFailure(const Eigen::Vector3d& pos, double resolution);

  // Reset fail counter on success (target was resolved).
  void recordSuccess(const Eigen::Vector3d& pos, double resolution);

  // Returns true if the target is currently blacklisted (in cooldown).
  bool isBlacklisted(const Eigen::Vector3d& pos, double resolution) const;

  // Returns the current fail count (0 if unknown).
  int getFailCount(const Eigen::Vector3d& pos, double resolution) const;

  // Remove expired entries to prevent unbounded growth.
  void pruneExpired();

private:
  struct Entry
  {
    int        fail_count     = 0;
    ros::Time  cooldown_until;   // default-constructed = ros::Time(0)
  };

  uint64_t makeKey(const Eigen::Vector3d& pos, double resolution) const;

  int    n_fail_;
  double t_cooldown_;
  std::unordered_map<uint64_t, Entry> entries_;
};

}  // namespace aeplanner

#endif  // AEPLANNER_BLACKLIST_H
