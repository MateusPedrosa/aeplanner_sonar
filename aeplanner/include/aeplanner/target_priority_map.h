#ifndef AEPLANNER_TARGET_PRIORITY_MAP_H
#define AEPLANNER_TARGET_PRIORITY_MAP_H

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <bgkloctomap/bgkloctomap.h>

#include <aeplanner/voxel_classifier.h>
#include <aeplanner/frontier_detector.h>
#include <aeplanner/priority_scorer.h>
#include <aeplanner/blacklist.h>
#include <aeplanner/TargetList.h>

namespace aeplanner
{

struct TPMParams
{
  float  sigma2_thresh;    // min Var_beta for U-target (default 0.05)
  float  w_frontier;       // frontier bonus weight (default 0.6)
  float  cluster_norm;     // density normalisation (default 100.0)
  float  R_cluster;        // frontier clustering radius (default 2.0 m)
  float  r_max;            // map query radius (default 20.0 m)
  int    n_fail;           // failures before blacklisting (default 3)
  double t_cooldown;       // blacklist cooldown seconds (default 30.0)
  int    nbv_k;            // max targets to publish (default 1000)
  double resolution;       // voxel resolution for blacklist key (default 0.2 m)
  std::string world_frame; // TF frame for published messages (default "world")
  int    min_cluster_size;   // min voxels per E_TARGET cluster to consider as a target (default 5)
  int    min_u_cluster_size; // min voxels per U_TARGET cluster to consider as a target (default 1)
  double lambda_dist;        // exponential distance penalty (default 0.2)
};

// In-process Target Priority Map. Runs as a ros::Timer callback.
// Takes a brief shared_lock on ot_mutex_ to extract leaves from the map,
// then releases the lock before doing all classification and scoring work.
class TargetPriorityMap
{
public:
  TargetPriorityMap(ros::NodeHandle& nh,
                    const std::shared_ptr<la3dm::BGKLOctoMap>& ot,
                    std::shared_mutex& ot_mutex,
                    const TPMParams& params);

  // Timer callback — runs updateNow() unless paused.
  void update(const ros::TimerEvent&);

  // Synchronous update called explicitly at episode boundaries (EXPLORE arrival,
  // DWELL exit). Does not check the pause flag.
  void updateNow();

  // Called by the planner when it deems a target resolved (priority dropped).
  void recordSuccess(const Eigen::Vector3d& pos);

  // Called by the planner when a target was visited but not resolved.
  void recordFailure(const Eigen::Vector3d& pos);

  // Thread-safe snapshot of the current target list (for use by the sampler).
  std::vector<ScoredTarget> getTargets() const;

  // Update the robot position used for map radius query.
  void setRobotPos(const Eigen::Vector3d& pos);

  // Pause/resume the background timer callback.
  // All committed states (EXPLORE, RESOLVE, DWELL) pause the timer; updates
  // are driven explicitly via updateNow() at episode boundaries instead.
  void setPaused(bool p) { paused_.store(p, std::memory_order_relaxed); }

private:
  ros::Publisher                          targets_pub_;
  ros::Publisher                          viz_pub_;
  ros::Publisher                          clusters_pub_;
  ros::Publisher                          u_clusters_pub_;

  std::shared_ptr<la3dm::BGKLOctoMap>     ot_;
  std::shared_mutex&                      ot_mutex_;

  TPMParams params_;
  Blacklist blacklist_;

  Eigen::Vector3d robot_pos_;
  mutable std::mutex robot_pos_mutex_;

  std::vector<ScoredTarget> targets_;
  mutable std::mutex targets_mutex_;

  std::atomic<bool> paused_{ false };

  aeplanner::TargetList toROSMsg(const std::vector<ScoredTarget>& targets) const;
  void publishViz(const std::vector<ScoredTarget>& targets) const;
  void publishClustersViz(const std::vector<FrontierCluster>& clusters,
                          const std::vector<ClassifiedVoxel>& frontier_voxels) const;
  void publishUClustersViz(const std::vector<FrontierCluster>& u_clusters,
                           const std::vector<ClassifiedVoxel>& u_voxels) const;
};

}  // namespace aeplanner

#endif  // AEPLANNER_TARGET_PRIORITY_MAP_H
