#ifndef AEPLANNER_DWELL_CONTROLLER_H
#define AEPLANNER_DWELL_CONTROLLER_H

#include <memory>
#include <shared_mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <eigen3/Eigen/Dense>

#include <bgkloctomap/bgkloctomap.h>
#include <aeplanner/priority_scorer.h>
#include <aeplanner/param.h>

namespace aeplanner
{

enum class DwellExitReason
{
  NONE,        // still dwelling
  RESOLVED,    // Var_beta < var_resolved_thresh
  TIMEOUT,     // elapsed > T_dwell
  FOV_LOST     // target outside sonar azimuthal FoV
};

// DwellController: holds position and monitors the target voxel's Var_beta.
// The planner calls run() once per planning cycle while in DWELL state.
class DwellController
{
public:
  DwellController(ros::NodeHandle& nh,
                  const std::shared_ptr<la3dm::BGKLOctoMap>& map,
                  std::shared_mutex&                          map_mutex);

  // Begin a dwell at the current robot pose, monitoring target t.
  void startDwell(const ScoredTarget& t, const Eigen::Vector4d& robot_state,
                  const Params& params);

  // Poll exit condition. Returns NONE while still dwelling.
  DwellExitReason checkExit(const Eigen::Vector4d& robot_state,
                             const Params&          params);

  // Publish hold-position command (re-issue current pose as goal).
  void publishHoldPose();

private:
  ros::Publisher                          hold_pub_;
  ros::Publisher                          dwell_status_pub_;

  std::shared_ptr<la3dm::BGKLOctoMap>     map_;
  std::shared_mutex&                      map_mutex_;

  ScoredTarget  target_;
  ros::Time     dwell_start_;
  geometry_msgs::PoseStamped hold_pose_;
  bool active_ = false;

  double getTargetVarBeta() const;
};

}  // namespace aeplanner

#endif  // AEPLANNER_DWELL_CONTROLLER_H
