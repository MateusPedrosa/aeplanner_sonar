#include <aeplanner/dwell_controller.h>
#include <tf/transform_datatypes.h>
#include <cmath>

namespace aeplanner
{

DwellController::DwellController(
    ros::NodeHandle& nh,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    std::shared_mutex& map_mutex)
  : map_(map), map_mutex_(map_mutex)
{
  // Publish hold-position as a PoseStamped to the move_base goal topic
  // (or any topic the navigation stack listens on for the AUV).
  hold_pub_         = nh.advertise<geometry_msgs::PoseStamped>("/viewplanner/hold_pose", 1, true);
  dwell_status_pub_ = nh.advertise<std_msgs::Float32>("/viewplanner/dwell_status", 1);
}

void DwellController::startDwell(const ScoredTarget&    t,
                                  const Eigen::Vector4d& robot_state,
                                  const Params&          params)
{
  target_      = t;
  dwell_start_ = ros::Time::now();
  active_      = true;

  hold_pose_.header.frame_id = "world";
  hold_pose_.header.stamp    = ros::Time::now();
  hold_pose_.pose.position.x = robot_state[0];
  hold_pose_.pose.position.y = robot_state[1];
  hold_pose_.pose.position.z = robot_state[2];
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(robot_state[3]),
                        hold_pose_.pose.orientation);
}

void DwellController::publishHoldPose()
{
  if (!active_) return;
  hold_pose_.header.stamp = ros::Time::now();
  hold_pub_.publish(hold_pose_);

  // Publish current Var_beta at target voxel for monitoring
  float var = (float)getTargetVarBeta();
  std_msgs::Float32 msg;
  msg.data = var;
  dwell_status_pub_.publish(msg);
}

DwellExitReason DwellController::checkExit(const Eigen::Vector4d& robot_state,
                                            const Params&          params)
{
  if (!active_) return DwellExitReason::NONE;

  double elapsed = (ros::Time::now() - dwell_start_).toSec();
  if (elapsed > params.T_dwell)
    return DwellExitReason::TIMEOUT;

  float var = (float)getTargetVarBeta();
  if (var < params.var_resolved_thresh)
    return DwellExitReason::RESOLVED;

  // FoV loss check: compute azimuth of target relative to robot yaw
  double dx  = target_.pos.x() - robot_state[0];
  double dy  = target_.pos.y() - robot_state[1];
  double az  = std::atan2(dy, dx) - robot_state[3];
  // Normalise to (-π, π]
  while (az >  M_PI) az -= 2.0 * M_PI;
  while (az < -M_PI) az += 2.0 * M_PI;
  double hfov_rad = params.hfov * M_PI / 180.0;
  if (std::fabs(az) > hfov_rad * 0.5)
    return DwellExitReason::FOV_LOST;

  return DwellExitReason::NONE;
}

double DwellController::getTargetVarBeta() const
{
  std::shared_lock<std::shared_mutex> lk(map_mutex_);
  la3dm::OcTreeNode node = map_->search(
      (float)target_.pos.x(), (float)target_.pos.y(), (float)target_.pos.z());
  return (double)node.get_var();
}

}  // namespace aeplanner
