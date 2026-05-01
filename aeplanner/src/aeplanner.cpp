#include <aeplanner/aeplanner.h>
#include <tf2/utils.h>

namespace aeplanner
{
AEPlanner::AEPlanner(const ros::NodeHandle& nh)
  : nh_(nh)
  , as_(nh_, "make_plan", boost::bind(&AEPlanner::execute, this, _1), false)
  , point_sub_(nh_.subscribe("pointcloud", 1, &AEPlanner::cloudCallback, this))
  , agent_pose_sub_(nh_.subscribe("agent_pose", 1, &AEPlanner::agentPoseCallback, this))
  , rrt_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("rrtree", 1000))
  , bbx_marker_pub_(nh_.advertise<visualization_msgs::Marker>("bbx", 1000, true)) // Latched
  , viewpoints_pub_(nh_.advertise<geometry_msgs::PoseArray>("sonar_viewpoints", 1, true)) // Latched
  , reevaluate_server_(nh_.advertiseService("reevaluate", &AEPlanner::reevaluate, this))
  , current_state_initialized_(false)
  , ot_(NULL)
{
  params_ = readParams();
  as_.start();

  ot_ = std::make_shared<la3dm::BGKLOctoMap>(params_.resolution, params_.block_depth, params_.sf2, params_.ell, params_.free_thresh, params_.occupied_thresh, params_.var_thresh, params_.prior_A, params_.prior_B, params_.theta_bw, params_.phi_bw, params_.free_ray_range_weight);

  la3dm::OcTreeNode::tau_var  = params_.tau_var;
  la3dm::OcTreeNode::tau_info = params_.tau_info;

  m_pub_occ_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/occupied_cells_vis_array", params_.resolution);
  // m_pub_free_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/free_cells_vis_array", params_.resolution);
  // m_pub_free_txt_ = new la3dm::TextMarkerArrayPub(nh_, "/bgkloctomap/free_cells_txt_vis_array", params_.resolution);
  m_pub_unc_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/uncertain_cells_vis_array", params_.resolution);
  // m_pub_unk_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/unknown_cells_vis_array", params_.resolution);
  m_pub_var_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/variance_vis_array", params_.resolution);

  double viz_period = (params_.viz_rate > 0.0) ? (1.0 / params_.viz_rate) : 0.5;
  viz_timer_ = nh_.createTimer(ros::Duration(viz_period), &AEPlanner::publishMapViz, this);

  // Initialise Target Priority Map — queries the map directly under shared_lock.
  TPMParams tpm_params;
  tpm_params.sigma2_thresh = params_.sigma2_thresh;
  tpm_params.w_frontier    = params_.w_frontier;
  tpm_params.cluster_norm  = params_.cluster_norm;
  tpm_params.R_cluster     = params_.R_cluster;
  tpm_params.r_max         = (float)params_.r_max;
  tpm_params.n_fail        = params_.N_fail;
  tpm_params.t_cooldown    = params_.T_cooldown;
  tpm_params.nbv_k         = params_.nbv_k;
  tpm_params.resolution    = params_.resolution;
  tpm_params.world_frame   = params_.world_frame;
  tpm_params.min_cluster_size   = params_.min_cluster_size;
  tpm_params.min_u_cluster_size = params_.min_u_cluster_size;

  tpm_ = std::make_unique<TargetPriorityMap>(nh_, ot_, ot_mutex_, tpm_params);

  double tpm_period = (params_.tpm_rate > 0.0) ? (1.0 / params_.tpm_rate) : 0.5;
  tpm_timer_ = nh_.createTimer(ros::Duration(tpm_period),
                               &TargetPriorityMap::update,
                               tpm_.get());

  state_pub_ = nh_.advertise<std_msgs::String>("/viewplanner/state", 1, true);

  hemisphere_sampler_ = std::make_unique<HemisphereSampler>();
  frontier_sampler_   = std::make_unique<FrontierSampler>();

  state_machine_    = std::make_unique<PlannerStateMachine>();
  dwell_controller_ = std::make_unique<DwellController>(nh_, ot_, ot_mutex_);
}

void AEPlanner::execute(const aeplanner::aeplannerGoalConstPtr& goal)
{
  aeplanner::aeplannerResult result;
  const ros::WallTime t_exec_start = ros::WallTime::now();
  // Thin wrapper: log state + elapsed time, then call setSucceeded.
  auto succeed = [&](const char* state_tag) {
    ROS_WARN_STREAM_THROTTLE(1.0,
      "[EXEC_TIMING] state=" << state_tag
      << "  elapsed=" << (ros::WallTime::now() - t_exec_start).toSec() << "s");
    as_.setSucceeded(result);
  };

  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_initialized_)
  {
    ROS_WARN("Agent's pose not yet received");
    ROS_WARN("Make sure it is being published and correctly mapped");
    succeed("NO_POSE"); return;
  }
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    succeed("NO_MAP"); return;
  }

  // Drain scan points queued by cloudCallback into the occupied-voxel snapshot.
  // No ot_mutex_ acquired — cloudCallback pushes raw PCL points (already in
  // world frame) to pending_occupied_points_ under pending_mutex_, and execute()
  // (single-threaded action-server) drains them here under the same cheap mutex.
  planning_snapshot_.resolution = ot_->get_resolution();
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    for (const auto& pt : pending_occupied_points_)
      planning_snapshot_.insert(pt.x(), pt.y(), pt.z());
    pending_occupied_points_.clear();
  }

  // State machine tick — determines EXPLORE/RESOLVE/DWELL/DONE
  std::vector<ScoredTarget> tpm_targets;
  if (tpm_) tpm_targets = tpm_->getTargets();

  PlannerState planner_state =
      state_machine_->tick(tpm_targets, current_state_, params_);

  // Publish current state for debugging
  {
    std_msgs::String state_msg;
    state_msg.data = plannerStateToString(planner_state);
    state_pub_.publish(state_msg);
  }

  // Pause TPM during RESOLVE: extractLeaves() holds shared_lock for ~1 s,
  // starving cloudCallback while the robot is already committed to a target.
  if (tpm_) tpm_->setPaused(planner_state == PlannerState::RESOLVE);

  // Handle DWELL: skip RRT, hold position and monitor target voxel
  if (planner_state == PlannerState::DWELL)
  {
    dwell_controller_->publishHoldPose();
    DwellExitReason reason = dwell_controller_->checkExit(current_state_, params_);
    if (reason != DwellExitReason::NONE)
    {
      const ScoredTarget& dt = state_machine_->dwellTarget();
      if (reason == DwellExitReason::RESOLVED)
      {
        ROS_INFO("[DWELL] Target resolved (Var_beta dropped). Recording success.");
        if (tpm_) tpm_->recordSuccess(dt.pos);
      }
      else
      {
        ROS_WARN_STREAM("[DWELL] Exiting DWELL: "
                        << (reason == DwellExitReason::TIMEOUT ? "TIMEOUT" : "FOV_LOST"));
        if (tpm_) tpm_->recordFailure(dt.pos);
      }
      state_machine_->exitDwell();
    }
    // Return the current pose as goal so rpl_exploration keeps the robot
    // stationary and immediately replans (avoiding the "no frontiers →
    // exploration complete" branch).
    result.is_clear = true;
    result.pose.pose = vecToPose(current_state_);
    result.pose.header.stamp    = ros::Time::now();
    result.pose.header.frame_id = params_.world_frame;
    succeed("DWELL"); return;
  }

  // Handle DONE: no targets at all
  if (planner_state == PlannerState::DONE)
  {
    ROS_INFO("[VIEWPLANNER] No targets remaining. Mission complete.");
    result.is_clear = false;
    succeed("DONE"); return;
  }

  // ── RESOLVE: committed navigation to best viewpoint ──────────────────────
  // Select the best candidate once, then step toward it with forward-facing
  // yaw. DWELL is entered only when the robot arrives (dist < dwell_arrival_thresh)
  if (planner_state == PlannerState::RESOLVE)
  {
    // Invalidate any stale commitment if we just re-entered RESOLVE.
    if (prev_planner_state_ != PlannerState::RESOLVE &&
        prev_planner_state_ != PlannerState::DWELL)
    {
      has_committed_viewpoint_ = false;
      resolve_waypoints_.clear();
      resolve_wp_idx_ = 0;
      state_machine_->exitResolveCommitment();
    }

    prev_planner_state_ = PlannerState::RESOLVE;

    // ── Select viewpoint once per RESOLVE episode ─────────────────────────
    if (!has_committed_viewpoint_)
    {
      if (tpm_targets.empty())
      {
        result.is_clear             = true;
        result.pose.pose            = vecToPose(current_state_);
        result.pose.header.stamp    = ros::Time::now();
        result.pose.header.frame_id = params_.world_frame;
        succeed("RESOLVE_HOLD"); return;
      }

      // Build priority cache once per selection episode — it is robot-position-based,
      // not target-specific, so one build covers all targets in the loop below.
      priority_cache_valid_ = false;
      computePriorityCache();

      int winning_ti = -1;
      double winning_best_score = -1.0;
      std::vector<std::pair<Eigen::Vector4d, double>> scored_candidates;

      for (int ti = 0; ti < static_cast<int>(tpm_targets.size()); ++ti)
      {
        const ScoredTarget& top = tpm_targets[ti];

        DirectedSampler* sampler =
            (top.type == TargetType::E_FREE &&
             top.w_normal < params_.w_normal_thresh)
            ? static_cast<DirectedSampler*>(frontier_sampler_.get())
            : static_cast<DirectedSampler*>(hemisphere_sampler_.get());

        auto candidates = sampler->sampleCandidates(top, current_state_, ot_, params_);

        double best_score   = -1.0;
        int    n_valid_cands = 0;
        std::vector<std::pair<Eigen::Vector4d, double>> cands_this_target;

        for (const CandidatePose& cp : candidates)
        {
          if (!isInsideBoundaries(cp.state)) continue;
          {
            std::shared_lock<std::shared_mutex> lk(ot_mutex_);
            if (ot_->search(cp.state[0], cp.state[1], cp.state[2]).get_state()
                    == la3dm::State::UNKNOWN)
              continue;
          }
          n_valid_cands++;
          auto [gain, yaw] = gainCubature(cp.state);
          double dist  = (cp.state.head<3>() - current_state_.head<3>()).norm();
          double score = gain * std::exp(-params_.lambda_dist * dist);
          cands_this_target.push_back({cp.state, score});
          if (score > best_score)
          {
            best_score           = score;
            committed_viewpoint_ = cp.state;
          }
        }

        if (n_valid_cands == 0 || best_score <= 0.0)
        {
          ROS_WARN_STREAM("[RESOLVE] Target #" << ti
              << " (" << top.pos.transpose() << ") — "
              << (n_valid_cands == 0 ? "no valid candidates" : "all zero gain")
              << "; recording failure, trying next target.");
          if (tpm_) tpm_->recordFailure(top.pos);
          continue;
        }

        // Found a viable target — commit to it.
        winning_ti         = ti;
        winning_best_score = best_score;
        scored_candidates  = std::move(cands_this_target);
        committed_target_        = top;
        has_committed_viewpoint_ = true;
        state_machine_->enterResolveCommitment();
        resolve_waypoints_.clear();
        resolve_wp_idx_ = 0;
        ROS_INFO_STREAM("[RESOLVE] Committed viewpoint ("
          << committed_viewpoint_[0] << ", " << committed_viewpoint_[1]
          << ", " << committed_viewpoint_[2]
          << ") yaw=" << (committed_viewpoint_[3] * 180.0 / M_PI) << " deg"
          << "  score=" << best_score
          << "  (target #" << ti << " of " << tpm_targets.size() << ")");
        break;
      }

      if (winning_ti < 0)
      {
        ROS_WARN_STREAM("[RESOLVE] All " << tpm_targets.size()
            << " targets exhausted — no positive-gain candidates. Holding position.");
        result.is_clear             = true;
        result.pose.pose            = vecToPose(current_state_);
        result.pose.header.stamp    = ros::Time::now();
        result.pose.header.frame_id = params_.world_frame;
        succeed("RESOLVE_HOLD_NO_CAND"); return;
      }

      // ── Publish candidates + committed viewpoint (clears old RRT tree) ──
      {
        visualization_msgs::MarkerArray viz;

        // Clear previous RRT markers
        visualization_msgs::Marker del;
        del.action = visualization_msgs::Marker::DELETEALL;
        viz.markers.push_back(del);

        // Candidate viewpoints — yellow arrows scaled by score
        int cand_id = 0;
        for (const auto& [state, score] : scored_candidates)
        {
          visualization_msgs::Marker m;
          m.header.stamp    = ros::Time::now();
          m.header.frame_id = params_.world_frame;
          m.ns = "resolve_candidates";
          m.id = cand_id++;
          m.type   = visualization_msgs::Marker::ARROW;
          m.action = visualization_msgs::Marker::ADD;
          m.pose.position.x = state[0];
          m.pose.position.y = state[1];
          m.pose.position.z = state[2];
          tf::Quaternion cq; cq.setEuler(0.0, 0.0, state[3]);
          m.pose.orientation.x = cq.x(); m.pose.orientation.y = cq.y();
          m.pose.orientation.z = cq.z(); m.pose.orientation.w = cq.w();
          double rel = score / winning_best_score;
          m.scale.x = std::max(rel * 1.5, 0.1);
          m.scale.y = 0.12; m.scale.z = 0.12;
          m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8;
          m.lifetime = ros::Duration(30.0);
          viz.markers.push_back(m);
        }

        // Committed viewpoint — large green arrow
        {
          visualization_msgs::Marker m;
          m.header.stamp    = ros::Time::now();
          m.header.frame_id = params_.world_frame;
          m.ns = "resolve_committed"; m.id = 0;
          m.type   = visualization_msgs::Marker::ARROW;
          m.action = visualization_msgs::Marker::ADD;
          m.pose.position.x = committed_viewpoint_[0];
          m.pose.position.y = committed_viewpoint_[1];
          m.pose.position.z = committed_viewpoint_[2];
          tf::Quaternion vq; vq.setEuler(0.0, 0.0, committed_viewpoint_[3]);
          m.pose.orientation.x = vq.x(); m.pose.orientation.y = vq.y();
          m.pose.orientation.z = vq.z(); m.pose.orientation.w = vq.w();
          m.scale.x = 2.0; m.scale.y = 0.3; m.scale.z = 0.3;
          m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
          m.lifetime = ros::Duration(0);  // persistent until overwritten
          viz.markers.push_back(m);
        }

        // Committed target — magenta sphere at the U_TARGET position
        {
          visualization_msgs::Marker m;
          m.header.stamp    = ros::Time::now();
          m.header.frame_id = params_.world_frame;
          m.ns = "resolve_target"; m.id = 0;
          m.type   = visualization_msgs::Marker::SPHERE;
          m.action = visualization_msgs::Marker::ADD;
          m.pose.position.x = committed_target_.pos.x();
          m.pose.position.y = committed_target_.pos.y();
          m.pose.position.z = committed_target_.pos.z();
          m.pose.orientation.w = 1.0;
          m.scale.x = m.scale.y = m.scale.z = 1.0;
          m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0;
          m.lifetime = ros::Duration(0);
          viz.markers.push_back(m);
        }

        rrt_marker_pub_.publish(viz);
      }
    }

    // ── Ensure we have a planned path ─────────────────────────────────────
    if (resolve_waypoints_.empty())
    {
      resolve_waypoints_ = planPathToGoal(committed_viewpoint_);
      resolve_wp_idx_    = 0;

      if (resolve_waypoints_.empty())
      {
        ROS_WARN("[RESOLVE] planPathToGoal failed; re-selecting viewpoint next iteration.");
        has_committed_viewpoint_ = false;
        state_machine_->exitResolveCommitment();
        // Hold current position and re-evaluate next tick.
        result.is_clear             = true;
        result.pose.pose            = vecToPose(current_state_);
        result.pose.header.stamp    = ros::Time::now();
        result.pose.header.frame_id = params_.world_frame;
        succeed("RESOLVE_PATH_FAIL"); return;
      }

      // Publish the full planned path as a LINE_STRIP
      {
        visualization_msgs::MarkerArray viz;
        visualization_msgs::Marker path_marker;
        path_marker.header.stamp    = ros::Time::now();
        path_marker.header.frame_id = params_.world_frame;
        path_marker.ns = "resolve_path"; path_marker.id = 0;
        path_marker.type   = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point p0;
        p0.x = current_state_[0]; p0.y = current_state_[1]; p0.z = current_state_[2];
        path_marker.points.push_back(p0);
        for (const auto& wp : resolve_waypoints_)
        {
          geometry_msgs::Point p;
          p.x = wp[0]; p.y = wp[1]; p.z = wp[2];
          path_marker.points.push_back(p);
        }
        path_marker.scale.x = 0.02;
        path_marker.color.r = 0.0; path_marker.color.g = 0.9;
        path_marker.color.b = 0.2; path_marker.color.a = 0.9;
        path_marker.lifetime = ros::Duration(0);
        viz.markers.push_back(path_marker);
        rrt_marker_pub_.publish(viz);
      }
    }

    // ── Follow waypoints ───────────────────────────────────────────────────
    double dist_to_wp =
        (current_state_.head<3>() - resolve_waypoints_[resolve_wp_idx_].head<3>()).norm();

    bool is_final_wp = (resolve_wp_idx_ == (int)resolve_waypoints_.size() - 1);
    double arrival_thresh = is_final_wp ? params_.dwell_arrival_thresh
                                        : params_.wp_arrival_threshold;
    if (dist_to_wp < arrival_thresh)
    {
      ++resolve_wp_idx_;
      if (resolve_wp_idx_ >= (int)resolve_waypoints_.size())
      {
        ROS_INFO("[RESOLVE] Arrived at viewpoint. Entering DWELL.");
        state_machine_->enterDwell(committed_target_);
        dwell_controller_->startDwell(committed_target_, current_state_, params_);
        has_committed_viewpoint_ = false;
        state_machine_->exitResolveCommitment();
        resolve_waypoints_.clear();
        resolve_wp_idx_ = 0;

        result.is_clear             = true;
        result.pose.pose            = vecToPose(current_state_);
        result.pose.header.stamp    = ros::Time::now();
        result.pose.header.frame_id = params_.world_frame;
        succeed("RESOLVE_ARRIVED"); return;
      }
    }

    // Proactively validate all remaining waypoint segments against the current
    // map before commanding any movement. This catches map-update races before
    // the robot reaches a blocked segment rather than after.
    {
      bool path_blocked = false;
      for (int k = resolve_wp_idx_; k < (int)resolve_waypoints_.size(); ++k)
      {
        const Eigen::Vector4d& from = (k == resolve_wp_idx_)
                                      ? current_state_
                                      : resolve_waypoints_[k - 1];
        if (collisionLine(from, resolve_waypoints_[k], params_.bounding_radius))
        {
          ROS_WARN("[RESOLVE] Upcoming path segment %d blocked; re-planning.", k);
          path_blocked = true;
          break;
        }
      }
      if (path_blocked)
      {
        resolve_waypoints_.clear();
        resolve_wp_idx_ = 0;
        // Hold current position and re-plan next tick.
        result.is_clear             = true;
        result.pose.pose            = vecToPose(current_state_);
        result.pose.header.stamp    = ros::Time::now();
        result.pose.header.frame_id = params_.world_frame;
        succeed("RESOLVE_PATH_BLOCKED"); return;
      }
    }

    // Step toward the current waypoint
    const Eigen::Vector4d& wp = resolve_waypoints_[resolve_wp_idx_];
    Eigen::Vector3d diff   = wp.head<3>() - current_state_.head<3>();
    double remaining       = diff.norm();
    Eigen::Vector3d dir    = diff / remaining;
    double step            = std::min(params_.extension_range, remaining);

    Eigen::Vector4d next_pose;
    next_pose.head<3>() = current_state_.head<3>() + step * dir;

    {
      double dist_to_goal =
          (committed_viewpoint_.head<3>() - next_pose.head<3>()).norm();
      if (dist_to_goal < params_.dwell_arrival_thresh)
        next_pose[3] = committed_viewpoint_[3];
      else if (params_.resolve_face_target)
        next_pose[3] = std::atan2(
            committed_target_.pos.y() - next_pose[1],
            committed_target_.pos.x() - next_pose[0]);
      else
        next_pose[3] = wp[3];
    }

    // ── Refresh committed viewpoint + target markers on every step ───────
    {
      visualization_msgs::MarkerArray viz;

      visualization_msgs::Marker vp;
      vp.header.stamp    = ros::Time::now();
      vp.header.frame_id = params_.world_frame;
      vp.ns = "resolve_committed"; vp.id = 0;
      vp.type   = visualization_msgs::Marker::ARROW;
      vp.action = visualization_msgs::Marker::ADD;
      vp.pose.position.x = committed_viewpoint_[0];
      vp.pose.position.y = committed_viewpoint_[1];
      vp.pose.position.z = committed_viewpoint_[2];
      tf::Quaternion vq; vq.setEuler(0.0, 0.0, committed_viewpoint_[3]);
      vp.pose.orientation.x = vq.x(); vp.pose.orientation.y = vq.y();
      vp.pose.orientation.z = vq.z(); vp.pose.orientation.w = vq.w();
      vp.scale.x = 2.0; vp.scale.y = 0.3; vp.scale.z = 0.3;
      vp.color.r = 0.0; vp.color.g = 1.0; vp.color.b = 0.0; vp.color.a = 1.0;
      vp.lifetime = ros::Duration(0);
      viz.markers.push_back(vp);

      visualization_msgs::Marker tgt;
      tgt.header.stamp    = ros::Time::now();
      tgt.header.frame_id = params_.world_frame;
      tgt.ns = "resolve_target"; tgt.id = 0;
      tgt.type   = visualization_msgs::Marker::SPHERE;
      tgt.action = visualization_msgs::Marker::ADD;
      tgt.pose.position.x = committed_target_.pos.x();
      tgt.pose.position.y = committed_target_.pos.y();
      tgt.pose.position.z = committed_target_.pos.z();
      tgt.pose.orientation.w = 1.0;
      tgt.scale.x = tgt.scale.y = tgt.scale.z = 1.0;
      tgt.color.r = 1.0; tgt.color.g = 0.0; tgt.color.b = 1.0; tgt.color.a = 1.0;
      tgt.lifetime = ros::Duration(0);
      viz.markers.push_back(tgt);

      rrt_marker_pub_.publish(viz);
    }

    result.is_clear             = true;
    result.pose.pose            = vecToPose(next_pose);
    result.pose.header.stamp    = ros::Time::now();
    result.pose.header.frame_id = params_.world_frame;
    succeed("RESOLVE_STEP"); return;
  }

  // ── EXPLORE: committed navigation to best E-target viewpoint ─────────────
  if (planner_state == PlannerState::EXPLORE)
  {
    // Clear stale commitment when re-entering from any other state.
    if (prev_planner_state_ != PlannerState::EXPLORE)
    {
      has_committed_explore_viewpoint_ = false;
      explore_waypoints_.clear();
      explore_wp_idx_ = 0;
    }

    // Try to commit to a new viewpoint if we don't have one.
    if (!has_committed_explore_viewpoint_)
    {
      // Collect E_OCC / E_FREE targets in priority order (tpm_targets is sorted).
      std::vector<const ScoredTarget*> e_targets;
      for (const auto& t : tpm_targets)
        if (t.type == TargetType::E_OCC || t.type == TargetType::E_FREE)
          e_targets.push_back(&t);

      if (!e_targets.empty())
      {
        const ScoredTarget& top = *e_targets[0];

        DirectedSampler* sampler =
            (top.type == TargetType::E_FREE &&
             top.w_normal < params_.w_normal_thresh)
            ? static_cast<DirectedSampler*>(frontier_sampler_.get())
            : static_cast<DirectedSampler*>(hemisphere_sampler_.get());

        auto candidates = sampler->sampleCandidates(top, current_state_, ot_, params_);

        priority_cache_valid_ = false;
        computePriorityCache();

        double best_score = -1.0;
        std::vector<std::pair<Eigen::Vector4d, double>> scored_candidates;
        for (const CandidatePose& cp : candidates)
        {
          if (!isInsideBoundaries(cp.state)) continue;
          if (planning_snapshot_.occupied(
                  static_cast<float>(cp.state[0]),
                  static_cast<float>(cp.state[1]),
                  static_cast<float>(cp.state[2]))) continue;
          double dist  = (cp.state.head<3>() - current_state_.head<3>()).norm();
          double score = gainExploration(cp.state) * std::exp(-params_.lambda_dist * dist);
          scored_candidates.push_back({cp.state, score});
          if (score > best_score)
          {
            best_score                   = score;
            committed_explore_viewpoint_ = cp.state;
          }
        }

        if (best_score >= params_.min_explore_gain)
        {
          committed_explore_target_          = top;
          has_committed_explore_viewpoint_   = true;
          explore_waypoints_.clear();
          explore_wp_idx_ = 0;
          ROS_INFO_STREAM("[EXPLORE] Committed viewpoint ("
            << committed_explore_viewpoint_[0] << ", "
            << committed_explore_viewpoint_[1] << ", "
            << committed_explore_viewpoint_[2] << ") yaw="
            << (committed_explore_viewpoint_[3] * 180.0 / M_PI) << " deg"
            << "  score=" << best_score);

          // Publish candidates + committed viewpoint
          {
            visualization_msgs::MarkerArray viz;

            visualization_msgs::Marker del;
            del.action = visualization_msgs::Marker::DELETEALL;
            viz.markers.push_back(del);

            int cand_id = 0;
            for (const auto& [state, score] : scored_candidates)
            {
              visualization_msgs::Marker m;
              m.header.stamp    = ros::Time::now();
              m.header.frame_id = params_.world_frame;
              m.ns = "explore_candidates"; m.id = cand_id++;
              m.type   = visualization_msgs::Marker::ARROW;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = state[0];
              m.pose.position.y = state[1];
              m.pose.position.z = state[2];
              tf::Quaternion cq; cq.setEuler(0.0, 0.0, state[3]);
              m.pose.orientation.x = cq.x(); m.pose.orientation.y = cq.y();
              m.pose.orientation.z = cq.z(); m.pose.orientation.w = cq.w();
              double rel = (best_score > 0) ? score / best_score : 0.0;
              m.scale.x = std::max(rel * 1.5, 0.1);
              m.scale.y = 0.12; m.scale.z = 0.12;
              m.color.r = 0.5; m.color.g = 0.8; m.color.b = 1.0; m.color.a = 0.8;
              m.lifetime = ros::Duration(30.0);
              viz.markers.push_back(m);
            }

            // Committed viewpoint — blue arrow
            {
              visualization_msgs::Marker m;
              m.header.stamp    = ros::Time::now();
              m.header.frame_id = params_.world_frame;
              m.ns = "explore_committed"; m.id = 0;
              m.type   = visualization_msgs::Marker::ARROW;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = committed_explore_viewpoint_[0];
              m.pose.position.y = committed_explore_viewpoint_[1];
              m.pose.position.z = committed_explore_viewpoint_[2];
              tf::Quaternion vq; vq.setEuler(0.0, 0.0, committed_explore_viewpoint_[3]);
              m.pose.orientation.x = vq.x(); m.pose.orientation.y = vq.y();
              m.pose.orientation.z = vq.z(); m.pose.orientation.w = vq.w();
              m.scale.x = 2.0; m.scale.y = 0.3; m.scale.z = 0.3;
              m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; m.color.a = 1.0;
              m.lifetime = ros::Duration(0);
              viz.markers.push_back(m);
            }

            // Committed E-target — cyan sphere
            {
              visualization_msgs::Marker m;
              m.header.stamp    = ros::Time::now();
              m.header.frame_id = params_.world_frame;
              m.ns = "explore_target"; m.id = 0;
              m.type   = visualization_msgs::Marker::SPHERE;
              m.action = visualization_msgs::Marker::ADD;
              m.pose.position.x = committed_explore_target_.pos.x();
              m.pose.position.y = committed_explore_target_.pos.y();
              m.pose.position.z = committed_explore_target_.pos.z();
              m.pose.orientation.w = 1.0;
              m.scale.x = m.scale.y = m.scale.z = 0.8;
              m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
              m.lifetime = ros::Duration(0);
              viz.markers.push_back(m);
            }

            rrt_marker_pub_.publish(viz);
          }
        }
        else if (best_score < 0.0)
        {
          // No geometrically valid candidate at all — record failure so the
          // blacklist eventually removes this target if it stays unreachable.
          ROS_WARN("[EXPLORE] No valid candidate for E-target; recording failure.");
          if (tpm_) tpm_->recordFailure(top.pos);
        }
        else
        {
          ROS_INFO_STREAM("[EXPLORE] Best score " << best_score
            << " below min_explore_gain " << params_.min_explore_gain << "; holding.");
        }
      }
    }

    // Follow the planned path to the committed viewpoint.
    if (has_committed_explore_viewpoint_)
    {
      // Plan a path if we don't have one yet.
      if (explore_waypoints_.empty())
      {
        explore_waypoints_ = planPathToGoal(committed_explore_viewpoint_);
        explore_wp_idx_    = 0;

        if (explore_waypoints_.empty())
        {
          ROS_WARN("[EXPLORE] planPathToGoal failed; recording failure.");
          if (tpm_) tpm_->recordFailure(committed_explore_target_.pos);
          has_committed_explore_viewpoint_ = false;
        }
        else
        {
          // Publish the planned path as a LINE_STRIP
          {
            visualization_msgs::MarkerArray viz;
            visualization_msgs::Marker path_m;
            path_m.header.stamp    = ros::Time::now();
            path_m.header.frame_id = params_.world_frame;
            path_m.ns = "explore_path"; path_m.id = 0;
            path_m.type   = visualization_msgs::Marker::LINE_STRIP;
            path_m.action = visualization_msgs::Marker::ADD;
            geometry_msgs::Point p0;
            p0.x = current_state_[0]; p0.y = current_state_[1]; p0.z = current_state_[2];
            path_m.points.push_back(p0);
            for (const auto& wp : explore_waypoints_)
            {
              geometry_msgs::Point p;
              p.x = wp[0]; p.y = wp[1]; p.z = wp[2];
              path_m.points.push_back(p);
            }
            path_m.scale.x = 0.08;
            path_m.color.r = 0.0; path_m.color.g = 0.7;
            path_m.color.b = 1.0; path_m.color.a = 0.9;
            path_m.lifetime = ros::Duration(0);
            viz.markers.push_back(path_m);
            rrt_marker_pub_.publish(viz);
          }
        }
      }

      // If we have a valid path, step along it.
      if (!explore_waypoints_.empty())
      {
        // Arrival at current waypoint?
        double dist_to_wp =
            (current_state_.head<3>() - explore_waypoints_[explore_wp_idx_].head<3>()).norm();
        bool is_final_wp = (explore_wp_idx_ == (int)explore_waypoints_.size() - 1);
        double arrival_thresh = is_final_wp ? params_.dwell_arrival_thresh
                                            : params_.wp_arrival_threshold;
        if (dist_to_wp < arrival_thresh)
        {
          ++explore_wp_idx_;
          if (explore_wp_idx_ >= (int)explore_waypoints_.size())
          {
            ROS_INFO("[EXPLORE] Arrived at viewpoint. Re-evaluating.");
            has_committed_explore_viewpoint_ = false;
            explore_waypoints_.clear();
            explore_wp_idx_ = 0;
            result.is_clear             = true;
            result.pose.pose            = vecToPose(current_state_);
            result.pose.header.stamp    = ros::Time::now();
            result.pose.header.frame_id = params_.world_frame;
            prev_planner_state_ = PlannerState::EXPLORE;
            succeed("EXPLORE_ARRIVED"); return;
          }
        }

        // Validate remaining path segments.
        {
          bool path_blocked = false;
          for (int k = explore_wp_idx_; k < (int)explore_waypoints_.size(); ++k)
          {
            const Eigen::Vector4d& from = (k == explore_wp_idx_)
                                          ? current_state_
                                          : explore_waypoints_[k - 1];
            if (collisionLine(from, explore_waypoints_[k], params_.bounding_radius))
            {
              ROS_WARN("[EXPLORE] Path segment %d blocked; re-planning.", k);
              path_blocked = true;
              break;
            }
          }
          if (path_blocked)
          {
            explore_waypoints_.clear();
            explore_wp_idx_ = 0;
            result.is_clear             = true;
            result.pose.pose            = vecToPose(current_state_);
            result.pose.header.stamp    = ros::Time::now();
            result.pose.header.frame_id = params_.world_frame;
            prev_planner_state_ = PlannerState::EXPLORE;
            succeed("EXPLORE_PATH_BLOCKED"); return;
          }
        }

        // Step toward the current waypoint.
        const Eigen::Vector4d& wp = explore_waypoints_[explore_wp_idx_];
        Eigen::Vector3d diff   = wp.head<3>() - current_state_.head<3>();
        double remaining       = diff.norm();
        Eigen::Vector3d dir    = diff / remaining;
        double step            = std::min(params_.extension_range, remaining);

        Eigen::Vector4d next_pose;
        next_pose.head<3>() = current_state_.head<3>() + step * dir;
        next_pose[3]        = wp[3];

        // Refresh committed viewpoint + target markers.
        {
          visualization_msgs::MarkerArray viz;

          visualization_msgs::Marker vp;
          vp.header.stamp    = ros::Time::now();
          vp.header.frame_id = params_.world_frame;
          vp.ns = "explore_committed"; vp.id = 0;
          vp.type   = visualization_msgs::Marker::ARROW;
          vp.action = visualization_msgs::Marker::ADD;
          vp.pose.position.x = committed_explore_viewpoint_[0];
          vp.pose.position.y = committed_explore_viewpoint_[1];
          vp.pose.position.z = committed_explore_viewpoint_[2];
          tf::Quaternion vq; vq.setEuler(0.0, 0.0, committed_explore_viewpoint_[3]);
          vp.pose.orientation.x = vq.x(); vp.pose.orientation.y = vq.y();
          vp.pose.orientation.z = vq.z(); vp.pose.orientation.w = vq.w();
          vp.scale.x = 2.0; vp.scale.y = 0.3; vp.scale.z = 0.3;
          vp.color.r = 0.0; vp.color.g = 0.5; vp.color.b = 1.0; vp.color.a = 1.0;
          vp.lifetime = ros::Duration(0);
          viz.markers.push_back(vp);

          visualization_msgs::Marker tgt;
          tgt.header.stamp    = ros::Time::now();
          tgt.header.frame_id = params_.world_frame;
          tgt.ns = "explore_target"; tgt.id = 0;
          tgt.type   = visualization_msgs::Marker::SPHERE;
          tgt.action = visualization_msgs::Marker::ADD;
          tgt.pose.position.x = committed_explore_target_.pos.x();
          tgt.pose.position.y = committed_explore_target_.pos.y();
          tgt.pose.position.z = committed_explore_target_.pos.z();
          tgt.pose.orientation.w = 1.0;
          tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.8;
          tgt.color.r = 0.0; tgt.color.g = 1.0; tgt.color.b = 1.0; tgt.color.a = 1.0;
          tgt.lifetime = ros::Duration(0);
          viz.markers.push_back(tgt);

          rrt_marker_pub_.publish(viz);
        }

        result.is_clear             = true;
        result.pose.pose            = vecToPose(next_pose);
        result.pose.header.stamp    = ros::Time::now();
        result.pose.header.frame_id = params_.world_frame;
        prev_planner_state_ = PlannerState::EXPLORE;
        succeed("EXPLORE_STEP"); return;
      }
    }
    // No committed path — hold position and re-evaluate next tick.
    result.is_clear             = true;
    result.pose.pose            = vecToPose(current_state_);
    result.pose.header.stamp    = ros::Time::now();
    result.pose.header.frame_id = params_.world_frame;
    prev_planner_state_         = PlannerState::EXPLORE;
    succeed("EXPLORE_HOLD"); return;
  }

  // Should never reach here with the new state machine.
  result.is_clear             = true;
  result.pose.pose            = vecToPose(current_state_);
  result.pose.header.stamp    = ros::Time::now();
  result.pose.header.frame_id = params_.world_frame;
  succeed("FALLTHROUGH_HOLD");
}

Eigen::Vector4d AEPlanner::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of
  // param_.max_sampling_radius
  Eigen::Vector4d point = Eigen::Vector4d::Zero();
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = params_.max_sampling_radius * 2.0 *
                 (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (pow(point[0], 2.0) + pow(point[1], 2.0) + pow(point[2], 2.0) >
           pow(params_.max_sampling_radius, 2.0));

  return point;
}

RRTNode* AEPlanner::chooseParent(RRTNode* node, double l, kdtree* tree)
{
  kdres* nearest = kd_nearest_range3(tree, node->state_[0], node->state_[1],
                                     node->state_[2], l + 0.5); // FIXME why +0.5?

  if (kd_res_size(nearest) <= 0)
    nearest = kd_nearest3(tree, node->state_[0], node->state_[1], node->state_[2]);
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RRTNode* node_nn = (RRTNode*)kd_res_item_data(nearest);

  RRTNode* best_node = node_nn;
  double best_node_cost = best_node->cost();
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    double node_cost = node_nn->cost();
    if (best_node and node_cost < best_node_cost)
    {
      best_node = node_nn;
      best_node_cost = node_cost;
    }

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}

std::vector<Eigen::Vector4d> AEPlanner::planPathToGoal(const Eigen::Vector4d& goal)
{
  kdtree* path_kd = kd_create(3);

  RRTNode* root = new RRTNode();
  root->state_ = current_state_;
  kd_insert3(path_kd, root->state_[0], root->state_[1], root->state_[2], root);

  RRTNode* best_goal_node = nullptr;
  double   best_goal_dist = std::numeric_limits<double>::max();

  int dbg_no_parent = 0, dbg_oob = 0, dbg_collision = 0, dbg_added = 0;
  for (int i = 0; i < params_.cutoff_iterations && ros::ok(); ++i)
  {
    // Goal-biased sampling: 30 % chance to sample the goal directly
    Eigen::Vector4d sample = ((double)rand() / RAND_MAX < 0.3)
        ? goal
        : current_state_ + sampleNewPoint();

    RRTNode* new_node = new RRTNode();
    new_node->state_  = sample;

    // RRT*: pick best-cost parent among near neighbours
    RRTNode* nearest = chooseParent(new_node, params_.extension_range, path_kd);
    if (!nearest) { delete new_node; ++dbg_no_parent; continue; }

    new_node->state_ = restrictDistance(nearest->state_, sample);

    // Validity checks: only reject out-of-bounds or collision with an OCCUPIED
    // voxel. Unknown space is traversable for a sparse underwater sonar — most
    // of the 3D volume is UNKNOWN even near the robot, so gating on UNKNOWN here
    // would block all path growth. Actual obstacle avoidance is handled by
    // collisionLine (which only rejects OCCUPIED), consistent with the rest of
    // the planner.
    if (!isInsideBoundaries(new_node->state_))
        { delete new_node; ++dbg_oob; continue; }
    if (collisionLine(nearest->state_, new_node->state_, params_.bounding_radius))
        { delete new_node; ++dbg_collision; continue; }
    ++dbg_added;

    new_node->parent_ = nearest;
    nearest->children_.push_back(new_node);
    kd_insert3(path_kd, new_node->state_[0], new_node->state_[1], new_node->state_[2], new_node);

    // RRT*: rewire near neighbours through new_node if cheaper
    rewire(path_kd, new_node, params_.extension_range, params_.bounding_radius, params_.d_overshoot_);

    double d = (new_node->state_.head<3>() - goal.head<3>()).norm();
    if (d < best_goal_dist) { best_goal_dist = d; best_goal_node = new_node; }
    if (d < params_.dwell_arrival_thresh) break;
  }

  // Extract waypoints by tracing parent pointers back to root, then reversing.
  // Root itself is excluded (robot is already there).
  std::vector<Eigen::Vector4d> path;
  if (best_goal_node)
  {
    for (RRTNode* cur = best_goal_node; cur->parent_; cur = cur->parent_)
      path.push_back(cur->state_);
    std::reverse(path.begin(), path.end());

    // Snap last waypoint to exact goal so the sensing yaw is preserved exactly.
    // Check the final edge first; if it collides, leave the last RRT node as-is
    // so the robot arrives as close as possible without entering occupied space.
    if (!path.empty())
    {
      const Eigen::Vector4d& pre = (path.size() >= 2) ? path[path.size()-2]
                                                       : root->state_;
      if (!collisionLine(pre, goal, params_.bounding_radius))
        path.back() = goal;
    }
    else
    {
      if (!collisionLine(current_state_, goal, params_.bounding_radius))
        path.push_back(goal);
    }

    // Yaw for every intermediate waypoint: direction of travel TO that waypoint
    // (backward-looking: from previous position toward current waypoint).
    // This ensures the robot faces its actual travel direction rather than
    // pre-rotating toward the next segment before reaching the current turn.
    for (int i = 0; i + 1 < (int)path.size(); ++i)
    {
      const Eigen::Vector3d& prev = (i == 0) ? current_state_.head<3>() : path[i-1].head<3>();
      Eigen::Vector3d d = path[i].head<3>() - prev;
      if (d.norm() > 1e-6) path[i][3] = std::atan2(d[1], d[0]);
    }
  }

  // rewire() updates parent_ pointers but leaves children_ intact, so the
  // full tree is still reachable from root via children_ — delete root cleans all nodes.
  kd_free(path_kd);
  delete root;

  ROS_INFO_STREAM("[RESOLVE] planPathToGoal: " << path.size()
    << " waypoints, best_goal_dist=" << best_goal_dist << " m"
    << "  [added=" << dbg_added << " no_parent=" << dbg_no_parent
    << " oob=" << dbg_oob
    << " collision=" << dbg_collision << "]");
  return path;
}

void AEPlanner::rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r,
                       double r_os)
{
  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  RRTNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->state_[0], new_node->state_[1],
                                     new_node->state_[2], l + 0.5); // FIXME why +0.5?
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    Eigen::Vector3d p1(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
    Eigen::Vector3d p2(node_nn->state_[0], node_nn->state_[1], node_nn->state_[2]);
    if (node_nn->cost() > new_node->cost() + (p1 - p2).norm())
    {
      if (!collisionLine(new_node->state_, node_nn->state_, r))
        node_nn->parent_ = new_node;
    }
    kd_res_next(nearest);
  }
}

Eigen::Vector4d AEPlanner::restrictDistance(Eigen::Vector4d nearest,
                                            Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1],
                            new_pos[2] - origin[2]);
  if (direction.norm() > params_.extension_range)
    direction = params_.extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

bool AEPlanner::reevaluate(aeplanner::Reevaluate::Request& req,
                           aeplanner::Reevaluate::Response& res)
{
  ROS_DEBUG_STREAM("Reevaluate start!");
  for (std::vector<geometry_msgs::Point>::iterator it = req.point.begin();
       it != req.point.end(); ++it)
  {
    Eigen::Vector4d pos(it->x, it->y, it->z, 0);
    std::pair<double, double> gain_response = gainCubature(pos);
    res.gain.push_back(gain_response.first);
    res.yaw.push_back(gain_response.second);
  }
  ROS_DEBUG_STREAM("Reevaluate done!");

  return true;
}

void AEPlanner::computePriorityCache()
{
  // Step 1 of PDF §7: rank voxels by priority = Var_beta × directional_imbalance.
  // Skip rebuild if the robot hasn't moved significantly — the leaf iteration
  // under shared_lock is O(N_all_leaves_in_sphere) and grows as FREE voxels
  // accumulate from sonar sweeps. A 1.5 m threshold is well below r_max so the
  // visible set barely changes, but the rebuild rate drops to once per 1.5 m of
  // travel rather than once per execute() tick.
  constexpr double kRebuildDist = 1.5;
  if (priority_cache_valid_ &&
      (current_state_.head<3>() - priority_cache_built_pos_).norm() < kRebuildDist)
    return;

  priority_cache_.clear();

  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;

  // Use the robot's current base position as the fixed planning origin.
  la3dm::point3f t_plan_p3f((float)current_state_[0],
                             (float)current_state_[1],
                             (float)current_state_[2]);
  Eigen::Vector3d t_plan(current_state_[0], current_state_[1], current_state_[2]);

  priority_cache_.reserve(4096);

  ros::WallTime t_cache_lock_start = ros::WallTime::now();
  {
    std::shared_lock<std::shared_mutex> lock(ot_mutex_);
    ros::WallTime t_cache_locked = ros::WallTime::now();

    for (auto it = ot->begin_leaf_in_sphere(t_plan_p3f, (float)params_.r_max);
         it != ot->end_leaf(); ++it)
    {
      la3dm::OcTreeNode &node = it.get_node();

      if (!node.has_active_info_matrix()) continue;

      la3dm::point3f p_loc = it.get_loc();
      Eigen::Vector3d p_k(p_loc.x(), p_loc.y(), p_loc.z());

      Eigen::Vector3d diff = p_k - t_plan;
      double dist = diff.norm();
      if (dist < 1e-6) continue;

      la3dm::point3f los_p3f((float)(diff.x()/dist),
                             (float)(diff.y()/dist),
                             (float)(diff.z()/dist));

      float priority = node.get_var();
      if (priority < 1e-8f) continue;

      float lam1_unused, lam2_unused;
      la3dm::point3f v_weak_p3f;
      node.get_2d_eigenstruct(los_p3f, lam1_unused, lam2_unused, v_weak_p3f);

      priority_cache_.push_back({p_k, priority,
                                 Eigen::Vector3d(v_weak_p3f.x(), v_weak_p3f.y(), v_weak_p3f.z())});
    }
    ros::WallTime t_cache_done = ros::WallTime::now();
    ROS_WARN_THROTTLE(5.0,
      "[CACHE_TIMING] lock_wait=%.3fs  lock_held=%.3fs  voxels_found=%zu",
      (t_cache_locked     - t_cache_lock_start).toSec(),
      (t_cache_done       - t_cache_locked).toSec(),
      priority_cache_.size());
  } // shared_lock released — map read complete

  // Top-K selection does not access the map; run without holding the lock.
  int K = params_.nbv_k;
  if ((int)priority_cache_.size() > K) {
    std::nth_element(priority_cache_.begin(), priority_cache_.begin() + K,
                     priority_cache_.end(),
                     [](const PriorityVoxel &a, const PriorityVoxel &b) {
                       return a.priority > b.priority;
                     });
    priority_cache_.resize(K);
  }

  priority_cache_valid_    = true;
  priority_cache_built_pos_ = current_state_.head<3>();
  ROS_DEBUG_STREAM("computePriorityCache: " << priority_cache_.size()
                   << " priority voxels (r_max=" << params_.r_max << ")");
}

std::pair<double, double> AEPlanner::gainCubature(Eigen::Vector4d state)
{
  // No lock — reads planning_snapshot_ built at the top of execute().
  // Occlusion check uses a linear ray march against the snapshot instead of
  // the RayCaster (which required a live map read per traversal step).
  tf::Vector3 base_origin(state[0], state[1], state[2]);

  tf::StampedTransform base_to_sensor;
  try {
    tf_listener_.lookupTransform(params_.robot_frame, params_.sensor_frame,
                                 ros::Time(0), base_to_sensor);
  } catch (tf::TransformException &ex) {
    ROS_ERROR_THROTTLE(1.0, "AEPlanner: %s. Assuming sensor is at base_link.", ex.what());
    base_to_sensor.setIdentity();
  }

  const double hfov_rad = params_.hfov * M_PI / 180.0;
  const double vfov_rad = params_.vfov * M_PI / 180.0;
  const double r_max    = params_.r_max;
  const double r_min    = params_.r_min;
  const float  snap_res = planning_snapshot_.resolution;

  tf::Quaternion base_quat;
  base_quat.setEuler(0.0, 0.0, state[3]);
  tf::Pose sensor_pose_in_world = tf::Pose(base_quat, base_origin) * base_to_sensor;

  Eigen::Vector3d t_cand(sensor_pose_in_world.getOrigin().x(),
                         sensor_pose_in_world.getOrigin().y(),
                         sensor_pose_in_world.getOrigin().z());

  tf::Matrix3x3 sensor_rot     = sensor_pose_in_world.getBasis();
  tf::Matrix3x3 sensor_rot_inv = sensor_rot.transpose();

  // Sonar z-axis in world frame R[:,2] — used for n_pred (PDF §6)
  tf::Vector3 z_cand_tf = sensor_rot * tf::Vector3(0.0, 0.0, 1.0);
  Eigen::Vector3d z_cand(z_cand_tf.x(), z_cand_tf.y(), z_cand_tf.z());

  double score = 0.0;

  for (const PriorityVoxel &pv : priority_cache_)
  {
    // Range check
    Eigen::Vector3d diff = pv.pos - t_cand;
    double range = diff.norm();
    if (range > r_max || range < r_min) continue;

    // FOV check: azimuth and elevation in sensor frame
    Eigen::Vector3d los_k = diff / range;
    tf::Vector3 los_k_sensor = sensor_rot_inv * tf::Vector3(los_k.x(), los_k.y(), los_k.z());
    double az = std::atan2(los_k_sensor.y(), los_k_sensor.x());
    double el = std::atan2(los_k_sensor.z(),
                           std::sqrt(los_k_sensor.x()*los_k_sensor.x() +
                                     los_k_sensor.y()*los_k_sensor.y()));
    if (std::fabs(az) > hfov_rad * 0.5 || std::fabs(el) > vfov_rad * 0.5) continue;

    // Boundary check
    Eigen::Vector4d pv4(pv.pos.x(), pv.pos.y(), pv.pos.z(), 0);
    if (!isInsideBoundaries(pv4)) continue;

    // Occlusion check: linear march from sensor toward voxel, step = resolution.
    // Any snapshot-occupied voxel closer than snap_res to the target voxel is
    // the target itself — stop without declaring occlusion.
    {
      const int n_steps = static_cast<int>(range / snap_res) + 1;
      bool occluded = false;
      for (int s = 1; s < n_steps && !occluded; ++s)
      {
        Eigen::Vector3d pt = t_cand + s * static_cast<double>(snap_res) * los_k;
        float dx = static_cast<float>(pt.x() - pv.pos.x());
        float dy = static_cast<float>(pt.y() - pv.pos.y());
        float dz = static_cast<float>(pt.z() - pv.pos.z());
        if (std::sqrt(dx*dx + dy*dy + dz*dz) <= snap_res) break;  // reached target
        if (planning_snapshot_.occupied(static_cast<float>(pt.x()),
                                        static_cast<float>(pt.y()),
                                        static_cast<float>(pt.z())))
          occluded = true;
      }
      if (occluded) continue;
    }

    // Predicted constraint direction n_pred = z_cand - dot(z_cand, los_k) * los_k  (PDF §6)
    Eigen::Vector3d n_pred = z_cand - z_cand.dot(los_k) * los_k;
    double n_norm = n_pred.norm();
    if (n_norm < 1e-8) continue;
    n_pred /= n_norm;

    // Alignment with weak direction (squared dot product — n and -n equivalent)
    double alignment = std::pow(n_pred.dot(pv.v_weak), 2);
    score += pv.priority * alignment;
  }

  ROS_DEBUG_STREAM("gainCubature at (" << state[0] << ", " << state[1] << ", " << state[2]
                   << ") yaw=" << (state[3] * 180.0 / M_PI) << " deg  score: " << score);
  return std::make_pair(score, state[3]);
}

double AEPlanner::gainExploration(const Eigen::Vector4d& state)
{
  // Set up sensor pose — same TF lookup as gainCubature.
  tf::Vector3 base_origin(state[0], state[1], state[2]);
  tf::StampedTransform base_to_sensor;
  try {
    tf_listener_.lookupTransform(params_.robot_frame, params_.sensor_frame,
                                 ros::Time(0), base_to_sensor);
  } catch (tf::TransformException& ex) {
    ROS_ERROR_THROTTLE(1.0, "AEPlanner: %s. Assuming sensor at base_link.", ex.what());
    base_to_sensor.setIdentity();
  }

  const double hfov_rad = params_.hfov * M_PI / 180.0;
  const double vfov_rad = params_.vfov * M_PI / 180.0;
  const double r_max    = params_.r_max;
  const double r_min    = params_.r_min;
  const float  snap_res = planning_snapshot_.resolution;

  tf::Quaternion base_quat;
  base_quat.setEuler(0.0, 0.0, state[3]);
  tf::Pose sensor_pose_in_world = tf::Pose(base_quat, base_origin) * base_to_sensor;

  Eigen::Vector3d t_sensor(sensor_pose_in_world.getOrigin().x(),
                            sensor_pose_in_world.getOrigin().y(),
                            sensor_pose_in_world.getOrigin().z());
  tf::Matrix3x3 sensor_rot = sensor_pose_in_world.getBasis();

  // Cast a uniform azimuth × elevation grid of rays through the FOV.
  // For each ray, march from r_min to r_max and count UNKNOWN voxels.
  // Hold shared_lock for the whole pass — bounded to N_az * N_el * (r_max/snap_res) lookups.
  int n_unknown = 0;
  {
    std::shared_lock<std::shared_mutex> lk(ot_mutex_);
    const int N_az = std::max(1, params_.n_explore_rays_az);
    const int N_el = std::max(1, params_.n_explore_rays_el);

    for (int ia = 0; ia < N_az; ++ia)
    for (int ie = 0; ie < N_el; ++ie)
    {
      double az = (N_az > 1) ? hfov_rad * (ia / (N_az - 1.0) - 0.5) : 0.0;
      double el = (N_el > 1) ? vfov_rad * (ie / (N_el - 1.0) - 0.5) : 0.0;

      // Ray direction in sensor frame, rotated to world frame.
      tf::Vector3 d_world_tf = sensor_rot * tf::Vector3(
          std::cos(el) * std::cos(az),
          std::cos(el) * std::sin(az),
          std::sin(el));
      Eigen::Vector3d ray_dir(d_world_tf.x(), d_world_tf.y(), d_world_tf.z());

      for (double range = r_min; range <= r_max; range += snap_res)
      {
        Eigen::Vector3d pt = t_sensor + range * ray_dir;
        float px = static_cast<float>(pt.x());
        float py = static_cast<float>(pt.y());
        float pz = static_cast<float>(pt.z());

        // Occupied voxel blocks the ray.
        if (planning_snapshot_.occupied(px, py, pz)) break;

        if (ot_->search(px, py, pz).get_state() == la3dm::State::UNKNOWN)
          ++n_unknown;
      }
    }
  }

  // Cubature component (lock-free — uses priority_cache_).
  double cub_score = gainCubature(state).first;

  return params_.w_unknown * static_cast<double>(n_unknown)
       + params_.w_cubature_explore * cub_score;
}

bool AEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}


bool AEPlanner::collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r)
{
  // No lock — reads planning_snapshot_ built at the top of execute().
  // Hash lookups replace per-cell octree searches, and the map writer is free
  // to insert new pointclouds throughout the entire planning loop.
  const float res = planning_snapshot_.resolution;

  octomap::point3d start((float)p1[0], (float)p1[1], (float)p1[2]);
  octomap::point3d end((float)p2[0], (float)p2[1], (float)p2[2]);

  float min_x = std::min((float)p1[0], (float)p2[0]) - (float)r;
  float min_y = std::min((float)p1[1], (float)p2[1]) - (float)r;
  float min_z = std::min((float)p1[2], (float)p2[2]) - (float)r;
  float max_x = std::max((float)p1[0], (float)p2[0]) + (float)r;
  float max_y = std::max((float)p1[1], (float)p2[1]) + (float)r;
  float max_z = std::max((float)p1[2], (float)p2[2]) + (float)r;
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  for (float x = min_x; x <= max_x; x += res) {
    for (float y = min_y; y <= max_y; y += res) {
      for (float z = min_z; z <= max_z; z += res) {
        if (planning_snapshot_.occupied(x, y, z)) {
          octomap::point3d pt(x, y, z);
          if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 || (end - pt).norm() < r)
            return true;
        }
      }
    }
  }

  return false;
}

void AEPlanner::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  tf::StampedTransform transform;
  try {
      // ros::Time(0) fetches the latest available transform, which is robust
      // with simulated time where the TF buffer may not yet hold the exact
      // message timestamp at the start of the simulation.
      tf_listener_.lookupTransform(params_.world_frame, msg->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR_THROTTLE(1.0, "cloudCallback TF lookup failed: %s", ex.what());
      return;
  }

  la3dm::point3f origin;
  tf::Vector3 translation = transform.getOrigin();
  tf::Quaternion orientation = transform.getRotation();

  tf::Matrix3x3 mat(orientation);
  tf::Vector3 up = mat.getColumn(2);

  la3dm::point3f sensor_up(up.x(), up.y(), up.z());

  origin.x() = (float) translation.x();
  origin.y() = (float) translation.y();
  origin.z() = (float) translation.z();

  size_t expected = (size_t)msg->width * msg->height * msg->point_step;
  if (msg->data.size() < expected) {
    ROS_WARN_THROTTLE(1.0, "Malformed PointCloud2: data.size()=%zu < expected=%zu, skipping",
                      msg->data.size(), expected);
    return;
  }

  sensor_msgs::PointCloud2 cloud_map;
  pcl_ros::transformPointCloud(params_.world_frame, *msg, cloud_map, tf_listener_);

  la3dm::PCLPointCloud::Ptr pcl_cloud (new la3dm::PCLPointCloud());
  pcl::fromROSMsg(cloud_map, *pcl_cloud);

  // Phase 1: prepare — downsampling, ray tracing, BGKL kernel training.
  // Does not access block_arr; no lock required.
  auto t_prepare_start = ros::WallTime::now();
  auto prepared = ot_->prepare_pointcloud_update(
      *pcl_cloud, origin, sensor_up,
      params_.ds_resolution, params_.free_resolution, params_.r_max);

  if (prepared.empty) return;

  // Phase 2: commit — writes block_arr. Hold unique_lock only for this fast phase.
  auto t_lock_wait_start = ros::WallTime::now();
  std::unique_lock<std::shared_mutex> lock(ot_mutex_);
  auto t_lock_acquired = ros::WallTime::now();
  ot_->commit_pointcloud_update(prepared);
  auto t_committed = ros::WallTime::now();

  lock.unlock();
  ROS_WARN_THROTTLE(2.0,
    "[CB_TIMING] prepare=%.3fs  wait_for_lock=%.3fs  commit=%.3fs",
    (t_lock_wait_start - t_prepare_start).toSec(),
    (t_lock_acquired   - t_lock_wait_start).toSec(),
    (t_committed       - t_lock_acquired).toSec());
  // 'prepared' destructor runs here, freeing BGKL3f objects (after lock release)

  // Queue scan points for the planning snapshot.
  // execute() drains this into planning_snapshot_ at the top of each tick.
  // No ot_mutex_ needed — pcl_cloud is already in world frame.
  {
    std::lock_guard<std::mutex> lk(pending_mutex_);
    pending_occupied_points_.reserve(pending_occupied_points_.size() + pcl_cloud->size());
    for (const auto& pt : *pcl_cloud)
      pending_occupied_points_.emplace_back(pt.x, pt.y, pt.z);
  }

  // Publish viewpoint history so callers can monitor the sonar update rate.
  // Latched: new subscribers always receive the buffered history without
  // periodic republish. Bounded ring — the message size is O(1) regardless of
  // run length, so per-callback serialisation cost stays constant.
  {
    geometry_msgs::Pose pose;
    pose.position.x = origin.x();
    pose.position.y = origin.y();
    pose.position.z = origin.z();
    // Orientation from the TF transform that was already computed above
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();

    constexpr size_t kMaxViewpointHistory = 200;
    if (viewpoints_msg_.poses.size() >= kMaxViewpointHistory)
      viewpoints_msg_.poses.erase(viewpoints_msg_.poses.begin());
    viewpoints_msg_.poses.push_back(pose);

    viewpoints_msg_.header.frame_id = params_.world_frame;
    viewpoints_msg_.header.stamp    = ros::Time::now();
    viewpoints_pub_.publish(viewpoints_msg_);
  }
}

void AEPlanner::publishMapViz(const ros::TimerEvent&)
{
  if (!ot_)
    return;

  // Skip the snapshot+lock entirely when no one is listening — the loop below
  // is O(N_leaves_in_sphere) under shared_lock and adds avoidable contention
  // with cloudCallback's writer lock as the map grows. Latched topics replay
  // the most recently published message on subscriber connect, so an RViz
  // session starting later still gets the previous snapshot until the next tick.
  if (m_pub_occ_->getNumSubscribers() == 0 &&
      m_pub_unc_->getNumSubscribers() == 0 &&
      m_pub_var_->getNumSubscribers() == 0)
    return;

  // Snapshot occupied/uncertain voxels under the shared_lock.
  // FREE and UNKNOWN are not visualised, so only ~(occupied+uncertain) entries
  // are collected — typically a tiny fraction of the total leaf count.
  // Marker building and ROS publish happen AFTER the lock is released so that
  // slow serialisation/I/O does not block cloudCallback's write lock.
  struct VoxelViz { float x, y, z, size, var; la3dm::State state; };
  std::vector<VoxelViz> snaps;
  snaps.reserve(8192);

  ros::WallTime t_viz_lock_start = ros::WallTime::now();
  {
    std::shared_lock<std::shared_mutex> lock(ot_mutex_);
    ros::WallTime t_viz_locked = ros::WallTime::now();

    la3dm::point3f viz_center((float)current_state_[0],
                              (float)current_state_[1],
                              (float)current_state_[2]);
    auto leaf_begin = (params_.max_vis_radius > 0.0 && current_state_initialized_)
                          ? ot_->begin_leaf_in_sphere(viz_center, (float)params_.max_vis_radius)
                          : ot_->begin_leaf();

    for (auto it = leaf_begin; it != ot_->end_leaf(); ++it) {
      auto state = it.get_node().get_state();
      if (state == la3dm::State::OCCUPIED || state == la3dm::State::UNCERTAIN) {
        la3dm::point3f p = it.get_loc();
        snaps.push_back({p.x(), p.y(), p.z(), it.get_size(),
                         it.get_node().get_var(), state});
      }
    }
    ros::WallTime t_viz_done = ros::WallTime::now();
    ROS_WARN_THROTTLE(5.0,
      "[VIZ_TIMING] lock_wait=%.3fs  lock_held=%.3fs  snaps=%zu  subs: occ=%u unc=%u var=%u",
      (t_viz_locked - t_viz_lock_start).toSec(),
      (t_viz_done   - t_viz_locked).toSec(),
      snaps.size(),
      m_pub_occ_->getNumSubscribers(),
      m_pub_unc_->getNumSubscribers(),
      m_pub_var_->getNumSubscribers());
  } // shared_lock released — map iteration complete

  // Build marker arrays and publish without holding the map lock.
  const float max_var_vis = 0.25f;
  m_pub_occ_->clear();
  m_pub_unc_->clear();
  m_pub_var_->clear();

  for (const auto& v : snaps) {
    if (v.state == la3dm::State::OCCUPIED) {
      m_pub_occ_->insert_point3d(v.x, v.y, v.z, params_.min_z, params_.max_z, v.size);
    } else {
      m_pub_unc_->insert_point3d_color(v.x, v.y, v.z, v.size, 1.0f, 1.0f, 0.0f);
    }
    std::string ns = (v.state == la3dm::State::OCCUPIED) ? "occupied" : "uncertain";
    m_pub_var_->insert_color_point3d(v.x, v.y, v.z, 0.0, max_var_vis, v.var, v.size, ns);
  }

  m_pub_occ_->publish();
  m_pub_unc_->publish();
  m_pub_var_->publish();

  if (!params_.boundary_min.empty() && !params_.boundary_max.empty()) {
    visualization_msgs::Marker bbx_marker =
        createBoundingBoxMarker(params_.boundary_min, params_.boundary_max, 0, "map");
    bbx_marker_pub_.publish(bbx_marker);
  }
}


void AEPlanner::agentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  current_state_[2] = msg.pose.position.z;
  current_state_[3] = tf2::getYaw(msg.pose.orientation);

  current_state_initialized_ = true;

  if (tpm_)
    tpm_->setRobotPos(Eigen::Vector3d(current_state_[0],
                                      current_state_[1],
                                      current_state_[2]));
}

geometry_msgs::Pose AEPlanner::vecToPose(Eigen::Vector4d state)
{
  tf::Vector3 origin(state[0], state[1], state[2]);
  double yaw = state[3];

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  tf::Pose pose_tf(quat, origin);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(pose_tf, pose);

  return pose;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float AEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1,
                                   const octomap::point3d& pt2, float lsq, float rsq,
                                   const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq);  // return distance squared to axis
  }
}

}  // namespace aeplanner
