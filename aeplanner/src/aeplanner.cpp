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
  , gain_pub_(nh_.advertise<pigain::Node>("gain_node", 1000))
  , bbx_marker_pub_(nh_.advertise<visualization_msgs::Marker>("bbx", 1000, true)) // Latched
  , reevaluate_server_(nh_.advertiseService("reevaluate", &AEPlanner::reevaluate, this))
  , best_node_client_(nh_.serviceClient<pigain::BestNode>("best_node_server"))
  , current_state_initialized_(false)
  , ot_(NULL)
  , best_node_(NULL)
  , best_branch_root_(NULL)
{
  params_ = readParams();
  as_.start();

  ot_ = std::make_shared<la3dm::BGKLOctoMap>(params_.resolution, params_.block_depth, params_.sf2, params_.ell, params_.free_thresh, params_.occupied_thresh, params_.var_thresh, params_.prior_A, params_.prior_B, params_.theta_bw, params_.phi_bw, params_.free_ray_range_weight);

  la3dm::OcTreeNode::tau_var  = params_.tau_var;
  la3dm::OcTreeNode::tau_info = params_.tau_info;

  m_pub_occ_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/occupied_cells_vis_array", params_.resolution);
  m_pub_free_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/free_cells_vis_array", params_.resolution);
  // m_pub_free_txt_ = new la3dm::TextMarkerArrayPub(nh_, "/bgkloctomap/free_cells_txt_vis_array", params_.resolution);
  m_pub_unc_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/uncertain_cells_vis_array", params_.resolution);
  // m_pub_unk_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/unknown_cells_vis_array", params_.resolution);
  m_pub_var_ = new la3dm::MarkerArrayPub(nh_, "/bgkloctomap/variance_vis_array", params_.resolution);
  // Initialize kd-tree
  kd_tree_ = kd_create(3);

  double viz_period = (params_.viz_rate > 0.0) ? (1.0 / params_.viz_rate) : 0.5;
  viz_timer_ = nh_.createTimer(ros::Duration(viz_period), &AEPlanner::publishMapViz, this);

  // Initialise Target Priority Map (in-process, shares ot_ read-only)
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

  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_initialized_)
  {
    ROS_WARN("Agent's pose not yet received");
    ROS_WARN("Make sure it is being published and correctly mapped");
    as_.setSucceeded(result);
    return;
  }
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }

  // State machine tick — determines EXPLORE/RESOLVE/DWELL/REPOSITION/DONE
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
    // Return non-moving result while dwelling
    result.is_clear = false;
    as_.setSucceeded(result);
    return;
  }

  // Handle DONE: no targets at all
  if (planner_state == PlannerState::DONE)
  {
    ROS_INFO("[VIEWPLANNER] No targets remaining. Mission complete.");
    result.is_clear = false;
    as_.setSucceeded(result);
    return;
  }

  // REPOSITION: select a global target to bias sampling toward
  has_reposition_target_ = false;
  if (planner_state == PlannerState::REPOSITION && !tpm_targets.empty())
  {
    const ScoredTarget* global =
        selectGlobalTarget(tpm_targets, current_state_.head<3>(), params_.local_radius);
    if (global)
    {
      std::shared_lock<std::shared_mutex> map_lk(ot_mutex_);
      bool reachable = isLikelyReachable(
          current_state_.head<3>(), global->pos, ot_);
      map_lk.unlock();

      if (reachable)
      {
        reposition_target_     = *global;
        has_reposition_target_ = true;
        ROS_INFO_STREAM("[REPOSITION] Global target at ("
                        << global->pos.x() << ", "
                        << global->pos.y() << ", "
                        << global->pos.z() << ") priority=" << global->priority);
      }
    }
  }

  ROS_DEBUG("Init");
  RRTNode* root = initialize();
  ROS_DEBUG("expandRRT");
  if (root->gain_ > 0.25 or !root->children_.size() or // FIXME parameterize
      root->score(params_.lambda) < params_.zero_gain)
    expandRRT();
  else
    best_node_ = root->children_[0];

  // expandRRT can exhaust its sampling budget without finding any feasible
  // node (e.g. when the map is still mostly UNKNOWN right after the init
  // motion). Return a non-clear result with frontiers so rpl_exploration can
  // fall back to the RRT-to-frontiers branch instead of the action hanging.
  if (!best_node_)
  {
    ROS_WARN("expandRRT returned no feasible node; reporting non-clear result so driver can re-route.");
    result.is_clear = false;
    result.frontiers = getFrontiers();
    as_.setSucceeded(result);
    delete root;
    kd_free(kd_tree_);
    return;
  }

  ROS_DEBUG("getCopyOfParent");
  best_branch_root_ = best_node_->getCopyOfParentBranch();

  ROS_DEBUG("createRRTMarker");
  rrt_marker_pub_.publish(createRRTMarkerArray(root, params_.lambda));
  
  // Publish bounding box
  bbx_marker_pub_.publish(createBoundingBoxMarker(params_.boundary_min, params_.boundary_max, 0, params_.world_frame));

  ROS_DEBUG("publishRecursive");
  publishEvaluatedNodesRecursive(root);

  // If we just arrived at a RESOLVE viewpoint, transition to DWELL
  if (planner_state == PlannerState::RESOLVE && !tpm_targets.empty())
  {
    const ScoredTarget& top = tpm_targets[0];
    // Check if robot is already at standoff distance from the target
    double dist_to_target = (current_state_.head<3>() - top.pos).norm();
    if (dist_to_target < params_.d_standoff * 1.5)
    {
      ROS_INFO_STREAM("[RESOLVE] Arrived at standoff. Entering DWELL for target at ("
                      << top.pos.x() << ", " << top.pos.y() << ", " << top.pos.z() << ")");
      state_machine_->enterDwell(top);
      dwell_controller_->startDwell(top, current_state_, params_);
    }
  }

  ROS_DEBUG("extractPose");
  result.pose.pose = vecToPose(best_branch_root_->children_[0]->state_);
  ROS_INFO_STREAM("Best node score: " << best_node_->score(params_.lambda));
  if (best_node_->score(params_.lambda) > params_.zero_gain)
    result.is_clear = true;
  else
  {
    result.frontiers = getFrontiers();
    result.is_clear = false;
    delete best_branch_root_;
    best_branch_root_ = NULL;
  }
  as_.setSucceeded(result);

  ROS_DEBUG("Deleting/Freeing!");
  delete root;
  kd_free(kd_tree_);
  ROS_DEBUG("Done!");
}

RRTNode* AEPlanner::initialize()
{
  // Initialize kd-tree
  kd_tree_ = kd_create(3);
  best_node_ = NULL;
  RRTNode* root = NULL;

  if (best_branch_root_)
  {
    // Initialize with previous best branch

    // Discard root node from tree since we just were there...
    root = best_branch_root_->children_[0];
    root->parent_ = NULL;
    best_branch_root_->children_.clear();
    delete best_branch_root_;

    initializeKDTreeWithPreviousBestBranch(root);
    reevaluatePotentialInformationGainRecursive(root);
  }
  else
  {
    // Initialize without any previous branch
    root = new RRTNode();
    root->state_[0] = current_state_[0];
    root->state_[1] = current_state_[1];
    root->state_[2] = current_state_[2];
    kd_insert3(kd_tree_, root->state_[0], root->state_[1], root->state_[2], root);
  }

  return root;
}

void AEPlanner::initializeKDTreeWithPreviousBestBranch(RRTNode* root)
{
  RRTNode* current_node = root;
  do
  {
    kd_insert3(kd_tree_, current_node->state_[0], current_node->state_[1],
               current_node->state_[2], current_node);
    if (current_node->children_.size())
      current_node = current_node->children_[0];
  } while (current_node->children_.size());
}

void AEPlanner::reevaluatePotentialInformationGainRecursive(RRTNode* node)
{
  if (!priority_cache_valid_) computePriorityCache();
  std::pair<double, double> ret = gainCubature(node->state_); // FIXME use gp?
  node->state_[3] = ret.second;  // Assign yaw angle that maximizes g
  node->gain_ = ret.first;
  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
    reevaluatePotentialInformationGainRecursive(*node_it);
}

void AEPlanner::expandRRT()
{
  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;

  // Pre-compute the priority voxel cache once for this planning cycle.
  // All gainCubature calls within this expandRRT will reuse it, avoiding
  // the expensive per-node map iteration.
  priority_cache_valid_ = false;
  computePriorityCache();
  ROS_INFO_STREAM("[NBV] Priority cache: " << priority_cache_.size() << " voxels");

  // Expand an RRT tree and calculate information gain in every node
  ROS_DEBUG_STREAM("Entering expanding RRT");
  for (int n = 0; (n < params_.init_iterations or
                   (n < params_.cutoff_iterations and
                    best_node_->score(params_.lambda) < params_.zero_gain)) and
                  ros::ok();
       ++n)
  {
    ROS_DEBUG_STREAM("In expand RRT iteration: " << n);

    // --- One-shot diagnostics on the very first iteration ---
    if (n == 0) {
      ROS_WARN_STREAM("[DIAG] expandRRT: robot at ("
        << current_state_[0] << ", " << current_state_[1] << ", " << current_state_[2]
        << "), sampling_radius=" << params_.max_sampling_radius);
      // Count octree leaves by state near the robot
      std::shared_lock<std::shared_mutex> diag_lock(ot_mutex_);
      size_t free_ct = 0, occ_ct = 0, unk_ct = 0, unc_ct = 0;
      la3dm::point3f robot_pt((float)current_state_[0], (float)current_state_[1], (float)current_state_[2]);
      for (auto it = ot->begin_leaf_in_sphere(robot_pt, 10.0f); it != ot->end_leaf(); ++it) {
        la3dm::State s = (*it).get_state();
        if (s == la3dm::State::FREE)            ++free_ct;
        else if (s == la3dm::State::OCCUPIED)   ++occ_ct;
        else if (s == la3dm::State::UNCERTAIN)  ++unc_ct;
        else                                    ++unk_ct;
      }
      ROS_WARN_STREAM("[DIAG] octree leaves within 10m: free=" << free_ct
        << " occupied=" << occ_ct << " uncertain=" << unc_ct << " unknown=" << unk_ct);
      // Sample 5 random points inside radius and log their state
      for (int dbg = 0; dbg < 5; ++dbg) {
        Eigen::Vector4d o = sampleNewPoint();
        Eigen::Vector4d p = current_state_ + o;
        la3dm::OcTreeNode r = ot->search(p[0], p[1], p[2]);
        const char* st = (r.get_state()==la3dm::State::FREE) ? "FREE" :
                         (r.get_state()==la3dm::State::OCCUPIED) ? "OCC" :
                         (r.get_state()==la3dm::State::UNCERTAIN) ? "UNC" : "UNK";
        ROS_WARN_STREAM("[DIAG] sample " << dbg << ": ("
          << p[0] << "," << p[1] << "," << p[2] << ") -> " << st);
      }
    }
    // ---------------------------------------------------------

    RRTNode* new_node = new RRTNode();
    RRTNode* nearest;
    la3dm::OcTreeNode ot_result;

    // Sample new point around agent and check that
    // (1) it is within the boundaries
    // (2) it is in known space
    // (3) the path between the new node and it's parent does not contain any
    // obstacles

    bool sample_found = false;
    int unknown_count = 0, collision_count = 0, oob_count = 0;
    const int fast_tries = std::min(100, params_.max_sampling_attempts);

    auto try_candidate = [&](Eigen::Vector4d candidate) -> bool {
      new_node->state_ = candidate;
      nearest = chooseParent(new_node, params_.extension_range);
      new_node->state_ = restrictDistance(nearest->state_, new_node->state_);
      {
        std::shared_lock<std::shared_mutex> lock(ot_mutex_);
        ot_result = ot->search(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
      }
      if (ot_result.get_state() == la3dm::State::UNKNOWN) { ++unknown_count; return false; }
      if (!isInsideBoundaries(new_node->state_))           { ++oob_count;     return false; }
      if (collisionLine(nearest->state_, new_node->state_, params_.bounding_radius))
                                                           { ++collision_count; return false; }
      return true;
    };

    // Directed sampling: prefer REPOSITION override if set, else top TPM target.
    // tpm_snapshot kept alive here so active_target pointer remains valid.
    {
      std::vector<ScoredTarget> tpm_snapshot;
      const ScoredTarget* active_target = nullptr;

      if (has_reposition_target_)
      {
        active_target = &reposition_target_;
      }
      else if (tpm_)
      {
        tpm_snapshot  = tpm_->getTargets();
        if (!tpm_snapshot.empty()) active_target = &tpm_snapshot[0];
      }

      if (active_target)
      {
        DirectedSampler* sampler =
            (active_target->type == TargetType::E_FREE &&
             active_target->w_normal < params_.w_normal_thresh)
            ? static_cast<DirectedSampler*>(frontier_sampler_.get())
            : static_cast<DirectedSampler*>(hemisphere_sampler_.get());

        auto candidates = sampler->sampleCandidates(
            *active_target, current_state_, ot, params_);
        for (const CandidatePose& cp : candidates)
        {
          if (try_candidate(cp.state)) { sample_found = true; break; }
        }
      }
    }

    // Fallback: original uniform sampling if directed sampler found nothing.
    for (int tries = 0; tries < fast_tries && !sample_found && ros::ok(); ++tries)
    {
      Eigen::Vector4d sample = current_state_ + sampleNewPoint();
      sample[3] = 2.0 * M_PI * (((double)rand()) / ((double)RAND_MAX) - 0.5);
      if (try_candidate(sample))
        sample_found = true;
    }

    if (!sample_found)
    {
      ROS_WARN_STREAM("expandRRT: no feasible sample after "
                      << params_.max_sampling_attempts
                      << " tries (unknown=" << unknown_count
                      << ", collision=" << collision_count
                      << ", oob=" << oob_count
                      << ") at iter " << n << ". Stopping expansion."
                      << " Map may still be mostly UNKNOWN — check sonar TF and insert_pointcloud rate.");
      delete new_node;
      break;
    }

    ROS_DEBUG_STREAM("New node (" << new_node->state_[0] << ", " << new_node->state_[1]
                                  << ", " << new_node->state_[2] << ")");
    // new_node is now ready to be added to tree
    new_node->parent_ = nearest;
    nearest->children_.push_back(new_node);

    // rewire tree with new node
    rewire(kd_tree_, nearest, params_.extension_range, params_.bounding_radius,
           params_.d_overshoot_);

    // Calculate potential information gain for new_node at its sampled yaw
    ROS_DEBUG_STREAM("Get gain");
    std::pair<double, double> ret = getGain(new_node);
    new_node->gain_ = ret.first;
    ROS_DEBUG_STREAM("Insert into KDTREE");
    kd_insert3(kd_tree_, new_node->state_[0], new_node->state_[1], new_node->state_[2],
               new_node);

    // Update best node — only non-UNKNOWN nodes qualify as NBV viewpoints.
    // UNKNOWN nodes remain in the tree for path connectivity but are never
    // selected as the goal pose sent to the robot.
    ROS_DEBUG_STREAM("Update best node");
    if (ot_result.get_state() != la3dm::State::UNKNOWN &&
        (!best_node_ or
         new_node->score(params_.lambda) > best_node_->score(params_.lambda)))
      best_node_ = new_node;

    ROS_DEBUG_STREAM("iteration Done!");
  }

  ROS_DEBUG_STREAM("expandRRT Done!");
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

RRTNode* AEPlanner::chooseParent(RRTNode* node, double l)
{
  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  // Find nearest neighbour
  kdres* nearest = kd_nearest_range3(kd_tree_, node->state_[0], node->state_[1],
                                     node->state_[2], l + 0.5); // FIXME why +0.5?

  if (kd_res_size(nearest) <= 0)
    nearest = kd_nearest3(kd_tree_, node->state_[0], node->state_[1], node->state_[2]);
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

std::pair<double, double> AEPlanner::getGain(RRTNode* node)
{
  node->gain_explicitly_calculated_ = true;
  std::pair<double, double> ret = gainCubature(node->state_);
  ROS_INFO_STREAM("gain expl: " << ret.first);
  return ret;
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
  // Computed once per planning cycle from the robot's current position so it is
  // not repeated for every RRT node candidate.
  priority_cache_.clear();

  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;
  std::shared_lock<std::shared_mutex> lock(ot_mutex_);

  // Use the robot's current base position as the fixed planning origin.
  la3dm::point3f t_plan_p3f((float)current_state_[0],
                             (float)current_state_[1],
                             (float)current_state_[2]);
  Eigen::Vector3d t_plan(current_state_[0], current_state_[1], current_state_[2]);

  priority_cache_.reserve(4096);

  for (auto it = ot->begin_leaf_in_sphere(t_plan_p3f, (float)params_.r_max);
       it != ot->end_leaf(); ++it)
  {
    la3dm::OcTreeNode &node = it.get_node();

    // Skip voxels that have never been observed (no evidence → no info matrix content)
    if (!node.classified) continue;

    // Skip voxels whose info matrix is already deallocated (well-constrained)
    if (!node.has_active_info_matrix()) continue;

    la3dm::point3f p_loc = it.get_loc();
    Eigen::Vector3d p_k(p_loc.x(), p_loc.y(), p_loc.z());

    Eigen::Vector3d diff = p_k - t_plan;
    double dist = diff.norm();
    if (dist < 1e-6) continue;

    la3dm::point3f los_p3f((float)(diff.x()/dist),
                           (float)(diff.y()/dist),
                           (float)(diff.z()/dist));

    // Priority = Beta variance only (λ_min/imbalance term removed per design decision)
    float priority = node.get_var();
    if (priority < 1e-8f) continue;

    // v_weak from info-matrix eigenstruct; used only for alignment in gainCubature()
    float lam1_unused, lam2_unused;
    la3dm::point3f v_weak_p3f;
    node.get_2d_eigenstruct(los_p3f, lam1_unused, lam2_unused, v_weak_p3f);

    priority_cache_.push_back({p_k, priority,
                               Eigen::Vector3d(v_weak_p3f.x(), v_weak_p3f.y(), v_weak_p3f.z())});
  }

  // Keep top-K by priority.
  int K = params_.nbv_k;
  if ((int)priority_cache_.size() > K) {
    std::nth_element(priority_cache_.begin(), priority_cache_.begin() + K,
                     priority_cache_.end(),
                     [](const PriorityVoxel &a, const PriorityVoxel &b) {
                       return a.priority > b.priority;
                     });
    priority_cache_.resize(K);
  }

  priority_cache_valid_ = true;
  ROS_DEBUG_STREAM("computePriorityCache: " << priority_cache_.size()
                   << " priority voxels (r_max=" << params_.r_max << ")");
}

std::pair<double, double> AEPlanner::gainCubature(Eigen::Vector4d state)
{
  // Evaluate information gain at the yaw given by state[3].
  // The cache is built once per planning cycle by computePriorityCache().
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] gainCubature: Waiting for shared_lock (read)");
  std::shared_lock<std::shared_mutex> lock(ot_mutex_);
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] gainCubature: Acquired shared_lock");

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

    // Occlusion check via RayCaster — runs only on FOV-surviving voxels
    {
      std::shared_ptr<la3dm::BGKLOctoMap> ot_local = ot_;
      la3dm::point3f t_p3f((float)t_cand.x(), (float)t_cand.y(), (float)t_cand.z());
      la3dm::point3f pv_p3f((float)pv.pos.x(), (float)pv.pos.y(), (float)pv.pos.z());
      la3dm::BGKLOctoMap::RayCaster rc(ot_local.get(), t_p3f, pv_p3f);
      la3dm::point3f rp; la3dm::OcTreeNode rn;
      la3dm::BlockHashKey rbk; la3dm::OcTreeHashKey rnk;
      bool occluded = false;
      while (!rc.end()) {
        if (rc.next(rp, rn, rbk, rnk)) {
          if (rn.get_state() == la3dm::State::OCCUPIED) {
            float dx = rp.x()-pv_p3f.x(), dy = rp.y()-pv_p3f.y(), dz = rp.z()-pv_p3f.z();
            if (std::sqrt(dx*dx + dy*dy + dz*dz) > (float)params_.resolution) {
              occluded = true; break;
            }
          }
        }
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
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] gainCubature: Releasing shared_lock and returning");
  return std::make_pair(score, state[3]);
}

geometry_msgs::PoseArray AEPlanner::getFrontiers()
{
  geometry_msgs::PoseArray frontiers;

  pigain::BestNode srv;
  srv.request.threshold = 50; // FIXME parameterize
  ROS_INFO_STREAM("Asking pigain for frontiers with threshold: " << srv.request.threshold);
  if (best_node_client_.call(srv))
  {
    ROS_INFO_STREAM("pigain returned " << srv.response.best_node.size() << " frontiers. Best gain was " << srv.response.gain);
    for (int i = 0; i < srv.response.best_node.size(); ++i)
    {
      geometry_msgs::Pose frontier;
      frontier.position = srv.response.best_node[i];
      frontiers.poses.push_back(frontier);
    }
  }
  else
  {
  }

  return frontiers;
}

bool AEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool AEPlanner::collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r)
{
  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] collisionLine: Waiting for shared_lock (read)");
  std::shared_lock<std::shared_mutex> lock(ot_mutex_);
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] collisionLine: Acquired shared_lock");
  ROS_DEBUG_STREAM("In collision");
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);

  float res = ot->get_resolution();
  float min_x = std::min(p1[0], p2[0]) - r;
  float min_y = std::min(p1[1], p2[1]) - r;
  float min_z = std::min(p1[2], p2[2]) - r;
  float max_x = std::max(p1[0], p2[0]) + r;
  float max_y = std::max(p1[1], p2[1]) + r;
  float max_z = std::max(p1[2], p2[2]) + r;
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  for (float x = min_x; x <= max_x; x += res) {
    for (float y = min_y; y <= max_y; y += res) {
      for (float z = min_z; z <= max_z; z += res) {
        la3dm::OcTreeNode node = ot->search(x, y, z);
        if (node.get_state() == la3dm::State::OCCUPIED) {
          octomap::point3d pt(x, y, z);
          if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r) {
            ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] collisionLine: Releasing shared_lock and returning true");
            return true;
          }
        }
      }
    }
  }
  ROS_DEBUG_STREAM("In collision (exiting)");

  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] collisionLine: Releasing shared_lock and returning false");
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

  // Exclusive write lock: blocks all readers (gainCubature, collisionLine,
  // publishMapViz) while BGK inference modifies block_arr.
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] cloudCallback: Waiting for unique_lock (write)");
  std::unique_lock<std::shared_mutex> lock(ot_mutex_);
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] cloudCallback: Acquired unique_lock, inserting pointcloud");
  ot_->insert_pointcloud(*pcl_cloud, origin, sensor_up, params_.ds_resolution, params_.free_resolution, params_.r_max);
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] cloudCallback: pointcloud inserted, lock released");
}

void AEPlanner::publishMapViz(const ros::TimerEvent&)
{
  if (!ot_)
    return;

  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] publishMapViz: Waiting for shared_lock");
  std::shared_lock<std::shared_mutex> lock(ot_mutex_);
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] publishMapViz: Acquired shared_lock");

  m_pub_occ_->clear();
  m_pub_free_->clear();
  // m_pub_free_txt_->clear();
  m_pub_unc_->clear();
  // m_pub_unk_->clear();
  m_pub_var_->clear();

  const float max_var_vis = 0.25f;
  la3dm::point3f viz_center((float)current_state_[0], (float)current_state_[1], (float)current_state_[2]);
  auto leaf_begin = (params_.max_vis_radius > 0.0 and current_state_initialized_)
                        ? ot_->begin_leaf_in_sphere(viz_center, (float)params_.max_vis_radius)
                        : ot_->begin_leaf();

  for (auto it = leaf_begin; it != ot_->end_leaf(); ++it) {
    la3dm::point3f p = it.get_loc();
    auto state = it.get_node().get_state();

    if (state == la3dm::State::OCCUPIED) {
      m_pub_occ_->insert_point3d(p.x(), p.y(), p.z(), params_.min_z, params_.max_z, it.get_size());
    } else if (state == la3dm::State::UNCERTAIN) {
      m_pub_unc_->insert_point3d_color(p.x(), p.y(), p.z(), it.get_size(), 1.0f, 1.0f, 0.0f);
    } else if (state == la3dm::State::FREE) {
      m_pub_free_->insert_point3d(p.x(), p.y(), p.z(), params_.min_z, params_.max_z, it.get_size());
    }

    if (state == la3dm::State::OCCUPIED || state == la3dm::State::UNCERTAIN) {
      std::string ns = (state == la3dm::State::OCCUPIED) ? "occupied" : "uncertain";
      m_pub_var_->insert_color_point3d(p.x(), p.y(), p.z(), 0.0, max_var_vis, it.get_node().get_var(), it.get_size(), ns);
    }
  }

  m_pub_occ_->publish();
  m_pub_free_->publish();
  m_pub_unc_->publish();
  // m_pub_unk_->publish();
  m_pub_var_->publish();
  ROS_DEBUG_STREAM("[DEBUG_AEPLANNER] publishMapViz: Releasing shared_lock");
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

void AEPlanner::publishEvaluatedNodesRecursive(RRTNode* node)
{
  if (!node)
    return;
  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
  {
    if ((*node_it)->gain_explicitly_calculated_)
    {
      pigain::Node pig_node;
      pig_node.gain = (*node_it)->gain_;
      pig_node.position.x = (*node_it)->state_[0];
      pig_node.position.y = (*node_it)->state_[1];
      pig_node.position.z = (*node_it)->state_[2];
      pig_node.yaw = (*node_it)->state_[3];
      gain_pub_.publish(pig_node);
    }

    publishEvaluatedNodesRecursive(*node_it);
  }
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
