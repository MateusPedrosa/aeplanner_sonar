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
  , gp_query_client_(nh_.serviceClient<pigain::Query>("gp_query_server"))
  , reevaluate_server_(nh_.advertiseService("reevaluate", &AEPlanner::reevaluate, this))
  , best_node_client_(nh_.serviceClient<pigain::BestNode>("best_node_server"))
  , current_state_initialized_(false)
  , ot_(NULL)
  , best_node_(NULL)
  , best_branch_root_(NULL)
{
  params_ = readParams();
  as_.start();

  ot_ = std::make_shared<la3dm::BGKLOctoMap>(params_.resolution, params_.block_depth, params_.sf2, params_.ell, params_.free_thresh, params_.occupied_thresh, params_.var_thresh, params_.prior_A, params_.prior_B, params_.theta_bw, params_.phi_bw);

  m_pub_occ_ = new la3dm::MarkerArrayPub(nh_, "/occupied_cells_vis_array", params_.resolution);
  m_pub_free_ = new la3dm::MarkerArrayPub(nh_, "/free_cells_vis_array", params_.resolution);
  m_pub_free_txt_ = new la3dm::TextMarkerArrayPub(nh_, "/free_cells_txt_vis_array", params_.resolution);
  m_pub_unc_ = new la3dm::MarkerArrayPub(nh_, "/uncertain_cells_vis_array", params_.resolution);
  m_pub_unk_ = new la3dm::MarkerArrayPub(nh_, "/unknown_cells_vis_array", params_.resolution);
  m_pub_var_ = new la3dm::MarkerArrayPub(nh_, "/variance_vis_array", params_.resolution);
  // Initialize kd-tree
  kd_tree_ = kd_create(3);
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

  ROS_DEBUG("Init");
  RRTNode* root = initialize();
  ROS_DEBUG("expandRRT");
  if (root->gain_ > 0.25 or !root->children_.size() or // FIXME parameterize
      root->score(params_.lambda) < params_.zero_gain)
    expandRRT();
  else
    best_node_ = root->children_[0];

  ROS_DEBUG("getCopyOfParent");
  best_branch_root_ = best_node_->getCopyOfParentBranch();

  ROS_DEBUG("createRRTMarker");
  rrt_marker_pub_.publish(createRRTMarkerArray(root, params_.lambda));
  
  // Publish bounding box
  bbx_marker_pub_.publish(createBoundingBoxMarker(params_.boundary_min, params_.boundary_max, 0, params_.world_frame));

  ROS_DEBUG("publishRecursive");
  publishEvaluatedNodesRecursive(root);

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

  // Expand an RRT tree and calculate information gain in every node
  ROS_DEBUG_STREAM("Entering expanding RRT");
  for (int n = 0; (n < params_.init_iterations or
                   (n < params_.cutoff_iterations and
                    best_node_->score(params_.lambda) < params_.zero_gain)) and
                  ros::ok();
       ++n)
  {
    ROS_DEBUG_STREAM("In expand RRT iteration: " << n);
    RRTNode* new_node = new RRTNode();
    RRTNode* nearest;
    la3dm::OcTreeNode ot_result;

    // Sample new point around agent and check that
    // (1) it is within the boundaries
    // (2) it is in known space
    // (3) the path between the new node and it's parent does not contain any
    // obstacles

    do
    {
      Eigen::Vector4d offset = sampleNewPoint();
      new_node->state_ = current_state_ + offset;

      nearest = chooseParent(new_node, params_.extension_range);

      new_node->state_ = restrictDistance(nearest->state_, new_node->state_);

      ROS_DEBUG_STREAM("Trying node (" << new_node->state_[0] << ", "
                                       << new_node->state_[1] << ", "
                                       << new_node->state_[2] << ")");
      ROS_DEBUG_STREAM("    nearest (" << nearest->state_[0] << ", " << nearest->state_[1]
                                       << ", " << nearest->state_[2] << ")");
      ot_result = ot->search(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
      if (ot_result.get_state() == la3dm::State::UNKNOWN)
        continue;
      ROS_DEBUG_STREAM("ot check done!");

      ROS_DEBUG_STREAM("Inside boundaries?  " << isInsideBoundaries(new_node->state_));
      ROS_DEBUG_STREAM("In known space?     " << ot_result);
      ROS_DEBUG_STREAM("Collision?          " << collisionLine(
                           nearest->state_, new_node->state_, params_.bounding_radius));
    } while (!isInsideBoundaries(new_node->state_) or ot_result.get_state() == la3dm::State::UNKNOWN or
             collisionLine(nearest->state_, new_node->state_, params_.bounding_radius));

    ROS_DEBUG_STREAM("New node (" << new_node->state_[0] << ", " << new_node->state_[1]
                                  << ", " << new_node->state_[2] << ")");
    // new_node is now ready to be added to tree
    new_node->parent_ = nearest;
    nearest->children_.push_back(new_node);

    // rewire tree with new node
    rewire(kd_tree_, nearest, params_.extension_range, params_.bounding_radius,
           params_.d_overshoot_);

    // Calculate potential information gain for new_node
    ROS_DEBUG_STREAM("Get gain");
    std::pair<double, double> ret = getGain(new_node);
    new_node->state_[3] = ret.second;  // Assign yaw angle that maximizes g
    new_node->gain_ = ret.first;
    ROS_DEBUG_STREAM("Insert into KDTREE");
    kd_insert3(kd_tree_, new_node->state_[0], new_node->state_[1], new_node->state_[2],
               new_node);

    // Update best node

    ROS_DEBUG_STREAM("Update best node");
    if (!best_node_ or
        new_node->score(params_.lambda) > best_node_->score(params_.lambda))
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
  pigain::Query srv;
  srv.request.point.x = node->state_[0];
  srv.request.point.y = node->state_[1];
  srv.request.point.z = node->state_[2];

  if (gp_query_client_.call(srv))
  {
    if (srv.response.sigma < params_.sigma_thresh)
    {
      double gain = srv.response.mu;
      double yaw = srv.response.yaw;

      ROS_INFO_STREAM("gain impl: " << gain);
      return std::make_pair(gain, yaw);
    }
  }

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

std::pair<double, double> AEPlanner::gainCubature(Eigen::Vector4d state)
{
  std::shared_ptr<la3dm::BGKLOctoMap> ot = ot_;

  // This function computes the gain
  double hfov = params_.hfov, vfov = params_.vfov;

  double dr = params_.resolution, dphi = params_.dphi, dtheta = params_.dtheta;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int body_yaw, phi, theta;
  double body_yaw_rad, theta_rad, phi_rad;

  std::map<int, double> gain_per_yaw;

  // Proposed state of the robot (base_link) in the world frame:
    tf::Vector3 base_origin(state[0], state[1], state[2]);

  for (body_yaw = -180; body_yaw < 180; body_yaw += dtheta)
  {
    body_yaw_rad = body_yaw * M_PI/ 180.0f;

    tf::Quaternion base_quat;
    base_quat.setEuler(0.0, 0.0, body_yaw_rad); // (Roll, Pitch, Yaw)
    tf::Pose base_pose_in_world(base_quat, base_origin);

    // Transform from base_link to sensor_frame
    tf::StampedTransform base_to_sensor;
    try
    {
      // Static transform between the robot frame and the sensor frame
      tf_listener_.lookupTransform(params_.robot_frame, params_.sensor_frame, ros::Time(0), base_to_sensor);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "AEPlanner: %s. Assuming sensor is at base_link.", ex.what());
      base_to_sensor.setIdentity();
    }

    // Sensor's pose in the world frame
    tf::Pose sensor_pose_in_world = base_pose_in_world * base_to_sensor;
    
    Eigen::Vector3d sensor_origin(sensor_pose_in_world.getOrigin().x(),
                                  sensor_pose_in_world.getOrigin().y(),
                                  sensor_pose_in_world.getOrigin().z());

    // 3x3 rotation matrix for the sensor to transform the rays
    tf::Matrix3x3 sensor_rot = sensor_pose_in_world.getBasis();

    double g = 0;
    for (theta = -hfov/2; theta < hfov/2; theta += dtheta)
    {
      theta_rad = theta * M_PI / 180.0f;
      for (phi = 90 - vfov / 2; phi < 90 + vfov / 2; phi += dphi)
      {
        phi_rad = phi * M_PI / 180.0f;

        for (r = params_.r_min; r < params_.r_max; r += dr)
        {
          // Calculate the ray vector in the sensor's local frame
          // Standard spherical to cartesian where X is forward
          tf::Vector3 ray_local(
              r * sin(phi_rad) * cos(theta_rad),
              r * sin(phi_rad) * sin(theta_rad),
              r * cos(phi_rad)
          );

          // Rotate the ray to the world frame using the sensor's rotation
          tf::Vector3 ray_world = sensor_rot * ray_local;

          // The point to query in the world is the sensor's origin + the rotated ray
          Eigen::Vector3d vec(
              sensor_origin.x() + ray_world.x(),
              sensor_origin.y() + ray_world.y(),
              sensor_origin.z() + ray_world.z()
          );

          la3dm::OcTreeNode result = ot->search(vec[0], vec[1], vec[2]);

          Eigen::Vector4d v(vec[0], vec[1], vec[2], 0);
          if (!isInsideBoundaries(v))
            break;
          // Break if occupied so we don't count any information gain behind a wall.
          if (result.get_state() == la3dm::State::OCCUPIED)
            break; // No gain from cells behind an occupied cell
          else if (result.get_state() == la3dm::State::UNCERTAIN)
            {
              if (r < params_.uncertain_threshold)
            g += 2 * (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
break; // No gain from cells behind an uncertain cell
            }
          else if (result.get_state() == la3dm::State::UNKNOWN)
            g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
}
        }
      }
    }
    gain_per_yaw[body_yaw] = g;
  }

  // Debug: log all sampled body yaws and their scores
  ROS_DEBUG_STREAM("gainCubature at (" << state[0] << ", " << state[1] << ", " << state[2] << "):");
  for (const auto& kv : gain_per_yaw)
      ROS_DEBUG_STREAM("  yaw " << std::setw(5) << kv.first << " deg  score: " << kv.second);

  auto best_it = std::max_element(gain_per_yaw.begin(), gain_per_yaw.end(),
    [](const std::pair<int,double>& a, const std::pair<int,double>& b) {
        return a.second < b.second;
    });

  int best_yaw_deg = best_it->first;
  double best_yaw_score = best_it->second;

  ROS_DEBUG_STREAM("  --> best_yaw: " << best_yaw_deg << " deg  score: " << best_yaw_score);

  // double r_max = params_.r_max;
  // double h_max = params_.hfov / M_PI * 180;
  // double v_max = params_.vfov / M_PI * 180;

  // gain = best_yaw_score / ((r_max*r_max*r_max/3) * h_max * (1-cos(v_max))) ;

  double yaw = best_yaw_deg * M_PI / 180.f;

  state[3] = yaw;
  return std::make_pair(best_yaw_score, yaw);
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
            return true;
          }
        }
      }
    }
  }
  ROS_DEBUG_STREAM("In collision (exiting)");

  return false;
}

void AEPlanner::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  tf::StampedTransform transform;
  try {
      // tf_listener_.waitForTransform(params_.world_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(5.0));
      tf_listener_.lookupTransform(params_.world_frame, msg->header.frame_id, msg->header.stamp, transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
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

  sensor_msgs::PointCloud2 cloud_map;
  pcl_ros::transformPointCloud(params_.world_frame, *msg, cloud_map, tf_listener_);

  la3dm::PCLPointCloud::Ptr pcl_cloud (new la3dm::PCLPointCloud());
  pcl::fromROSMsg(cloud_map, *pcl_cloud);

  ot_->insert_pointcloud(*pcl_cloud, origin, sensor_up, params_.ds_resolution, params_.free_resolution, params_.r_max);

  m_pub_occ_->clear();
  m_pub_free_->clear();
  m_pub_free_txt_->clear();
  m_pub_unc_->clear();
  m_pub_unk_->clear();
  m_pub_var_->clear();

  float max_var_vis = 0.5f;
  for (auto it = ot_->begin_leaf(); it != ot_->end_leaf(); ++it) {
      la3dm::point3f p = it.get_loc();
      
      if (it.get_node().get_state() == la3dm::State::OCCUPIED) {
          m_pub_occ_->insert_point3d(p.x(), p.y(), p.z(), -10.0, 10.0, it.get_size());
      } else if (it.get_node().get_state() == la3dm::State::FREE) {
          m_pub_free_->insert_point3d(p.x(), p.y(), p.z(), -10.0, 10.0, it.get_size());
      } else if (it.get_node().get_state() == la3dm::State::UNCERTAIN) {
          m_pub_unc_->insert_point3d(p.x(), p.y(), p.z(), -10.0, 10.0, it.get_size());
      } else if (it.get_node().get_state() == la3dm::State::UNKNOWN) {
          m_pub_unk_->insert_point3d(p.x(), p.y(), p.z(), -10.0, 10.0, it.get_size());
      }
      
      auto state = it.get_node().get_state();
      if (state == la3dm::State::OCCUPIED || state == la3dm::State::FREE || state == la3dm::State::UNCERTAIN) {
          std::string ns = (state == la3dm::State::OCCUPIED) ? "occupied" : (state == la3dm::State::FREE) ? "free" : "uncertain";
          m_pub_var_->insert_color_point3d(p.x(), p.y(), p.z(), 0.0, max_var_vis, it.get_node().get_var(), it.get_size(), ns);
      }
  }

  m_pub_occ_->publish();
  m_pub_free_->publish();
  m_pub_unc_->publish();
  m_pub_unk_->publish();
  m_pub_var_->publish();
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

void AEPlanner::agentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  current_state_[2] = msg.pose.position.z;
  current_state_[3] = tf2::getYaw(msg.pose.orientation);

  current_state_initialized_ = true;
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
