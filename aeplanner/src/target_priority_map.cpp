#include <aeplanner/target_priority_map.h>
#include <aeplanner/Target.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>

namespace aeplanner
{

TargetPriorityMap::TargetPriorityMap(
    ros::NodeHandle& nh,
    const std::shared_ptr<la3dm::BGKLOctoMap>& map,
    std::shared_mutex& map_mutex,
    const TPMParams& params)
  : map_(map)
  , map_mutex_(map_mutex)
  , params_(params)
  , blacklist_(params.n_fail, params.t_cooldown)
  , robot_pos_(Eigen::Vector3d::Zero())
{
  targets_pub_  = nh.advertise<aeplanner::TargetList>("/tpm/targets", 1, true);
  viz_pub_      = nh.advertise<visualization_msgs::MarkerArray>("/tpm/targets_viz", 1, true);
  clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/tpm/clusters_viz", 1, true);
}

void TargetPriorityMap::update(const ros::TimerEvent&)
{
  if (!map_) return;

  Eigen::Vector3d robot_pos;
  {
    std::lock_guard<std::mutex> lk(robot_pos_mutex_);
    robot_pos = robot_pos_;
  }

  // Classify voxels (shared read lock on map)
  std::vector<ClassifiedVoxel> all_voxels;
  {
    std::shared_lock<std::shared_mutex> map_lk(map_mutex_);
    all_voxels = classifyVoxels(map_, robot_pos,
                                params_.r_max, params_.sigma2_thresh);
  }

  // Split into U-targets and frontier voxels
  std::vector<ClassifiedVoxel> u_voxels, frontier_voxels;
  for (const ClassifiedVoxel& v : all_voxels)
  {
    if (v.type == TargetType::U_TARGET)
      u_voxels.push_back(v);
    else
      frontier_voxels.push_back(v);
  }

  // Cluster U_TARGETs (surface-filtered by classifier) so isolated uncertain
  // voxels are ignored and spatially connected groups form one target each
  std::vector<FrontierCluster> u_clusters = detectFrontierClusters(u_voxels, params_.R_cluster);
  for (FrontierCluster& uc : u_clusters)
    uc.type = TargetType::U_TARGET;

  // Cluster E_OCC and E_FREE frontiers separately so they are never mixed
  std::vector<ClassifiedVoxel> e_occ_voxels, e_free_voxels;
  e_occ_voxels.reserve(frontier_voxels.size());
  e_free_voxels.reserve(frontier_voxels.size());
  for (const ClassifiedVoxel& v : frontier_voxels)
  {
    if (v.type == TargetType::E_OCC) e_occ_voxels.push_back(v);
    else                             e_free_voxels.push_back(v);
  }

  std::vector<FrontierCluster> occ_clusters  = detectFrontierClusters(e_occ_voxels,  params_.R_cluster);
  std::vector<FrontierCluster> free_clusters = detectFrontierClusters(e_free_voxels, params_.R_cluster);

  // Offset free-cluster member_indices so they index into the combined voxel vector
  const size_t occ_offset = e_occ_voxels.size();
  for (FrontierCluster& c : free_clusters)
    for (size_t& idx : c.member_indices)
      idx += occ_offset;

  // Combined voxel vector for viz lookup and merged cluster list for scoring
  std::vector<ClassifiedVoxel> all_frontier_voxels = e_occ_voxels;
  all_frontier_voxels.insert(all_frontier_voxels.end(),
                              e_free_voxels.begin(), e_free_voxels.end());

  std::vector<FrontierCluster> clusters;
  clusters.reserve(occ_clusters.size() + free_clusters.size());
  clusters.insert(clusters.end(), occ_clusters.begin(),  occ_clusters.end());
  clusters.insert(clusters.end(), free_clusters.begin(), free_clusters.end());

  // Score and sort
  ScorerParams sp;
  sp.w_frontier   = params_.w_frontier;
  sp.cluster_norm = params_.cluster_norm;

  std::vector<ScoredTarget> scored = scoreTargets(u_clusters, u_voxels, clusters, sp);

  // Apply blacklist filter
  std::vector<ScoredTarget> filtered;
  filtered.reserve(scored.size());
  for (ScoredTarget& t : scored)
  {
    if (blacklist_.isBlacklisted(t.pos, params_.resolution)) continue;
    t.fail_count = blacklist_.getFailCount(t.pos, params_.resolution);
    filtered.push_back(t);
  }

  // Suppress E_FREE targets whenever any E_OCC target is available
  bool has_occ_frontier = false;
  for (const ScoredTarget& t : filtered)
    if (t.type == TargetType::E_OCC) { has_occ_frontier = true; break; }
  if (has_occ_frontier)
    filtered.erase(std::remove_if(filtered.begin(), filtered.end(),
        [](const ScoredTarget& t){ return t.type == TargetType::E_FREE; }),
        filtered.end());

  // Trim to nbv_k
  if ((int)filtered.size() > params_.nbv_k)
    filtered.resize(params_.nbv_k);

  // Store and publish
  {
    std::lock_guard<std::mutex> lk(targets_mutex_);
    targets_ = filtered;
  }

  aeplanner::TargetList msg = toROSMsg(filtered);
  msg.header.stamp    = ros::Time::now();
  msg.header.frame_id = params_.world_frame;
  targets_pub_.publish(msg);

  publishViz(filtered);
  publishClustersViz(clusters, all_frontier_voxels);
  blacklist_.pruneExpired();
}

void TargetPriorityMap::recordSuccess(const Eigen::Vector3d& pos)
{
  blacklist_.recordSuccess(pos, params_.resolution);
}

void TargetPriorityMap::recordFailure(const Eigen::Vector3d& pos)
{
  blacklist_.recordFailure(pos, params_.resolution);
}

std::vector<ScoredTarget> TargetPriorityMap::getTargets() const
{
  std::lock_guard<std::mutex> lk(targets_mutex_);
  return targets_;
}

void TargetPriorityMap::setRobotPos(const Eigen::Vector3d& pos)
{
  std::lock_guard<std::mutex> lk(robot_pos_mutex_);
  robot_pos_ = pos;
}

aeplanner::TargetList TargetPriorityMap::toROSMsg(
    const std::vector<ScoredTarget>& targets) const
{
  aeplanner::TargetList msg;
  msg.targets.reserve(targets.size());
  for (const ScoredTarget& t : targets)
  {
    aeplanner::Target tm;
    tm.position.x  = t.pos.x();
    tm.position.y  = t.pos.y();
    tm.position.z  = t.pos.z();
    tm.type        = static_cast<uint8_t>(t.type);
    tm.priority    = t.priority;
    tm.normal.x    = t.normal.x();
    tm.normal.y    = t.normal.y();
    tm.normal.z    = t.normal.z();
    tm.w_normal    = t.w_normal;
    tm.fail_count  = t.fail_count;
    msg.targets.push_back(tm);
  }
  return msg;
}

void TargetPriorityMap::publishViz(const std::vector<ScoredTarget>& targets) const
{
  if (viz_pub_.getNumSubscribers() == 0) return;

  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker del;
  del.action = visualization_msgs::Marker::DELETEALL;
  del.ns = "tpm";
  ma.markers.push_back(del);

  int id = 0;
  for (const ScoredTarget& t : targets)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = params_.world_frame;
    m.header.stamp    = ros::Time::now();
    m.ns              = "tpm";
    m.id              = id++;
    m.type            = visualization_msgs::Marker::SPHERE;
    m.action          = visualization_msgs::Marker::ADD;
    m.pose.position.x = t.pos.x();
    m.pose.position.y = t.pos.y();
    m.pose.position.z = t.pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    m.lifetime = ros::Duration(0);

    // Colour: U-target=red, E_OCC=orange, E_FREE=blue
    if (t.type == TargetType::U_TARGET)
    { m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.8f; }
    else if (t.type == TargetType::E_OCC)
    { m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.8f; }
    else
    { m.color.r = 0.0f; m.color.g = 0.3f; m.color.b = 1.0f; m.color.a = 0.8f; }

    ma.markers.push_back(m);
  }
  viz_pub_.publish(ma);
}

void TargetPriorityMap::publishClustersViz(
    const std::vector<FrontierCluster>& clusters,
    const std::vector<ClassifiedVoxel>& frontier_voxels) const
{
  if (clusters_pub_.getNumSubscribers() == 0) return;

  // Simple HSV→RGB for per-cluster colour cycling.
  auto hsvToRgb = [](float h, float& r, float& g, float& b) {
    h = std::fmod(h, 360.0f);
    float c = 0.9f, x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    if      (h < 60)  { r=c; g=x; b=0; }
    else if (h < 120) { r=x; g=c; b=0; }
    else if (h < 180) { r=0; g=c; b=x; }
    else if (h < 240) { r=0; g=x; b=c; }
    else if (h < 300) { r=x; g=0; b=c; }
    else              { r=c; g=0; b=x; }
    // add brightness offset so colours are never black
    r += 0.1f; g += 0.1f; b += 0.1f;
  };

  visualization_msgs::MarkerArray ma;

  // Delete all markers in both namespaces
  for (const char* ns : {"tpm_clusters_occ", "tpm_clusters_free"})
  {
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    del.ns = ns;
    ma.markers.push_back(del);
  }

  // Per-namespace id counters so ids are unique within each namespace
  int id_occ = 0, id_free = 0;

  for (size_t ci = 0; ci < clusters.size(); ++ci)
  {
    const FrontierCluster& c = clusters[ci];
    const bool is_occ = (c.type == TargetType::E_OCC);
    const std::string ns = is_occ ? "tpm_clusters_occ" : "tpm_clusters_free";
    int& id = is_occ ? id_occ : id_free;

    // Assign a distinct hue per cluster via golden-angle increment.
    float hue = std::fmod((float)ci * 137.508f, 360.0f);
    float cr, cg, cb;
    hsvToRgb(hue, cr, cg, cb);

    // --- Voxel cubes ---
    visualization_msgs::Marker cubes;
    cubes.header.frame_id    = params_.world_frame;
    cubes.header.stamp       = ros::Time::now();
    cubes.ns                 = ns;
    cubes.id                 = id++;
    cubes.type               = visualization_msgs::Marker::CUBE_LIST;
    cubes.action             = visualization_msgs::Marker::ADD;
    cubes.scale.x = cubes.scale.y = cubes.scale.z = 0.25;
    cubes.color.r = cr; cubes.color.g = cg; cubes.color.b = cb; cubes.color.a = 0.5f;
    cubes.lifetime           = ros::Duration(0);
    cubes.pose.orientation.w = 1.0;

    for (size_t idx : c.member_indices)
    {
      geometry_msgs::Point p;
      p.x = frontier_voxels[idx].pos.x();
      p.y = frontier_voxels[idx].pos.y();
      p.z = frontier_voxels[idx].pos.z();
      cubes.points.push_back(p);
    }
    ma.markers.push_back(cubes);

    // --- Normal arrow (only when reliable) ---
    if (c.w_normal >= 0.1f)
    {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id    = params_.world_frame;
      arrow.header.stamp       = ros::Time::now();
      arrow.ns                 = ns;
      arrow.id                 = id++;
      arrow.type               = visualization_msgs::Marker::ARROW;
      arrow.action             = visualization_msgs::Marker::ADD;
      arrow.scale.x            = 0.05;   // shaft diameter
      arrow.scale.y            = 0.10;   // head diameter
      arrow.scale.z            = 0.15;   // head length
      arrow.color.r = cr; arrow.color.g = cg; arrow.color.b = cb; arrow.color.a = 1.0f;
      arrow.lifetime           = ros::Duration(0);

      geometry_msgs::Point p0, p1;
      p0.x = c.centroid.x(); p0.y = c.centroid.y(); p0.z = c.centroid.z();
      double len = c.w_normal * 1.5;
      p1.x = c.centroid.x() + c.normal.x() * len;
      p1.y = c.centroid.y() + c.normal.y() * len;
      p1.z = c.centroid.z() + c.normal.z() * len;
      arrow.points.push_back(p0);
      arrow.points.push_back(p1);
      ma.markers.push_back(arrow);
    }
  }

  clusters_pub_.publish(ma);
}

}  // namespace aeplanner
