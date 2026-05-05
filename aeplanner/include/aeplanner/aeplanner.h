#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <ros/ros.h>

#include <mutex>
#include <shared_mutex>
#include <unordered_set>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <bgkloctomap/bgkloctomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/markerarray_pub.h>

#include <eigen3/Eigen/Dense>

#include <kdtree/kdtree.h>

#include <aeplanner/data_structures.h>
#include <aeplanner/param.h>
#include <aeplanner/Reevaluate.h>

#include <aeplanner/aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <aeplanner/aeplannerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>

#include <aeplanner/target_priority_map.h>
#include <aeplanner/directed_sampler.h>
#include <aeplanner/hemisphere_sampler.h>
#include <aeplanner/frontier_sampler.h>
#include <aeplanner/planner_state_machine.h>
#include <aeplanner/dwell_controller.h>
#include <std_msgs/String.h>

namespace aeplanner
{

// Snapshot of occupied voxels built once per execute() tick under a single
// short shared_lock. All collision and gain computation during that tick uses
// this copy, keeping ot_mutex_ free for cloudCallback inserts throughout the
// entire planning loop.
struct OccupiedSnapshot
{
  std::unordered_set<int64_t> keys;
  float resolution = 0.1f;

  // 21 bits per axis, offset so negative coordinates map cleanly.
  // Supports ±1,048,576 voxels per axis = ±104 km at 0.1 m resolution.
  static constexpr int32_t COORD_OFFSET = 1 << 20;

  static int64_t packKey(int32_t ix, int32_t iy, int32_t iz)
  {
    uint32_t ux = static_cast<uint32_t>(ix + COORD_OFFSET) & 0x1FFFFFu;
    uint32_t uy = static_cast<uint32_t>(iy + COORD_OFFSET) & 0x1FFFFFu;
    uint32_t uz = static_cast<uint32_t>(iz + COORD_OFFSET) & 0x1FFFFFu;
    return (static_cast<int64_t>(ux) << 42) |
           (static_cast<int64_t>(uy) << 21) |
            static_cast<int64_t>(uz);
  }

  void insert(float x, float y, float z)
  {
    keys.insert(packKey(static_cast<int32_t>(std::floor(x / resolution)),
                        static_cast<int32_t>(std::floor(y / resolution)),
                        static_cast<int32_t>(std::floor(z / resolution))));
  }

  bool occupied(float x, float y, float z) const
  {
    return keys.count(packKey(static_cast<int32_t>(std::floor(x / resolution)),
                               static_cast<int32_t>(std::floor(y / resolution)),
                               static_cast<int32_t>(std::floor(z / resolution)))) > 0;
  }

  bool empty() const { return keys.empty(); }
};

class AEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;

  std::shared_ptr<la3dm::BGKLOctoMap> ot_;
  // Reader-writer lock for ot_:
  //   cloudCallback (insert_pointcloud)  → unique_lock (exclusive write)
  //   computePriorityCache               → shared_lock, held for one leaf iteration
  //   publishMapViz                      → try shared_lock, skip if writer active
  //   collisionLine / gainCubature       → NO lock (use planning_snapshot_ instead)
  // cloudCallback's unique_lock is only blocked by the brief cache/viz iterations;
  // all planning computation runs lock-free against planning_snapshot_.
  mutable std::shared_mutex ot_mutex_;

  la3dm::MarkerArrayPub *m_pub_occ_, *m_pub_free_, *m_pub_unc_, *m_pub_unk_, *m_pub_var_;
  la3dm::TextMarkerArrayPub *m_pub_free_txt_;

  // Timer that drives voxel visualization independently of the sonar
  // callback, so a growing map does not starve cloudCallback.
  ros::Timer viz_timer_;

  // Target Priority Map — runs as a ros::Timer callback inside this process.
  std::unique_ptr<TargetPriorityMap> tpm_;
  ros::Timer                         tpm_timer_;
  ros::Publisher                     state_pub_;  // /viewplanner/state

  // Directed samplers (Phase 2)
  std::unique_ptr<HemisphereSampler> hemisphere_sampler_;
  std::unique_ptr<FrontierSampler>   frontier_sampler_;

  // State machine and DWELL controller (Phase 3)
  std::unique_ptr<PlannerStateMachine> state_machine_;
  std::unique_ptr<DwellController>     dwell_controller_;

  // Committed viewpoint for RESOLVE state — selected once per episode,
  // then followed step-by-step until arrival triggers DWELL.
  bool            has_committed_viewpoint_ = false;
  Eigen::Vector4d committed_viewpoint_;
  double          committed_viewpoint_roll_ = 0.0;  // robot roll commanded on final approach
  ScoredTarget    committed_target_;
  PlannerState    prev_planner_state_ = PlannerState::EXPLORE;

  // RRT*-planned waypoint sequence to the committed viewpoint.
  // Re-planned whenever empty (new commitment or path blocked).
  std::vector<Eigen::Vector4d> resolve_waypoints_;
  int                          resolve_wp_idx_ = 0;

  // Set at DWELL exit / EXPLORE arrival; consumed at the next selection phase
  // to trigger a synchronous TPM update before picking the next target.
  bool tpm_update_needed_ = false;

  // Committed viewpoint for EXPLORE state — mirrors RESOLVE variables.
  // No DWELL on arrival; re-evaluates immediately next tick.
  bool                         has_committed_explore_viewpoint_ = false;
  Eigen::Vector4d              committed_explore_viewpoint_;
  ScoredTarget                 committed_explore_target_;
  std::vector<Eigen::Vector4d> explore_waypoints_;
  int                          explore_wp_idx_       = 0;
  int                          explore_hold_count_   = 0;
  static constexpr int         kMaxExploreHoldTicks  = 10;

  // Subscribers
  ros::Subscriber point_sub_;
  ros::Subscriber agent_pose_sub_;

  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher bbx_marker_pub_;
  ros::Publisher viewpoints_pub_;

  // Accumulated sonar viewpoint history published after every map update
  geometry_msgs::PoseArray viewpoints_msg_;

  // TF
  tf::TransformListener tf_listener_;

  // Cached sensor-to-robot transform at servo rest position.
  // Populated on first successful lookupTransform; reused by all gainCubature / gainExploration calls.
  tf::StampedTransform sensor_transform_rest_;
  bool                 sensor_transform_rest_valid_ = false;

  // Services
  ros::ServiceServer reevaluate_server_;

  // Service server callback
  bool reevaluate(aeplanner::Reevaluate::Request& req,
                  aeplanner::Reevaluate::Response& res);

  Eigen::Vector4d sampleNewPoint();
  bool isInsideBoundaries(Eigen::Vector4d point);
  bool collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r);
  RRTNode* chooseParent(RRTNode* node, double l, kdtree* tree);
  void rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r, double r_os);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::pair<double, double> gainCubature(Eigen::Vector4d state, double robot_roll = 0.0);
  double gainExploration(const Eigen::Vector4d& state);

  std::vector<Eigen::Vector4d> planPathToGoal(const Eigen::Vector4d& goal);

  // Occupied-voxel snapshot used by collisionLine and gainCubature.
  // Written exclusively by execute() (single-threaded action-server callback);
  // no mutex needed for planning_snapshot_ itself.
  OccupiedSnapshot planning_snapshot_;

  // cloudCallback appends scan points here after each map commit.
  // execute() drains the queue into planning_snapshot_ at the start of each
  // tick. This keeps ot_mutex_ free throughout the entire planning loop.
  std::vector<Eigen::Vector3f> pending_occupied_points_;
  std::mutex                   pending_mutex_;

  // Priority voxel cache — computed once per planning cycle so that the
  // expensive map iteration is not repeated per candidate viewpoint.
  struct PriorityVoxel {
    Eigen::Vector3d pos;
    float           priority;
    Eigen::Vector3d v_weak;  // world-frame unit weak arc direction
  };
  std::vector<PriorityVoxel> priority_cache_;
  bool             priority_cache_valid_    = false;
  Eigen::Vector3d  priority_cache_built_pos_ = Eigen::Vector3d::Zero();
  void computePriorityCache();

  // Incremental candidate index (Option B).
  // candidate_index_ maps quantised voxel position → {priority, raw info[6]}.
  // It is bootstrapped once with a full map scan, then kept current by applying
  // dirty entries produced by commit_pointcloud_update() on every sonar callback.
  struct VoxelKey {
    int32_t ix, iy, iz;
    bool operator==(const VoxelKey& o) const noexcept {
      return ix == o.ix && iy == o.iy && iz == o.iz;
    }
  };
  struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const noexcept {
      uint32_t h = static_cast<uint32_t>(k.ix) * 2654435761u
                 ^ static_cast<uint32_t>(k.iy) * 2246822519u
                 ^ static_cast<uint32_t>(k.iz) * 3266489917u;
      return static_cast<size_t>(h);
    }
  };
  struct CandidateEntry { float priority; float info[6]; };
  std::unordered_map<VoxelKey, CandidateEntry, VoxelKeyHash> candidate_index_;
  bool index_bootstrapped_ = false;

  // Dirty entries queued by cloudCallback, consumed by computePriorityCache().
  std::vector<la3dm::BGKLOctoMap::DirtyEntry> dirty_buffer_;
  std::mutex                                   dirty_mutex_;

  // ---------------- Helpers ----------------
  geometry_msgs::Pose vecToPose(Eigen::Vector4d state, double roll = 0.0);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);

public:
  AEPlanner(const ros::NodeHandle& nh);

  void execute(const aeplanner::aeplannerGoalConstPtr& goal);

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void publishMapViz(const ros::TimerEvent&);
};

}  // namespace aeplanner

#endif
