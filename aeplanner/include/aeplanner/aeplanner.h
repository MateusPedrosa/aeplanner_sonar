#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <ros/ros.h>

#include <mutex>
#include <shared_mutex>

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

#include <pigain/Node.h>
#include <pigain/Query.h>
#include <pigain/BestNode.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>

#include <aeplanner/target_priority_map.h>
#include <aeplanner/directed_sampler.h>
#include <aeplanner/hemisphere_sampler.h>
#include <aeplanner/frontier_sampler.h>
#include <aeplanner/planner_state_machine.h>
#include <aeplanner/dwell_controller.h>
#include <aeplanner/global_target_selector.h>
#include <aeplanner/reachability_checker.h>
#include <std_msgs/String.h>

namespace aeplanner
{
class AEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;

  // Keep track of the best node and its score
  RRTNode* best_node_;
  RRTNode* best_branch_root_;

  std::shared_ptr<la3dm::BGKLOctoMap> ot_;
  // Reader-writer lock for ot_:
  //   cloudCallback (insert_pointcloud) → unique_lock (exclusive write)
  //   gainCubature / collisionLine (search) → shared_lock (concurrent reads)
  //   publishMapViz (begin_leaf iteration) → try shared_lock, skip if writer active
  // Multiple readers proceed simultaneously; only the writer blocks all.
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

  // Override target for REPOSITION state (Phase 4); empty optional = use TPM top
  bool          has_reposition_target_ = false;
  ScoredTarget  reposition_target_;

  // kd tree for finding nearest neighbours
  kdtree* kd_tree_;

  // Subscribers
  ros::Subscriber point_sub_;
  ros::Subscriber agent_pose_sub_;

  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher gain_pub_;
  ros::Publisher bbx_marker_pub_;

  // TF
  tf::TransformListener tf_listener_;

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceServer reevaluate_server_;

  // Service server callback
  bool reevaluate(aeplanner::Reevaluate::Request& req,
                  aeplanner::Reevaluate::Response& res);

  // ---------------- Initialization ----------------
  RRTNode* initialize();
  void initializeKDTreeWithPreviousBestBranch(RRTNode* root);
  void reevaluatePotentialInformationGainRecursive(RRTNode* node);

  // ---------------- Expand RRT Tree ----------------
  void expandRRT();

  Eigen::Vector4d sampleNewPoint();
  bool isInsideBoundaries(Eigen::Vector4d point);
  bool collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r);
  RRTNode* chooseParent(RRTNode* node, double l);
  void rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r, double r_os);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::pair<double, double> getGain(RRTNode* node);
  std::pair<double, double> gainCubature(Eigen::Vector4d state);

  // Priority voxel cache — computed once per planning cycle (expandRRT call)
  // so that the expensive map iteration in Step 1 is not repeated per RRT node.
  struct PriorityVoxel {
    Eigen::Vector3d pos;
    float           priority;
    Eigen::Vector3d v_weak;  // world-frame unit weak arc direction
  };
  std::vector<PriorityVoxel> priority_cache_;
  bool priority_cache_valid_ = false;
  void computePriorityCache();

  // ---------------- Helpers ----------------
  void publishEvaluatedNodesRecursive(RRTNode* node);
  geometry_msgs::Pose vecToPose(Eigen::Vector4d state);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);

  // ---------------- Frontier ----------------
  geometry_msgs::PoseArray getFrontiers();

public:
  AEPlanner(const ros::NodeHandle& nh);

  void execute(const aeplanner::aeplannerGoalConstPtr& goal);

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void publishMapViz(const ros::TimerEvent&);
};

}  // namespace aeplanner

#endif
