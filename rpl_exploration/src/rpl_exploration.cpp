#include <fstream>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <unistd.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <aeplanner_evaluation/Coverage.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <aeplanner/Node.h>
#include <aeplanner/aeplannerAction.h>
#include <rpl_exploration/FlyToAction.h>
#include <rrtplanner/rrtAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <nav_msgs/Path.h>
#include <tf2/utils.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ROS_INFO("Started exploration");

  // Open logfile;
  std::string path = ros::package::getPath("rpl_exploration");
  std::ofstream logfile, pathfile;
  logfile.open(path + "/data/logfile.csv");
  pathfile.open(path + "/data/path.csv");

  ros::Publisher pub(nh.advertise<geometry_msgs::PoseStamped>("/mavros/"
                                                              "setpoint_position/"
                                                              "local",
                                                              1000));
  ros::Publisher run_time_pub(nh.advertise<std_msgs::Float64>("/viewplanner/run_time_s", 1, true));

  ros::ServiceClient coverage_srv =
      nh.serviceClient<aeplanner_evaluation::Coverage>("/get_coverage");

  // wait for fly_to server to start
  // ROS_INFO("Waiting for fly_to action server");
  actionlib::SimpleActionClient<rpl_exploration::FlyToAction> ac("fly_to", true);
  // ac.waitForServer(); //will wait for infinite time
  // ROS_INFO("Fly to ction server started!");

  // wait for aep server to start
  ROS_INFO("Waiting for aeplanner action server");
  actionlib::SimpleActionClient<aeplanner::aeplannerAction> aep_ac("make_plan",
                                                                   true);
  aep_ac.waitForServer();  // will wait for infinite time
  ROS_INFO("aeplanner action server started!");

  // wait for fly_to server to start
  ROS_INFO("Waiting for rrt action server");
  actionlib::SimpleActionClient<rrtplanner::rrtAction> rrt_ac("rrt", true);
  // rrt_ac.waitForServer(); //will wait for infinite time
  ROS_INFO("rrt Action server started!");

  // Get current pose
  geometry_msgs::PoseStamped::ConstPtr init_pose =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/oceansim/robot/pose"); // "/mavros/local_position/pose"
  double init_yaw = tf2::getYaw(init_pose->pose.orientation);
  double init_x    = init_pose->pose.position.x;
  double init_y    = init_pose->pose.position.y;
  double init_z    = init_pose->pose.position.z;
  double init_z_up = init_z + 2.0;
  double yaw90     = init_yaw + M_PI / 2.0;

  // Initialization motion:
  //  1. Rise to +2 m
  //  2-5. Spin 360° in 4×90° steps to carve free space in all directions
  //  6. Forward 2 m (carve ahead, stay here)
  double fwd_x = init_x + 2.0 * std::cos(init_yaw);
  double fwd_y = init_y + 2.0 * std::sin(init_yaw);
  double initial_positions[6][4] = {
    { init_x,  init_y,  init_z_up, init_yaw },               // 1 – rise
    { init_x,  init_y,  init_z_up, init_yaw + M_PI / 2.0 },  // 2 – +90°
    { init_x,  init_y,  init_z_up, init_yaw + M_PI },         // 3 – +180°
    { init_x,  init_y,  init_z_up, init_yaw + 3*M_PI / 2.0},  // 4 – +270°
    { init_x,  init_y,  init_z_up, init_yaw },               // 5 – back to original yaw
    { fwd_x,   fwd_y,   init_z_up, init_yaw },               // 6 – forward
  };
  const int n_init = 6;
  // This is the initialization motion, necessary that the known free space
  // allows the planning of initial paths.
  ROS_INFO("Starting the planner: Performing initialization motion");
  geometry_msgs::PoseStamped last_pose;

  for (int i = 0; i < n_init; ++i)
  {
    rpl_exploration::FlyToGoal goal;
    goal.pose.pose.position.x = initial_positions[i][0];
    goal.pose.pose.position.y = initial_positions[i][1];
    goal.pose.pose.position.z = initial_positions[i][2];
    goal.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(initial_positions[i][3]);
    last_pose.pose = goal.pose.pose;

    ROS_INFO_STREAM("Sending initial goal " << i + 1 << "/" << n_init << " ...");
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(0));

    last_pose.header.stamp = ros::Time::now();
    last_pose.header.frame_id = "map";
    pub.publish(last_pose);
  }

  // Start planning: The planner is called and the computed path sent to the
  // controller.
  int iteration = 0;
  int actions_taken = 1;
  double total_distance_m = 0.0;

  ros::Time start = ros::Time::now();
  while (ros::ok())
  {
    ROS_INFO_STREAM("Planning iteration " << iteration);
    aeplanner::aeplannerGoal aep_goal;
    aep_goal.header.stamp = ros::Time::now();
    aep_goal.header.seq = iteration;
    aep_goal.header.frame_id = "map";
    aep_goal.actions_taken = actions_taken;
    aep_ac.sendGoal(aep_goal);

    while (!aep_ac.waitForResult(ros::Duration(0.05)))
    {
      last_pose.header.stamp = ros::Time::now();
      last_pose.header.frame_id = "map";
      pub.publish(last_pose);
    }

    ros::Duration fly_time;
    if (aep_ac.getResult()->is_clear)
    {
      actions_taken = 0;

      ros::Time s = ros::Time::now();
      geometry_msgs::PoseStamped goal_pose = aep_ac.getResult()->pose;
      // Write path to file
      pathfile << goal_pose.pose.position.x << ", " << goal_pose.pose.position.y
               << ", " << goal_pose.pose.position.z << ", n" << std::endl;

      {
        auto& p0 = last_pose.pose.position;
        auto& p1 = goal_pose.pose.position;
        double dx = p1.x-p0.x, dy = p1.y-p0.y, dz = p1.z-p0.z;
        total_distance_m += std::sqrt(dx*dx + dy*dy + dz*dz);
      }
      last_pose.pose = goal_pose.pose;
      rpl_exploration::FlyToGoal goal;
      goal.pose = goal_pose;
      ac.sendGoal(goal);

      ac.waitForResult(ros::Duration(0));

      fly_time = ros::Time::now() - s;
    }
    else
    {
      rrtplanner::rrtGoal rrt_goal;
      rrt_goal.start.header.stamp = ros::Time::now();
      rrt_goal.start.header.frame_id = "map";
      rrt_goal.start.pose = last_pose.pose;
      if (!aep_ac.getResult()->frontiers.poses.size())
      {
        // On early iterations the map may still be mostly UNKNOWN and pigain has
        // no frontiers yet — don't quit, just wait for more sonar data and retry.
        if (iteration < 5)
        {
          ROS_WARN_STREAM("No frontiers (iteration " << iteration
            << "); map may still be initialising. Waiting 2 s before retrying...");
          ros::Duration(2.0).sleep();
          continue;
        }
        ROS_WARN("Exploration complete!!!!!!!!!!!!!!!!!!!!!!!!!");

        double total_time_s = (ros::Time::now() - start).toSec();
        ROS_INFO("[VIEWPLANNER] Total mission run time: %.2f s", total_time_s);

        // Publish run time (latched)
        std_msgs::Float64 rt;
        rt.data = total_time_s;
        run_time_pub.publish(rt);
        ros::Duration(0.1).sleep();

        // Create timestamped results directory
        std::time_t now_t = std::time(nullptr);
        char ts_buf[32];
        std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%d_%H-%M-%S",
                      std::localtime(&now_t));
        std::string ts(ts_buf);
        std::string results_base = path + "/exploration_results";
        std::string run_dir = results_base + "/" + ts;
        system(("mkdir -p " + run_dir).c_str());

        // Query coverage if the service is available
        double coverage_pct = -1.0;
        {
          aeplanner_evaluation::Coverage cov_srv;
          if (coverage_srv.call(cov_srv))
            coverage_pct = cov_srv.response.coverage;
        }

        // Capture git commit hash (local read only, no network calls)
        std::string git_hash = "unknown";
        {
          FILE* pipe = popen(("git -C " + path + " rev-parse HEAD 2>/dev/null").c_str(), "r");
          if (pipe) {
            char buf[64] = {};
            if (fgets(buf, sizeof(buf), pipe)) {
              git_hash = buf;
              git_hash.erase(git_hash.find_last_not_of("\n\r") + 1);
            }
            pclose(pipe);
          }
        }

        // Write results.txt
        {
          char hostname[256] = "unknown";
          gethostname(hostname, sizeof(hostname));

          std::ofstream rf(run_dir + "/results.txt");
          rf << "=== Exploration Run Results ===\n"
             << "Date/Time:       " << ts << "\n"
             << "Hostname:        " << hostname << "\n"
             << "Git commit:      " << git_hash << "\n"
             << "Run time (s):    " << std::fixed << std::setprecision(2)
                                    << total_time_s << "\n"
             << "Distance (m):    " << std::setprecision(2)
                                    << total_distance_m << "\n"
             << "Iterations:      " << iteration << "\n";
          if (coverage_pct >= 0.0)
            rf << "Map coverage:    " << std::setprecision(1)
               << coverage_pct * 100.0 << " %\n";
          rf << "Final pose:      x=" << std::setprecision(3)
             << last_pose.pose.position.x
             << " y=" << last_pose.pose.position.y
             << " z=" << last_pose.pose.position.z << "\n"
             << "\n"
             << "Files in this directory:\n"
             << "  logfile.csv  — per-iteration: iter, elapsed_s, sampling_s,"
                " planning_s, collision_s, fly_s, tree_size\n"
             << "  path.csv     — waypoints visited (x, y, z, type)"
                " where type=n (AEP) or f (RRT frontier)\n"
             << "  map.ply        — final map variance state (colour = variance,"
                " blue=low, red=high)\n"
             << "  ros_params.yaml — all live ROS parameters at mission end\n"
             << "\n"
             << "=== exploration.yaml ===\n";
          std::ifstream ey(path + "/config/exploration.yaml");
          rf << ey.rdbuf();
          rf << "\n=== viewplanner_params.yaml ===\n";
          std::ifstream vy(path + "/config/viewplanner_params.yaml");
          rf << vy.rdbuf();
        }

        // Close and relocate CSV files into the run directory
        pathfile.close();
        logfile.close();
        system(("mv " + path + "/data/logfile.csv " + run_dir + "/logfile.csv").c_str());
        system(("mv " + path + "/data/path.csv "    + run_dir + "/path.csv").c_str());

        // Dump all live ROS parameters (captures any launch-time overrides)
        system(("rosparam dump " + run_dir + "/ros_params.yaml 2>/dev/null").c_str());

        // Capture final map state as a PLY file (runs as a background ROS node)
        std::string ply_script = path + "/nodes/variance_to_ply.py";
        system(("python3 " + ply_script + " " + run_dir + "/map.ply &").c_str());

        ROS_INFO("[VIEWPLANNER] Results saved to: %s", run_dir.c_str());

        ros::shutdown();
        return 0;
      }
      for (auto it = aep_ac.getResult()->frontiers.poses.begin();
           it != aep_ac.getResult()->frontiers.poses.end(); ++it)
      {
        rrt_goal.goal_poses.poses.push_back(*it);
      }

      rrt_ac.sendGoal(rrt_goal);
      while (!rrt_ac.waitForResult(ros::Duration(0.05)))
      {
        last_pose.header.stamp = ros::Time::now();
        last_pose.header.frame_id = "map";
        pub.publish(last_pose);
      }
      nav_msgs::Path path = rrt_ac.getResult()->path;

      ros::Time s = ros::Time::now();
      for (int i = path.poses.size() - 1; i >= 0; --i)
      {
        geometry_msgs::Pose goal_pose = path.poses[i].pose;
        // Write path to file
        pathfile << goal_pose.position.x << ", " << goal_pose.position.y << ", "
                 << goal_pose.position.z << ", f" << std::endl;

        {
          auto& p0 = last_pose.pose.position;
          double dx = goal_pose.position.x-p0.x, dy = goal_pose.position.y-p0.y, dz = goal_pose.position.z-p0.z;
          total_distance_m += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        last_pose.pose = goal_pose;
        rpl_exploration::FlyToGoal goal;
        goal.pose.pose = goal_pose;
        ac.sendGoal(goal);

        ac.waitForResult(ros::Duration(0));
      }
      actions_taken = -1;
      fly_time = ros::Time::now() - s;
    }

    ros::Duration elapsed = ros::Time::now() - start;

    ROS_INFO_STREAM("Iteration: "       << iteration << "  " <<
                    "Time: "            << elapsed << "  " <<
                    "Sampling: "        << aep_ac.getResult()->sampling_time.data << "  " <<
                    "Planning: "        << aep_ac.getResult()->planning_time.data << "  " <<
                    "Collision check: " << aep_ac.getResult()->collision_check_time.data  << "  " <<
                    "Flying: "          << fly_time << " " <<
                    "Tree size: "       << aep_ac.getResult()->tree_size);

    logfile << iteration << ", " 
            << elapsed << ", "
            << aep_ac.getResult()->sampling_time.data << ", "
            << aep_ac.getResult()->planning_time.data << ", "
            << aep_ac.getResult()->collision_check_time.data << ", "
            << fly_time << ", "
            << aep_ac.getResult()->tree_size << std::endl;

    iteration++;
  }

  pathfile.close();
  logfile.close();
}
