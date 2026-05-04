#include <fstream>

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
        ROS_WARN("Exploration complete!");
        ros::shutdown();
        break;
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
