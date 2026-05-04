#include <rpl_exploration/FlyToAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace rpl_exploration {
  class FlyTo
  {
    private:
      ros::NodeHandle nh_;
      ros::Publisher pub_;
      ros::Publisher nbv_pub_;
      actionlib::SimpleActionServer<rpl_exploration::FlyToAction> as_;

      tf::TransformListener listener;
    public:
      FlyTo() : pub_(nh_.advertise<geometry_msgs::PoseStamped>("fly_to_cmd", 1000)),
                nbv_pub_(nh_.advertise<geometry_msgs::PoseStamped>("/aeplanner/nbv_setpoint", 1000)),
                as_(nh_, "fly_to", boost::bind(&FlyTo::execute, this, _1, &as_), false)
      {
        ROS_INFO("Starting fly to server");
        as_.start();
      }
      void execute(const rpl_exploration::FlyToGoalConstPtr& goal, 
                   actionlib::SimpleActionServer<rpl_exploration::FlyToAction> * as)
      {
        ROS_INFO_STREAM("Got new goal: Fly to (" << goal->pose.pose.position.x << ", "
                                                 << goal->pose.pose.position.y << ", "
                                                 << goal->pose.pose.position.z << ") ");

        ros::Rate r(20);
        geometry_msgs::Point p = goal->pose.pose.position;

        float distance_to_goal = 9001; // Distance is over 9000
        float yaw_diff  = M_PI;
        float roll_diff = M_PI;

        tf::StampedTransform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

        tf::Quaternion gq(
          goal->pose.pose.orientation.x,
          goal->pose.pose.orientation.y,
          goal->pose.pose.orientation.z,
          goal->pose.pose.orientation.w);
        tf::Matrix3x3 m(gq);
        double goal_roll, pitch, goal_yaw;
        m.getRPY(goal_roll, pitch, goal_yaw);

        // Publish the goal exact once to the nbv_setpoint topic
        geometry_msgs::PoseStamped nbv_pose = goal->pose;
        nbv_pose.header.stamp = ros::Time::now();
        nbv_pose.header.frame_id = "map";
        nbv_pub_.publish(nbv_pose);

        // Check if target is reached...
        do
        {
          ROS_INFO_STREAM("Publishing goal to (" << p.x << ", " << p.y << ", " << p.z << ") ");
          geometry_msgs::PoseStamped goal_pose = goal->pose;
          goal_pose.header.stamp = ros::Time::now();
          goal_pose.header.frame_id = "map";
          pub_.publish(goal_pose);

          listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

          geometry_msgs::Point q;
          q.x = (float)transform.getOrigin().x(); 
          q.y = (float)transform.getOrigin().y(); 
          q.z = (float)transform.getOrigin().z();

          geometry_msgs::Quaternion tq;
          tq.x = (float)transform.getRotation().x(); 
          tq.y = (float)transform.getRotation().y(); 
          tq.z = (float)transform.getRotation().z();
          tq.w = (float)transform.getRotation().w();
          tf::Quaternion cq( tq.x, tq.y, tq.z, tq.w);
          tf::Matrix3x3 m(cq);
          double current_roll, current_pitch, current_yaw;
          m.getRPY(current_roll, current_pitch, current_yaw);

          ROS_INFO_STREAM("Current position: (" << q.x << ", " << q.y << ", " << q.z << ") ");
          geometry_msgs::Point d; 
          d.x = p.x - q.x;
          d.y = p.y - q.y;
          d.z = p.z - q.z;

          distance_to_goal = sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
          ROS_INFO_STREAM("Distance to goal: " << distance_to_goal);
          yaw_diff  = fabs(atan2(sin(goal_yaw  - current_yaw),  cos(goal_yaw  - current_yaw)));
          roll_diff = fabs(atan2(sin(goal_roll - current_roll), cos(goal_roll - current_roll)));

          r.sleep();
        } while(distance_to_goal > 0.2 or yaw_diff > (1.0/18.0)*M_PI or roll_diff > (1.0/18.0)*M_PI);


        as->setSucceeded();
      }
  };


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fly_to_server");

  rpl_exploration::FlyTo fly_to;

  ros::spin();
  return 0;
}
