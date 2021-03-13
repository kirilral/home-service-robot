#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal source;

  // set up the frame parameters
  source.target_pose.header.frame_id = "map";
  source.target_pose.header.stamp = ros::Time::now();

  // Go get the beer can
  source.target_pose.pose.position.x = -1.11564;
  source.target_pose.pose.position.y = 1.4267;
  source.target_pose.pose.orientation.w = 1.0;
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending beer can coordinates");
  ac.sendGoal(source);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached the beer can. Picking it up");
  else
    ROS_INFO("I couldn't reach the beer can");

  move_base_msgs::MoveBaseGoal sink;

  // set up the frame parameters
  sink.target_pose.header.frame_id = "map";
  sink.target_pose.header.stamp = ros::Time::now();  

  sink.target_pose.pose.position.x = -4.0925;
  sink.target_pose.pose.position.y = 1.4228;
  sink.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal coordinates");
  ac.sendGoal(sink);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Dropped off beer can");
  else
    ROS_INFO("I couldn't reach the window side");



  return 0;
}
