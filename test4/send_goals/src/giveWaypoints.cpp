/**
 * File created based on code found in the Sending Simple Goals tutorial from the
 * ROS site at http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
 */

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//to stop the zero length quaternion error
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> \
  MoveBaseClient;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server...");
  } //while

  //taking care of the zero length quaternion error
  tf::Quaternion quat = tf::createQuaternionFromYaw(0.0);
  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quat, qMsg);

  //create an array of goals
  move_base_msgs::MoveBaseGoal goal[5];
  for (int i = 0; i < 4; i++) {
    goal[i].target_pose.header.frame_id = "map";
    goal[i].target_pose.pose.orientation = qMsg;
  } //for i

  //assigning individual coordinates to specified goals
  goal[0].target_pose.pose.position.x = 18.0;
  goal[0].target_pose.pose.position.y = 28.5;

  goal[1].target_pose.pose.position.x = 20.0;
  goal[1].target_pose.pose.position.y = 28.5;

  goal[2].target_pose.pose.position.x = 20.0;
  goal[2].target_pose.pose.position.y = 25.0;

  goal[3].target_pose.pose.position.x = 17.0;
  goal[3].target_pose.pose.position.y = 25.0;
  
  //sending the goals after the previous one finishes
  for (int i = 0; i < 4 && ros::ok(); i++) {
    //sending the goal
    ROS_INFO("Sending next goal...");
    goal[i].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[i]);

    //wait for the goal to finish
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The base has reached the goal. Continuing...");
    } else {
      ROS_INFO("The base failed to reach the goal for some reason... Exiting.");
      break;
    } //if-else
  } //for i
  
  return EXIT_SUCCESS;
} //main
