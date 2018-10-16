/**
 * File created from the Sending Simple Goals tutorial from the ROS site at 
 * http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
 */

#include <ros/ros.h>

/*
 * Includes the action specification for move_base which is a ROS action that
 * exposes a high-level interface to the navigation stack. Essentially, the
 * move_base action accepts goals from clients and attempts to move the robot to
 * the specified position/orientation in the world.
 */
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

/*
 * This line creates a convenience typedef for a SimpleActionClient that will allow
 * us to communicate with actions that adhere to the MoveBaseAction action
 * interface.
 */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> \
  MoveBaseClient;

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_navigation_goals");

  /*
   * This line constructs an action client that we'll use to communicate with the
   * action named "move_base" that adheres to the MoveBaseAction interface. It also
   * tells the action client to start a thread to call ros::spin() so that ROS
   * callbacks will be processed by passing "true" as the second argument of the
   * MoveBaseClient constructor.
   */
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  /*
   * These lines wait for the action server to report that it has come up and is
   * ready to begin processing goals.
   */
  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  } //while

  /*
   * Here we create a goal to send to move_base using the 
   * move_base_msgs::MoveBaseGoal message type which is included automatically with
   * the MoveBaseActoin.h header. We'll just tell the base to move 1 meter forward
   * in the "base_link" coordinate frame. The call to ac.sendGoal will actually
   * push the goal out over the wire to teh move_base node for processing.
   */
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;
  //send the goal
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  /*
   * The only thing left to do now is to wait for the goal to finish using the
   * ac.waitForGoalToFinish [[ac.waitForResult ?]] call which will block until the 
   * move_base action is done processing the goal we sent it. After it finishes, we
   * can check if the goal succeeded or failed and output a message to the user
   * accordingly.
   */
  ac.waitForResult();
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved 1 meter forward");
  } else {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  } //if-else

  return 0;
} //main
