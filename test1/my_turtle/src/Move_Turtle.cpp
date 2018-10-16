/**
 * @author Adam King
 *
 * @file Move_Turtle.cpp
 *
 * This program is a publisher that makes the turtlebot move with a speed of 0.2
 * units until it hits on of the walls.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

char shouldContinue = 1;
double poseVals[2] = { -1.0d, -1.0d};

void poseCallback(const turtlesim::Pose::ConstPtr& msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "Move_Turtle");
  ros::NodeHandle node_handle;

  /*
   * Tell the master that we are going to be publishing a message of type 
   * geometry_msgs/Twist on the topic cmd_vel.
   */
  ros::Publisher cmd_vel_publisher = node_handle.advertise<geometry_msgs::\
    Twist>("turtle1/cmd_vel", 1000);

  ros::Subscriber pos_subsriber = node_handle.subscribe("turtle1/pose", 1000, poseCallback);

  // Specify frequency for looping
  ros::Rate loop_rate(10);

  while (ros::ok() && shouldContinue) {

    // Message object to be stuffed with data
    geometry_msgs::Twist msg;
    msg.linear.x = 2.0d;

    //Broadcast the message
    cmd_vel_publisher.publish(msg);

    //In case we need to add a subscription to this application
    ros::spinOnce();

    //Use the ros::Rate object to sleep for the time remaining to get 10Hz publish
    loop_rate.sleep();
  } //while

  if (ros::ok() && !shouldContinue) {
    ROS_INFO("Reached Wall");
  } //if

  return 0;
} //main

void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
  double x = msg->x;
  double y = msg->y;
  double theta = msg->y;
  double lin_vel = msg->linear_velocity;
  double ang_vel = msg->angular_velocity;
  
  ROS_INFO("x: %lf, y: %lf\n",
	   x, y
	   );
  /*
    if (x >= 11.088888 || y >= 11.088888 || theta >= 11.088888) {
    shouldContinue = 0;
    } //if
  */

  if (x <= poseVals[0] && y <= poseVals[1] && (x >= 11.08 || y >= 11.08)) {
    shouldContinue = 0;
  } //if
  
  poseVals[0] = x;
  poseVals[1] = y;
} //poseCallback
