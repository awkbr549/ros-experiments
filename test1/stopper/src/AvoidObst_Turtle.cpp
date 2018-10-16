/**
 * @author Adam King
 *
 * @file AvoidObst_Turtle.cpp
 *
 * This program is a publisher that keeps the robot moving until it gets too close
 * to an obstacle in the map, and then stops the robot.
 */

#include <time.h> //subscribe messages
#include <string> //for printing the LaserScan message
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" //publish messages
#include "sensor_msgs/LaserScan.h" //subscribe messages

float MIN_DISTANCE_TO_OBS = 0.5;
char shouldContinue = 1;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "AvoidObst_Turtle");
  ros::NodeHandle node_handle;

  /*
   * Tell the master that we are going to be publishing a message of type
   * geometry_msgs/Twist on the topic <TBD>
   */
  ros::Publisher cmd_vel_publisher = node_handle.advertise<geometry_msgs::\
    Twist>("cmd_vel", 1000);

  /*
   * Tell the master that we are subscribing to a message of type
   * sensor_msgs/LaserScan on the topic base_scan.
   */
  ros::Subscriber scan_subscriber = node_handle.subscribe("base_scan", 1000, scanCallback);

  //Specify frequency for looping
  ros::Rate loop_rate(10);

  while (ros::ok() && shouldContinue) {

    // Message object to be stuffed with data
    geometry_msgs::Twist msg;
    msg.linear.x = 2.0d;

    // Broadcast the message
    cmd_vel_publisher.publish(msg);

    // For the subscription to be caught
    ros::spinOnce();

    // Sleeping to match the desired frequency
    loop_rate.sleep();
  } //while

  if (ros::ok() && !shouldContinue) {
    ROS_INFO("Stop!");
  } //if

  return 0;
} //main

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  
  float range_min = msg->range_min;
  float range_max = msg->range_max;
  std::vector<float> ranges = msg->ranges;
  float closest = range_max;
  for(int i = 0; i < ranges.size(); i++) {
    float temp = ranges[i];

    if (temp < closest) {
      closest = temp;
    } //if
    
    //scrubbing sensor input
    if (temp < range_min) {
      ranges[i] = range_min;
    } //if
    if (temp > range_max) {
      ranges[i] = range_max;
    } //if
    
    //stop the robot if it gets close to an object
    if (temp < MIN_DISTANCE_TO_OBS) {
      shouldContinue = 0;
    } //if
  } //for i
  
  ROS_INFO("Closest range: %f", closest);
} //scanCallback
