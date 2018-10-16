#include <unistd.h> //for usleep
#include <stdlib.h> //for rand()

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/*
 *For this application, processes the sensor information to ensure the robot does
 *not hit an obstacle if it is less than the MIN_DIST_TO_OBS (the acceptable
 *minimum distance to an obstacle).
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

/*
 *Initializes the ros::Publisher that will broadcast velocity information and
 *initializes the ros::Subscriber that is listening for sensor data.
 */
void setupSubsAndPubs();

/*
 *Initializes the initial angular speed based on ANG_SPEED by choosing either
 *(+)ANG_SPEED or (-)ANG_SPEED at the ratio of the user's choosing.
 *
 *@args double prob - the probability (decimal) which the function will initialize
 *  the angular speed to (+)ANG_SPEED
 */
void setupAngSpeed(double prob);

/*
 *Returns `1' at the probability specified by the user.
 *
 *@args double prob - the probability (decimal) which the function will return 
 *  `1'
 */
int random(double prob);

/*
 *
 */
void timerCallback(const ros::TimerEvent& e);
