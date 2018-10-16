#include "ExploreMap.h"

#define MIN_DIST_TO_OBS 0.5f
#define ACCEPT_DIST_TO_OBS 4 * MIN_DIST_TO_OBS
#define LIN_SPEED 0.5f //1.0f //0.5f
#define ANG_SPEED 0.78539816339744 / 2 //PI / 4 / 2 --> 45deg
//1.57079632679489 / 2 //PI / 2 / 2 --> 90deg
//1.04719755119659 / 2 //PI / 3 / 2 --> 60deg
//0.78539816339744 / 2 //PI / 4 / 2--> 45deg
#define LOOP_RATE 10 //Hz
#define DURATION_MIN 60

bool shouldChangeDir = false;
bool shouldSpin = false;
bool shouldTerminate = false;
float angSpeed;

ros::Publisher cmdVelPub;
ros::Subscriber baseScanSub;
ros::Timer timer;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ExploreMap");
  
  setupSubsAndPubs();
  setupAngSpeed(0.5d);

  /*Initializing cmd_vel variables.*/
  ros::Rate loopRate(LOOP_RATE); //don't publish all the time
  geometry_msgs::Twist cmd; //message for cmd_vel publisher
  cmd.linear.x = LIN_SPEED;
  
  /*Exploring the map*/
  while(ros::ok() && !shouldTerminate) {
    ros::spinOnce();
    if (!shouldChangeDir) {
      for (int i = 0;
	   i < 2 * (LOOP_RATE + (4 * random(0.5d)) - 2) && !shouldChangeDir;
	   i++) {
	cmd.linear.x = LIN_SPEED;
	cmd.angular.z = (angSpeed < 0) ?
	  1.57079632679489 * angSpeed :
	  angSpeed;
	cmdVelPub.publish(cmd);
	ros::spinOnce();
	loopRate.sleep();
      } //for i
      angSpeed = -angSpeed;
    } else { //collision impending

      ROS_INFO("\nBacking up...\n");
      shouldChangeDir = false;
      cmd.linear.x = -LIN_SPEED;
      //cmd.angular.z = 0.0;
      cmdVelPub.publish(cmd);
      ros::spinOnce();
      loopRate.sleep();

      ROS_INFO("\nSpinning 45-ish degrees...\n");
      cmd.linear.x = 0.0;
      angSpeed = -angSpeed;
      cmd.angular.z = angSpeed;
      for (int i = 0;
	   i < 2 * (LOOP_RATE + (4 * random(0.5d)) - 2);
	   i++) {
	cmdVelPub.publish(cmd);
	ros::spinOnce();
	loopRate.sleep();
      } //for i

      ROS_INFO("\nContinuing forward...\n");
      shouldChangeDir = false;
    } //if-else

    loopRate.sleep();
  } //while
  /*******************************************************/
  
  return EXIT_SUCCESS;
} //main

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

  float rangeMin = msg->range_min;
  float rangeMax = msg->range_max;
  std::vector<float> ranges = msg->ranges;
  float closest = rangeMax;
  float temp;
  int randResult = random(0.05d);
  int leftOrRight = random(0.5d);
  for (int i = 0; i < ranges.size(); i++) {
    temp = ranges[i];
    
    //scrubbing sensor input
    if (temp < rangeMin || temp > rangeMax) { continue; }
    
    if (temp < closest) { closest = temp; }

    //stop the robot if it gets close to an object
    if (temp < MIN_DIST_TO_OBS) {
      shouldChangeDir = true;
      ROS_INFO("Found obstacle too close: %f\n", temp);
      return;
    } //if
  } //for i

  //if the function made it to here, then there was no obstacle too close
  if (closest > ACCEPT_DIST_TO_OBS) {
    shouldChangeDir = false;
  } //if
} //scanCallback

/*
 *Setting up publishers and subscribers
 */
void setupSubsAndPubs() {
  ros::NodeHandle nh;
  //to publish to /cmd_vel
  cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  //to subscribe to /base_scan
  baseScanSub = nh.subscribe("base_scan", 1000, scanCallback);
  //the timer to stop exploring
  timer = nh.createTimer(ros::Duration(DURATION_MIN * 60, 0), timerCallback);
} //setupSubsAndPubs

void setupAngSpeed(double prob) {
  if (random(prob) == 1) {
    angSpeed = ANG_SPEED;
  } else {
    angSpeed = -ANG_SPEED;
  } //if-else
} //setupAngSpeed

int random(double prob) {
  /*for (int i = 0; i < 100; i++) {
    ROS_INFO("%d\n", (bool)(prob + ((double)rand() / (double)RAND_MAX)));
    } //for i*/
  return (int)(prob + ((double)rand() / (double)RAND_MAX));
} //random

void timerCallback(const ros::TimerEvent& e) {
  shouldTerminate = true;
} //timerCallback
