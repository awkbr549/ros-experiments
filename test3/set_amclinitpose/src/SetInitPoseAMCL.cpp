#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>

bool stop;
// callback to verify that initpose is set
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
 if (msgAMCL->pose.pose.position.x > 14.0)
 {
  ROS_INFO("Initial pose is set");
  stop=true;
 }
}

int main(int argc, char** argv) {
    // Initiate new ROS node named "set_amclinitpose_node"
    ros::init(argc, argv, "set_amclinitpose_node");
    
    double itheta, ix, iy; 
    ros::NodeHandle n;
    ix=14.0;
    iy=28.4;
    itheta=0.0;

    ros::Publisher initpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Subscriber sub_amclpose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, poseAMCLCallback);//new
    stop=false;//new
    geometry_msgs::PoseWithCovarianceStamped start_msg;
    start_msg.header.stamp=ros::Time::now();
    start_msg.header.frame_id="map";
    start_msg.pose.pose.position.x=ix; 
    start_msg.pose.pose.position.y=iy;
    start_msg.pose.pose.position.z=0.0;
    
    // Convert the Euler angle to quaternion
    double radians = itheta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);

    start_msg.pose.pose.orientation = qMsg;
    // start_msg.pose.covariance[0] = 10;
    start_msg.pose.covariance[0] = 15;
    for(size_t i=1; i<7; ++i) {
        start_msg.pose.covariance[i] = 0;
    } //for
    // start_msg.pose.covariance[7] = 10;
    start_msg.pose.covariance[7] = 0;
    for(size_t i=8; i<36; ++i) {
        start_msg.pose.covariance[i] = 0;
    } //for
    
    ros::Rate rate(10);    
    while (ros::ok())
    {
    if (stop==false)
     initpose_pub.publish(start_msg);
    else
     ros::shutdown();
    ros::spinOnce();
    rate.sleep();
    }
    return 0;
};


