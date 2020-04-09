#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"


void globalposeCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  //define variables
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  //set the transform position origin
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
  //set the transform orientation
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, \
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  transform.setRotation(q);
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_world_tf_broadcaster");
  
  ros::NodeHandle nh;
  ros::Subscriber globalpos_sub = nh.subscribe("/mavros/global_position/local", 10, &globalposeCallback);
  
  ros::spin();
  return 0;
};