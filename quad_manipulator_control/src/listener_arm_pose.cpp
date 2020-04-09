#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include "eigen3/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_arm_pose");
  ros::NodeHandle nh;

  //pub the arm end effector pose data
  ros::Publisher pose_pub = nh.advertise<std_msgs::Float64MultiArray>("/quad_manipulator/endeffector/quat", 1000);

  //tf listener
  tf::TransformListener listener;

  //ros rate
  ros::Rate rate(10.0);

  while(nh.ok())
  {
    tf::StampedTransform transform;

    //try get the data
    try
    {
      listener.lookupTransform("base_link", "arm_link_3", ros::Time(0), transform);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // pub the data
    tf::Quaternion quat_data;
    quat_data = transform.getRotation();
    std_msgs::Float64MultiArray quat_array;
    quat_array.data.resize(4);
    quat_array.data[0] = quat_data.getW();
    quat_array.data[1] = quat_data.getX();
    quat_array.data[2] = quat_data.getY();
    quat_array.data[3] = quat_data.getZ();
    pose_pub.publish(quat_array);

    //
    rate.sleep();
  }

  return(0);
}
