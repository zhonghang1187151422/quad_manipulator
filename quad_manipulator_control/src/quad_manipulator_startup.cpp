/**
 * @file quad_manipulator_startup.cpp
 * @brief startup the quad manipulator. includeing px4 Offboard control and manipulator velocity control
 * Stack and tested in Gazebo SITL, ROS control plugin
 */
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



//global variables
bool px4_command_velocity_flag = 0;
geometry_msgs::TwistStamped px4_command_velocity;
bool manipulator_jointstate_flag = 0;
sensor_msgs::JointState manipulator_jointstate;



/*
 * callback function
*/
//get px4 command velocity
void ReadCommandVelocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  px4_command_velocity = *msg;
  px4_command_velocity_flag = 1;
}

//get manipulator joint state
void ReadManipulatorJointState_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  manipulator_jointstate = *msg;
  manipulator_jointstate_flag = 1;
}



/*
 * main function
*/
int main(int argc, char **argv)
{
  //define variables
  FILE *fpr1 = NULL, *fpr2 = NULL, *fpr3 = NULL, *fpr4 = NULL, *fpr5 = NULL, *fpr6 = NULL;

  //run the px4 offboard control
  fpr1 = popen("rosrun quad_manipulator_control quad_velocity_control", "r");
  ROS_INFO("rosrun quad_velocity_control success!\n");

  //ros initial
  ros::init(argc, argv, "quad_manipulator_startup");

  //define ros handle
  ros::NodeHandle startup_node;

  //subscribe to the px4_cmdvel topic
  ros::Subscriber px4_cmdvel_sub = startup_node.subscribe<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000, ReadCommandVelocity_Callback);
  ros::Subscriber manipulator_state_sub = startup_node.subscribe<sensor_msgs::JointState>("/quad_manipulator/joint_states", 1000, ReadManipulatorJointState_Callback);

  //the setpoint publishing rate MUST be faster than 2Hz
  double systemrate = 50.0;
  ros::Rate rate(systemrate);

  // wait for px4 offboard control success
  while(ros::ok() && !px4_command_velocity_flag)
  {
      ros::spinOnce();
      rate.sleep();
  }

  //launch the manipulator ros_control file
  fpr2 = popen("roslaunch quad_manipulator_control manipulator_control.launch", "r");
  ROS_INFO("roslaunch manipulator control success!\n");
  // wait for manipulator control success
  while(ros::ok() && !manipulator_jointstate_flag)
  {
      ros::spinOnce();
      rate.sleep();
  }

  //run the manipulator velocity control
  fpr3 = popen("rosrun quad_manipulator_control manipulator_velocity_node", "r");
  ROS_INFO("rosrun manipulator velocity control success!\n");

  //run the circle blob markers detection
  //fpr4 = popen("rosrun quad_manipulator_control image_circlemarkers_detection", "r");
  fpr4 = popen("roslaunch apriltags_ros example.launch", "r");
  ROS_INFO("roslaunch image circle blob markers detection success!\n");

  //run the quat data publish
  fpr5 = popen("rosrun quad_manipulator_control listener_arm_pose", "r");
  ROS_INFO("rosrun publish end effector quat data success!\n");

  //run the base_link to world
  fpr5 = popen("rosrun quad_manipulator_control broadcaster_base_world", "r");
  ROS_INFO("rosrun broadcaster base_link to world success!\n");

  //while for ros ok
  while(ros::ok())
  {
    //
    ros::spinOnce();
    rate.sleep();
  }


  //close oopen handles
  pclose(fpr1);
  ROS_INFO("stop quad_velocity_control success!\n");
  pclose(fpr2);
  ROS_INFO("stop manipulator_control success!\n");
  pclose(fpr3);
  ROS_INFO("stop manipulator_velocity_node success!\n");
  pclose(fpr4);
  ROS_INFO("stop image_circlemarkers_detection success!\n");
  pclose(fpr5);
  pclose(fpr6);

  //return
  return(0);
}
