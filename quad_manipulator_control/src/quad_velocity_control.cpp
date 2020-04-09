/**
 * @file quad_velocity_control.cpp
 * @brief quad manipulator control. px4 Offboard control
 * Stack and tested in Gazebo SITL
 */
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



//global variables
mavros_msgs::State px4_current_state;
geometry_msgs::PoseStamped px4_current_pose;
geometry_msgs::TwistStamped px4_current_velocity;
geometry_msgs::TwistStamped px4_set_velocity;



/*
 * callback function
*/
//get current px4 state
void ReadCurrentState_Callback(const mavros_msgs::State::ConstPtr& msg)
{
  px4_current_state = *msg;
}

//get current quadrotor pose
void ReadCurrentPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  px4_current_pose = *msg;
}

//get current quadrotor velocity
void ReadCurrentVelocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  px4_current_velocity = *msg;
}

//get quadrotor command velocity
void ReadCommandVelocity_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  px4_set_velocity = *msg;
}




/*
 * main function
*/
int main(int argc, char **argv)
{
  //initial global variables
  px4_set_velocity.twist.linear.x = 0.0; px4_set_velocity.twist.linear.y = 0.0; px4_set_velocity.twist.linear.z = 0.0;
  px4_set_velocity.twist.angular.x = 0.0; px4_set_velocity.twist.angular.y = 0.0; px4_set_velocity.twist.angular.z = 0.0;

  //ros initial
  ros::init(argc, argv, "quad_velocity_control");

  //define ros handle
  ros::NodeHandle px4_offb_node;

  //subscribe to the px4 current state
  ros::Subscriber px4_currentstate_sub = px4_offb_node.subscribe<mavros_msgs::State>("mavros/state", 100, ReadCurrentState_Callback);
  ros::Subscriber px4_currentpose_sub = px4_offb_node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, ReadCurrentPose_Callback);
  ros::Subscriber px4_currentvelocity_sub = px4_offb_node.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 100, ReadCurrentVelocity_Callback);
  //subscribe to the set velocity topic
  ros::Subscriber px4_setvelocity_sub = px4_offb_node.subscribe<geometry_msgs::TwistStamped>("/quad/setvelocity", 100, ReadCommandVelocity_Callback);
  //service call
  ros::ServiceClient px4_arming_client = px4_offb_node.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient px4_set_mode_client = px4_offb_node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  //publisher the target position and velocity
  ros::Publisher px4_localpos_pub = px4_offb_node.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
  ros::Publisher px4_cmdvel_pub = px4_offb_node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);

  //the setpoint publishing rate MUST be faster than 2Hz
  double systemrate = 50.0;
  ros::Rate rate(systemrate);

  /*
   * px4 enter offboard mode pre-ready
  */
  // wait for FCU connection
  while(ros::ok() && !px4_current_state.connected)
  {
      ros::spinOnce();
      rate.sleep();
  }
  //send a few setpoints before starting
  unsigned int count = 0;
  bool control_mode = 0;
  geometry_msgs::PoseStamped readypose;
  readypose.pose.position.x = 0;
  readypose.pose.position.y = 0;
  readypose.pose.position.z = 2;
  for(int i = 100; ros::ok() && i > 0; --i)
  {
      px4_localpos_pub.publish(readypose);
      ros::spinOnce();
      rate.sleep();
  }
  //pre-set command
  mavros_msgs::SetMode offb_set_mode; offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd; arm_cmd.request.value = true;
  ros::Time last_request = ros::Time::now();

  //while for ros ok
  while(ros::ok())
  {
    //check the px4 mode and enter in offboard mode
    if(px4_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if(px4_set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled\n");
      }
      last_request = ros::Time::now();
    }else
    {
      if(!px4_current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if(px4_arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed\n");
        }
        last_request = ros::Time::now();
      }
    }

    //control px4 in position control mode or velocity control mode
    double dist = sqrt(pow((px4_current_pose.pose.position.x - readypose.pose.position.x), 2) + pow((px4_current_pose.pose.position.y - readypose.pose.position.y), 2) +
        pow((px4_current_pose.pose.position.z - readypose.pose.position.z), 2));
    if(control_mode == 0)
    {
      if(dist < 0.1)
      {
        count++;
        if(count > 100)
        {
          control_mode = 1;
          ROS_INFO("px4 offboard in velocity control mode\n");
        }
      }else
      {
        //clear count
        count = 0;
      }
      //local position control mode
      px4_localpos_pub.publish(readypose);
    }else
    {
      //set the command velocity to px4
      px4_cmdvel_pub.publish(px4_set_velocity);
    }

    ros::spinOnce();
    rate.sleep();
  }


  return(0);
}
