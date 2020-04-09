#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "control_msgs/JointControllerState.h"

#include <sstream>
#include <math.h>



#define ROS_PI       3.1415



/**
 * declare global variables
*/
std_msgs::Float64MultiArray InitialjointValue;
std_msgs::Float64MultiArray CurrentjointValue;
std_msgs::Float64MultiArray SetJointVelocityValue;
bool ReadJoint1ValueFlag = 0, ReadJoint2ValueFlag = 0, ReadJoint3ValueFlag = 0;



/**
 * subscribe joint1 set velocity value callback function
*/
void JointSetVelocity_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  SetJointVelocityValue.data[0] = msg->data[0];
  SetJointVelocityValue.data[1] = msg->data[1];
  SetJointVelocityValue.data[2] = msg->data[2];
}



/**
 * subscribe joint1 position value callback function
*/
void Joint1ReadPosition_Callback(const control_msgs::JointControllerState::ConstPtr& msg)
{
    CurrentjointValue.data[0] = msg->process_value;
    ReadJoint1ValueFlag = 1;
}

/**
 * subscribe joint2 position value callback function
*/
void Joint2ReadPosition_Callback(const control_msgs::JointControllerState::Ptr& msg)
{
    CurrentjointValue.data[1] = msg->process_value;
    ReadJoint2ValueFlag = 1;
}

/**
 * subscribe joint3 position value callback function
*/
void Joint3ReadPosition_Callback(const control_msgs::JointControllerState::Ptr& msg)
{
    CurrentjointValue.data[2] = msg->process_value;
    ReadJoint3ValueFlag = 1;
}




/**
 * This tutorial demonstrates simple manipulator velocity control example.
 */
int main(int argc, char **argv)
{
  //ros initial
  ros::init(argc, argv, "manipulator_velocity_control");

  //initial the global variables
  InitialjointValue.data.resize(3);
  InitialjointValue.data[0] = 0; InitialjointValue.data[1] = 0; InitialjointValue.data[2] = 0;
  CurrentjointValue.data.resize(3);
  SetJointVelocityValue.data.resize(3); SetJointVelocityValue.data[0] = 0; SetJointVelocityValue.data[1] = 0; SetJointVelocityValue.data[2] = 0;

  //define node handle
  ros::NodeHandle joint_velocitycontrol_node;

  //subscribe the command velocity topics
  ros::Subscriber joint_setvelocity_sub = joint_velocitycontrol_node.subscribe("/quad_manipulator/joint_setvelocity", 1000, JointSetVelocity_Callback);
  //subscriber to the current joint position topics
  ros::Subscriber joint1_readposition_sub = joint_velocitycontrol_node.subscribe("/quad_manipulator/arm_joint1_position_controller/state", 1000, Joint1ReadPosition_Callback);
  ros::Subscriber joint2_readposition_sub = joint_velocitycontrol_node.subscribe("/quad_manipulator/arm_joint2_position_controller/state", 1000, Joint2ReadPosition_Callback);
  ros::Subscriber joint3_readposition_sub = joint_velocitycontrol_node.subscribe("/quad_manipulator/arm_joint3_position_controller/state", 1000, Joint3ReadPosition_Callback);
  //publisher to the commander joint position topics
  ros::Publisher joint1_setposition_pub = joint_velocitycontrol_node.advertise<std_msgs::Float64>("/quad_manipulator/arm_joint1_position_controller/command", 1000);
  ros::Publisher joint2_setposition_pub = joint_velocitycontrol_node.advertise<std_msgs::Float64>("/quad_manipulator/arm_joint2_position_controller/command", 1000);
  ros::Publisher joint3_setposition_pub = joint_velocitycontrol_node.advertise<std_msgs::Float64>("/quad_manipulator/arm_joint3_position_controller/command", 1000);

  //wait read current joint value finished
  while (ros::ok())
  {
    ros::spinOnce();
    if(ReadJoint1ValueFlag && ReadJoint2ValueFlag && ReadJoint3ValueFlag)
        break;
  }

  //set the loop rate
  double rate = 50;
  ros::Rate loop_rate(rate);

  //count
  int count = 0;
  //publisher set joint position value
  std_msgs::Float64MultiArray joint_value;
  joint_value.data.resize(3);
  for(unsigned int i=0; i<3; i++)
  {
    joint_value.data[i] = CurrentjointValue.data[i];
  }

  //set the manipulator in initial pose
  double RunTime = 10.0;
  double dim[3];
  std_msgs::Float64MultiArray jointarraydata[3];
  for(int i=0; i<3; i++)
  {
    dim[i] = (InitialjointValue.data[i]-CurrentjointValue.data[i]) / (RunTime * rate);
    jointarraydata[i].data.resize(static_cast<int>(RunTime * rate));
    for(int j=0; j<= (static_cast<int>(RunTime * rate)); j++)
    {
        jointarraydata[i].data[j] = CurrentjointValue.data[i] + j * dim[i];
    }
  }
  //set data array
  while(ros::ok())
  {
    double error = pow(InitialjointValue.data[0] - CurrentjointValue.data[0], 2) +
        pow(InitialjointValue.data[1] - CurrentjointValue.data[1], 2) +
        pow(InitialjointValue.data[2] - CurrentjointValue.data[2], 2);
    //check the target is or not reach
    if(sqrt(error) < 0.1)
    {
        ROS_INFO("initial joint value reached!\n");
        //stop
        break;
    }
    //set the joint value
    if(count <= static_cast<int>(RunTime * rate))
    {
      for(int i=0; i<3; i++)
      {
        joint_value.data[i] = jointarraydata[i].data[count];
      }
    }
    //publish the target joint value
    std_msgs::Float64 temp;
    temp.data = joint_value.data[0]; joint1_setposition_pub.publish(temp);
    temp.data = joint_value.data[1]; joint2_setposition_pub.publish(temp);
    temp.data = joint_value.data[2]; joint3_setposition_pub.publish(temp);

    //ros spin and loop rate
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  //while for ros ok loop
  while(ros::ok())
  {
    //refresh the set joint position value
    for(unsigned int i=0; i<3; i++)
    {
      //set value
      joint_value.data[i] += (SetJointVelocityValue.data[i] / rate);
    }

    //publisher the joint position value
    std_msgs::Float64 temp;
    temp.data = joint_value.data[0]; joint1_setposition_pub.publish(temp);
    temp.data = joint_value.data[1]; joint2_setposition_pub.publish(temp);
    temp.data = joint_value.data[2]; joint3_setposition_pub.publish(temp);

    //ROS INFO
    //std_msgs::String msg;
    //std::stringstream ss;
    //ss << "publish joint value: " << joint_value.data[0] << " " << joint_value.data[1] << " " << joint_value.data[2] << " " << joint_value.data[3] << " " << joint_value.data[4] << std::endl;
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());

    //ros spin and loop rate
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  //stop joint velocity control
  ROS_INFO("stop joint velocity control!");

  //return
  return(0);
}
