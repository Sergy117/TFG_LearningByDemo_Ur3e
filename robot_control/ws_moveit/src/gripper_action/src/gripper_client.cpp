#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gripper_action/gripperMoveAction.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cstdlib>


#include "tm_msgs/AskItem.h"

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_client");
 
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_demo;
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::AskItem>("tm_driver/ask_item");

  float param_float;

 
  nh.getParam("param", param_float);
 
  

  ROS_INFO("Got parameter : %f", param_float);
  

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<gripper_action::gripperMoveAction> ac("gripperMove", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  gripper_action::gripperMoveGoal goal;
  goal.width = param_float;
  goal.force = 50;
  goal.speed = 50;
  goal.type = 0;
  ac.sendGoal(goal);
  //wait for the action to return
  
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  

  return 0;
}

