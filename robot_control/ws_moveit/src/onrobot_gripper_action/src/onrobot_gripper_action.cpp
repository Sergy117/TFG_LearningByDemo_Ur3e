/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

//the original code was modified by Mourad ZRELLI and Mohamed Khalil SALLEM
//it was adapted for the onrobot gripper installed on Omron TM5

#include "ros/ros.h"

#include <boost/bind.hpp>

#include <actionlib/server/action_server.h>
#include <onrobot_gripper_msgs/JointControllerState.h>
#include <onrobot_gripper_msgs/OnrobotGripperCommand.h>
#include <onrobot_gripper_msgs/OnrobotGripperCommandAction.h>

#include <string> 
#include "std_msgs/String.h"

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/WriteItem.h"
#include "tm_msgs/AskItem.h"

class OnrobotGripperAction
{
private:
  typedef actionlib::ActionServer<onrobot_gripper_msgs::OnrobotGripperCommandAction> GAS;
  typedef GAS::GoalHandle GoalHandle;
public:
  OnrobotGripperAction(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "onrobot/gripper_action",
                   boost::bind(&OnrobotGripperAction::goalCB, this, _1),
                   boost::bind(&OnrobotGripperAction::cancelCB, this, _1),
                   false),
    has_active_goal_(false)
  {
    ros::NodeHandle pn("~");

    pn.param("goal_threshold", goal_threshold_, 0.02);
    pn.param("stall_velocity_threshold", stall_velocity_threshold_, 1e-6);
    pn.param("stall_timeout", stall_timeout_, 0.1);

    pub_controller_command_ =
      node_.advertise<onrobot_gripper_msgs::OnrobotGripperCommand>("command", 1);
    sub_controller_state_ =
      node_.subscribe("state", 1, &OnrobotGripperAction::controllerStateCB, this);

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &OnrobotGripperAction::watchdog, this);
    action_server_.start();
  }

  ~OnrobotGripperAction()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
  }

private:

  ros::NodeHandle node_;
  GAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Timer watchdog_timer_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  ros::Time goal_received_;

  double min_error_seen_;
  double goal_threshold_;
  double stall_velocity_threshold_;
  double stall_timeout_;
  ros::Time last_movement_time_;

  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->header.stamp).toSec());
      }

      if (should_abort)
      {
        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(GoalHandle gh)
  {
    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
    
  
    ros::ServiceClient client = node_.serviceClient<tm_msgs::WriteItem>("tm_driver/write_item");
  	
    tm_msgs::WriteItem send_width;
    tm_msgs::WriteItem send_force;
    tm_msgs::WriteItem execute;

	send_width.request.id = "changeWidth";
	send_width.request.item = "g_gripper_width";
	send_width.request.value = std::to_string(gh.getGoal()->command.position * 1000) ;
	
	send_force.request.id = "changeForce";
	send_force.request.item = "g_gripper_force";
	send_force.request.value = std::to_string(gh.getGoal()->command.max_effort) ;
	
	execute.request.id = "execute";
	execute.request.item = "g_gripper_activation";
	execute.request.value = "true" ;
	
	if (client.call(send_width) && client.call(send_force))
	{
	 if (send_width.response.ok && send_force.response.ok)
	 {
	   ROS_INFO_STREAM("State initialized & (width, force) Items sent to robot");
	   gh.setAccepted();
           active_goal_ = gh;
           goal_received_ = ros::Time::now();
       if (client.call(execute)) 
       {
          if(execute.response.ok)
            {
            ROS_INFO_STREAM("From gripperAS action sent to robot");
            has_active_goal_ = true;
            
            min_error_seen_ = 1e10;
            // Sends the command along to the controller.
    	    pub_controller_command_.publish(active_goal_.getGoal()->command);
            last_movement_time_ = ros::Time::now();
            
            }
      	  else ROS_WARN_STREAM(" From gripperAS action sent to robot , but response not yet ok ");

	 }
	 else ROS_WARN_STREAM("From gripperAS Items sent to robot , but response not yet ok ");
		
	} 
  	else
  	{
    ROS_ERROR_STREAM("From gripperAS Error WriteItem to robot");
  	}
  }
  
 }

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      if (last_controller_state_)
      {
        onrobot_gripper_msgs::OnrobotGripperCommand stop;
        stop.position = last_controller_state_->process_value;
        stop.max_effort = 0.0;
        pub_controller_command_.publish(stop);
      }

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }



  onrobot_gripper_msgs::JointControllerStateConstPtr last_controller_state_;
  void controllerStateCB(const onrobot_gripper_msgs::JointControllerStateConstPtr &msg)
  {
  
  
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
      return;

    // Ensures that the controller is tracking my setpoint.
    if (fabs(msg->set_point - active_goal_.getGoal()->command.position) > goal_threshold_)
    {
      if (now - goal_received_ < ros::Duration(1.0))
      {
        return;
      }
      else
      {
        ROS_ERROR("Cancelling goal: Controller is trying to achieve a different setpoint.");
        active_goal_.setCanceled();
        has_active_goal_ = false;
      }
    }


    onrobot_gripper_msgs::OnrobotGripperCommandFeedback feedback;
    feedback.position = msg->process_value;
    feedback.effort = msg->command;
    feedback.reached_goal = false;
    feedback.stalled = false;

    onrobot_gripper_msgs::OnrobotGripperCommandResult result;
    result.position = msg->process_value;
    result.effort = msg->command;
    result.reached_goal = false;
    result.stalled = false;

    if (fabs(msg->process_value - active_goal_.getGoal()->command.position ) < goal_threshold_)
    {
      feedback.reached_goal = true;

      result.reached_goal = true;
      active_goal_.setSucceeded(result);
      has_active_goal_ = false;
    }
    else
    {
      
        feedback.stalled = true;
        result.stalled = true;
        active_goal_.setAborted(result);
        has_active_goal_ = false;
      }
         active_goal_.publishFeedback(feedback);
    }

  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_action_node");
  ros::NodeHandle node;
  OnrobotGripperAction jte(node);

  ros::spin();

  return 0;
}
