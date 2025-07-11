#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <gripper_action/gripperMoveAction.h>
#include <string> 
#include "std_msgs/String.h"

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/WriteItem.h"
#include "tm_msgs/AskItem.h"


class gripperMoveAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<gripper_action::gripperMoveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  
  gripper_action::gripperMoveResult result_;

public:

  gripperMoveAction(std::string name) :
    as_(nh_, name, boost::bind(&gripperMoveAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~gripperMoveAction(void)
  {
  }

  void executeCB(const gripper_action::gripperMoveGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;



    // publish info to the console for the user
    ROS_INFO("%s: Executing, sending action to gripper with w=%f s=%i f=%i  t=%i", action_name_.c_str(), goal->width, goal->speed, goal->force, goal->type);

  	     
  	ros::NodeHandle nh_demo; 
  	ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::WriteItem>("tm_driver/write_item");
  	
  	tm_msgs::WriteItem send_width;
  	tm_msgs::WriteItem send_speed;
  	tm_msgs::WriteItem send_force;
  	tm_msgs::WriteItem send_type;
  	tm_msgs::WriteItem execute;
  	tm_msgs::WriteItem init_success;
  	tm_msgs::WriteItem init_fail;
  	tm_msgs::WriteItem init_error;
  	
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }
      
	send_width.request.id = "changeWidth";
	send_width.request.item = "g_gripper_width";
	send_width.request.value = std::to_string(goal->width) ;
	
	send_speed.request.id = "changeSpeed";
	send_speed.request.item = "g_gripper_speed";
	send_speed.request.value = std::to_string(goal->speed) ;

	send_force.request.id = "changeForce";
	send_force.request.item = "g_gripper_force";
	send_force.request.value = std::to_string(goal->force) ;
	
	send_type.request.id = "changeType";
	send_type.request.item = "g_gripper_type";
	send_type.request.value = std::to_string(goal->type) ;
	
	
	
	execute.request.id = "execute";
	execute.request.item = "g_gripper_activation";
	execute.request.value = "true" ;
	
    init_success.request.id = "Success";
	init_success.request.item = "g_gripper_success";
	init_success.request.value = "false" ;

    init_fail.request.id = "Fail";
	init_fail.request.item = "g_gripper_failed";
	init_fail.request.value = "false" ;
	
    init_error.request.id = "Error";
	init_error.request.item = "g_gripper_error";
	init_error.request.value = "false" ;
		
	if (client.call(send_width) && client.call(send_speed) && client.call(send_force) && client.call(send_type) && client.call(init_success) && client.call(init_fail) && client.call(init_error) )                             
     {
    	if (send_width.response.ok && send_speed.response.ok && send_force.response.ok && send_type.response.ok && init_success.response.ok  && init_fail.response.ok && init_error.response.ok )
    	{ 
    		ROS_INFO_STREAM("State initialized & (width, speed, force and type) Items sent to robot");
    		
     		 if (client.call(execute)) {
        		ROS_INFO_STREAM("action sent to robot");

        		}
        		
        		
      		 else ROS_WARN_STREAM("action sent to robot , but response not yet ok ");
    	}
    	else ROS_WARN_STREAM("Items sent to robot , but response not yet ok ");
 	}	 
  	else
  	{
    ROS_ERROR_STREAM("Error WriteItem to robot");
  	}
  	
  	

      	ros::NodeHandle nh_demo1; 
  	ros::ServiceClient client1 = nh_demo1.serviceClient<tm_msgs::AskItem>("tm_driver/ask_item");
  	tm_msgs::AskItem ask_success;
  	tm_msgs::AskItem ask_failed;
  	tm_msgs::AskItem ask_error;
  	
  	ask_success.request.id = "gripperSuccess";
  	ask_success.request.item = "g_gripper_success";
  	ask_success.request.wait_time = 1;
  
  //Request 
  	ask_failed.request.id = "gripperFail";
  	ask_failed.request.item = "g_gripper_failed";
  	ask_failed.request.wait_time = 1;
  
    //Request 
  	ask_error.request.id = "gripperError";
  	ask_error.request.item = "g_gripper_error";
  	ask_error.request.wait_time = 1;
  r.sleep();	

    if(success)
    {
    
  	if (client1.call(ask_success) && client1.call(ask_failed) && client1.call(ask_error))                             
  		{
    		if (ask_success.response.ok && ask_failed.response.ok && ask_error.response.ok) 
    		{

      	    
      		ROS_INFO_STREAM("AskItem to robot: id is: " << ask_success.response.id << ", value is: " << ask_success.response.value);
      		ROS_INFO_STREAM("AskItem to robot: id is: " << ask_failed.response.id << ", value is: " << ask_failed.response.value);
      		ROS_INFO_STREAM("AskItem to robot: id is: " << ask_error.response.id << ", value is: " << ask_error.response.value);
      		
    		result_.success = ask_success.response.value == "true";
      	        result_.failed = ask_failed.response.value == "true";
      	        result_.error =  ask_error.response.value == "true";
      	    	as_.setSucceeded(result_);
    		ROS_INFO("%s: Succeeded", action_name_.c_str());
    		// set the action state to succeeded
    		
    		}    	
    		else { 
      		ROS_WARN_STREAM("AskItem to robot , but response not yet ok ");
    		}
  		}
  		}
    else
  	{
    ROS_ERROR_STREAM("Error AskItem to robot");
  	}

  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripperMove");


  
  gripperMoveAction gripperMove("gripperMove");
  ros::spin();

  return 0;
}
