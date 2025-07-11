#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include <sensor_msgs/JointState.h>
#include <onrobot_gripper_msgs/JointControllerState.h>

// std header
#include <sstream>
#include <cstdlib>
#include <stdlib.h>

// TM Driver header

#include "tm_msgs/AskItem.h"
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  { 
    
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::JointState>("joint_states",1);
    pub1_ = n_.advertise<onrobot_gripper_msgs::JointControllerState>("state",1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/joint_states",1, &SubscribeAndPublish::callback, this);
  }
  

  void callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    
    sensor_msgs::JointState msg1;
    
    onrobot_gripper_msgs::JointControllerState msg2;
    
    	


  	

  	
  	
  	ros::ServiceClient client = n_.serviceClient<tm_msgs::AskItem>("tm_driver/ask_item");
  	tm_msgs::AskItem gripper_joint_position;
  	tm_msgs::AskItem gripper_joint_effort;
  	tm_msgs::AskItem target_width;
  
  	gripper_joint_position.request.id = "GripperJointPosition";
  	gripper_joint_position.request.item = "g_Gripper_OR_2FG7_ExternalWidth";
  	gripper_joint_position.request.wait_time = 0.1;
  
  	gripper_joint_effort.request.id = "GripperJointEffort";
  	gripper_joint_effort.request.item = "g_current_force";
  	gripper_joint_effort.request.wait_time = 0.1;
  	
  	target_width.request.id = "readTargetWidth";
  	target_width.request.item = "g_gripper_width";
  	target_width.request.wait_time = 0.1;
  	
  	
  	
  	if ( client.call(gripper_joint_position)  && client.call(target_width))         
   	if (gripper_joint_position.response.ok  && target_width.response.ok)  
   	{
   	
	
    	  
    	 std::string str2 ("=");
    	 msg1.header.stamp = ros::Time::now();		 
   	 msg1.name = {"finger_joint1"};
   	 msg1.position = {atof(gripper_joint_position.response.value.substr(gripper_joint_position.response.value.find(str2)+1, 5).c_str() ) * 0.001};
   	 msg1.effort  = {atof(gripper_joint_effort.response.value.substr(gripper_joint_effort.response.value.find(str2)+1, 5).c_str())};
   	
   	 msg1.velocity = {0.2};
   	 
   	 
   	 msg2.process_value  = atof(gripper_joint_position.response.value.substr(gripper_joint_position.response.value.find(str2)+1, 5).c_str()) * 0.001;
   	 msg2.set_point = atof(target_width.response.value.substr(target_width.response.value.find(str2)+1, 5).c_str()) * 0.001 ;
   	 //msg2.command  = atof(gripper_joint_effort.response.value.substr(gripper_joint_effort.response.value.length()-n, n).c_str()) ; 
   	 msg2.command  = 20;
   	 msg2.header.stamp = ros::Time::now();
    
    pub_.publish(msg1);
    pub1_.publish(msg2);
  }
}


private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub1_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "gripper_updater");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish gripper_updater;

  ros::spin();

  return 0;
}
