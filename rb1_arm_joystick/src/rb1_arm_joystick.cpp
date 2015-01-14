/*
 * rb1_arm_joystick
 * Copyright (c) 2015, Robotnik Automation, SLL
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
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
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
 *
 * \author Robotnik Automation SLL
 * \brief Allows to use a pad with the terabot_robot_control node
 */


#include <ros/ros.h>
#include <rb1_arm_joystick/ArmRefRb1.h>         
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define DEFAULT_NUM_OF_BUTTONS		20
//#define ARM							0
//#define GRIPPER						1

#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y       0
#define DEFAULT_AXIS_LINEAR_Z		3
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0

class RB1ArmJoystick
{
	public:
	RB1ArmJoystick();

	private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	//! It will publish into command velocity (for the robot)
	ros::Publisher arm_ref_pub_, gripper_ref_pub_;
	//! It will be suscribed to the joystick
	ros::Subscriber joy_sub_;

	//! Number of the button actions
	int button_up_, button_down_;
	int button_select_;
	int button_fold_;
    int button_open_, button_close_;
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	//! buttons to the arm
	int dead_man_arm_;
	//! current connection (0: arm, 1: gripper)
	short deviceConnected;
	//! Service to move the gripper/arm
	ros::ServiceClient gripper_setOperationMode_client;
	ros::ServiceClient gripper_move_client;
	ros::ServiceClient gripper_move_incr_client;
	ros::ServiceClient gripper_close_client;
	
	ros::ServiceClient arm_fold_client;

	int linear_x_, linear_y_, linear_z_, angular_;
	double l_scale_, a_scale_;
	std::string topic_joy;
};


RB1ArmJoystick::RB1ArmJoystick(){


	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

	//GENERIC BUTTONS
	nh_.param("button_up", button_up_, button_up_);  				// Triangle PS3
	nh_.param("button_down", button_down_, button_down_); 	    	// Cross PS3
	nh_.param("button_select", button_select_, button_select_);		// Select PS3
	nh_.param("button_fold", button_fold_, button_fold_);			// Start PS3
	nh_.param("button_open", button_open_, button_open_);			// Circle PS3
	nh_.param("button_close", button_close_, button_close_);	    // Square PS3
	
	nh_.param<std::string>("rb1_arm_joy", topic_joy, "joy");	    
	
	ROS_INFO("RB1ArmJoystick: joy_topic = %s", topic_joy.c_str());

	// ARM CONF
	nh_.param("dead_man_arm", dead_man_arm_, dead_man_arm_);					// R2 PS3

    // JOY AXIS DEFINITION AND SCALING 
	nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
    nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
    nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Y);
	nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);

	//bRegisteredButtonEvent = new bool(num_of_buttons_);
	for(int i = 0; i < DEFAULT_NUM_OF_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}

 	 // Listen through the node handle sensor_msgs::Joy messages from joystick
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RB1ArmJoystick::joyCallback, this);

	// Request service to send commands to the arm
	arm_fold_client = nh_.serviceClient<std_srvs::Empty>("/rb1_arm_robot_control/fold");

	// Publishes into the arm controller
	arm_ref_pub_ = nh_.advertise<rb1_arm_joystick::ArmRefRb1>("/rb1_arm/ref_cmd", 1);
	
    // Publishes into the arm controller
	gripper_ref_pub_ = nh_.advertise<std_msgs::Int32>("/rb1_arm/gripper", 1);
}

void RB1ArmJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	rb1_arm_joystick::ArmRefRb1 arm;
	geometry_msgs::Twist ref;
	int32_t gripper_ref = 0;
	bool gripper_event = false;
	
	// ARM MOVEMENTS
	// Actions dependant on arm dead-man button
	if (joy->buttons[dead_man_arm_] == 1){
		
		if (joy->buttons[button_up_] == 1){
			if(!bRegisteredButtonEvent[button_up_]){
				bRegisteredButtonEvent[button_up_] = true;
				arm.joint_selection = 1;
					
			}
		}else if (joy->buttons[button_down_] == 1){
			if(!bRegisteredButtonEvent[button_down_]){
				bRegisteredButtonEvent[button_down_] = true;
				arm.joint_selection = -1;
			}
		
		// Used to change between different robot operation modes (jbj, cartesian-euler and trajectory)
		}else if (joy->buttons[button_select_] == 1){
			if(!bRegisteredButtonEvent[button_select_]){
				bRegisteredButtonEvent[button_select_] = true;
				arm.select_reference = 1;					
			}
					
		// Allow to fold the arm throught the pad	
		}else if (joy->buttons[button_fold_] == 1){
			if(!bRegisteredButtonEvent[button_fold_]){
				bRegisteredButtonEvent[button_fold_] = true;
				std_srvs::Empty srv;
				arm_fold_client.call(srv);		
			}
		}else{
			
			// Arm linear and angular references
			ref.linear.x = l_scale_ * joy->axes[DEFAULT_AXIS_LINEAR_X];
			ref.linear.y = -l_scale_ * joy->axes[DEFAULT_AXIS_LINEAR_Y];
			ref.linear.z = -l_scale_ * joy->axes[DEFAULT_AXIS_LINEAR_Z];
			
			if (joy->axes[4]!=0.0) 
				ref.angular.x = a_scale_ * joy->axes[4];	
			if (joy->axes[6]!=0.0)
				ref.angular.x = -a_scale_ * joy->axes[6];	
			if (joy->axes[5]!=0.0) 
				ref.angular.y = -a_scale_ * joy->axes[5];	
			if (joy->axes[7]!=0.0)
				ref.angular.y = a_scale_ * joy->axes[7];	

            // Gripper speed reference
            if (joy->axes[button_close_]!=0.0) {
                 gripper_ref = (int) -(joy->axes[button_close_] * 100.0);
                 gripper_event = true;
            }else if (joy->axes[button_open_]!=0.0){
                 gripper_ref = (int) (joy->axes[button_open_] * 100.0);
                 gripper_event = true;
			 }
						
			bRegisteredButtonEvent[button_up_] = false;
			bRegisteredButtonEvent[button_down_] = false;
			bRegisteredButtonEvent[button_select_] = false;
			bRegisteredButtonEvent[button_fold_] = false;
		}
	}
	
	// Publish gripper msg
	if(gripper_event){
		std_msgs::Int32 msg;
		msg.data = gripper_ref; 
		gripper_ref_pub_.publish(msg);									
	}
	// Publish arm msg
	arm.cmd_vel = ref;
	arm_ref_pub_.publish(arm);	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rb1_arm_joystick");
	RB1ArmJoystick rb1_arm_joystick;
	ros::spin();
}

