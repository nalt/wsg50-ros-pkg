/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
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
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"
#include "wsg_50_common/Cmd.h"

#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"


//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

float increment;
bool objectGraspped;

int g_timer_cnt = 0;
ros::Publisher g_pub_state, g_pub_joint, g_pub_moving;
bool g_ismoving = false, g_use_script = false;
float g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
   
//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------


bool moveSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) ){
  		ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
		res.error = move(req.width, req.speed, false);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		res.error = move(req.width, req.speed, false);
	}

	ROS_INFO("Target position reached.");
  	return true;
}

bool graspSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) ){
  		ROS_INFO("Grasping object at %f mm/s.", req.width, req.speed);
		res.error = grasp(req.width, req.speed);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = grasp(req.width, req.speed);
	}

	ROS_INFO("Object grasped correctly.");
	objectGraspped=true;
  	return true;
}

bool incrementSrv(wsg_50_common::Incr::Request &req, wsg_50_common::Incr::Response &res)
{
	if (req.direction == "open"){
	
		if (!objectGraspped){
		
			float currentWidth = getOpening();
			float nextWidth = currentWidth + req.increment;
			if ( (currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN ){
				//grasp(nextWidth, 1);
				move(nextWidth,20, true);
				currentWidth = nextWidth;
			}else if( nextWidth >= GRIPPER_MAX_OPEN){
				//grasp(GRIPPER_MAX_OPEN, 1);
				move(GRIPPER_MAX_OPEN,1, true);
				currentWidth = GRIPPER_MAX_OPEN;
			}
		}else{
			ROS_INFO("Releasing object...");
			release(GRIPPER_MAX_OPEN, 20);
			objectGraspped = false;
		}
	}else if (req.direction == "close"){
	
		if (!objectGraspped){

			float currentWidth = getOpening();
			float nextWidth = currentWidth - req.increment;
		
			if ( (currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN ){
				//grasp(nextWidth, 1);
				move(nextWidth,20, true);
				currentWidth = nextWidth;
			}else if( nextWidth <= GRIPPER_MIN_OPEN){
				//grasp(GRIPPER_MIN_OPEN, 1);
				move(GRIPPER_MIN_OPEN,1, true);
				currentWidth = GRIPPER_MIN_OPEN;
			}
		}
	}
	return true;
}

bool releaseSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
{
	if ( (req.width >= 0.0 && req.width <= 110.0) && (req.speed > 0.0 && req.speed <= 420.0) ){
  		ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
		res.error = release(req.width, req.speed);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = release(req.width, req.speed);
	}
	ROS_INFO("Object released correctly.");
  	return true;
}

bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_INFO("Homing...");
	homing();
	ROS_INFO("Home position reached.");
	return true;
}

bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_WARN("Stop!");
	stop();
	ROS_WARN("Stopped.");
	return true;
}

bool setAccSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setAcceleration(req.val);
	return true;
}

bool setForceSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
{
	setGraspingForceLimit(req.val);
	return true;
}

bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ack_fault();
	return true;
}


void timer_cb(const ros::TimerEvent& ev)
{
	// ==== Get state values by built-in commands ====
	gripper_response info;
	float acc = 0.0;
	info.speed = 0.0;

	if (!g_use_script) {
		info.state_text = std::string(systemState());
		info.position = getOpening();
		acc = getAcceleration();
		info.f_motor = getForce();//getGraspingForce();

	} else {
		// ==== Call custom measure-and-move command ====
		int res = 0;
		if (!isnan(g_goal_position)) {
			ROS_INFO("Position command: pos=%5.1f, speed=%5.1f", g_goal_position, g_speed);
			res = measure_move(1, g_goal_position, g_speed, info);
		} else if (!isnan(g_goal_speed)) {
			ROS_INFO("Velocity command: speed=%5.1f", g_goal_speed);
			res = measure_move(2, 0, g_goal_speed, info);
		} else
			res = measure_move(0, 0, 0, info);
		if (!isnan(g_goal_position))
			g_goal_position = NAN;
		if (!isnan(g_goal_speed))
			g_goal_speed = NAN;

		if (!res) {
			ROS_ERROR("Measure-and-move command failed");
			return;
		}

		// ==== Moving msg ====
		if (g_ismoving != info.ismoving) {
			std_msgs::Bool moving_msg;
			moving_msg.data = info.ismoving;
			g_pub_moving.publish(moving_msg);
			g_ismoving = info.ismoving;
		}
	}

	// ==== Status msg ====
	wsg_50_common::Status status_msg;
	status_msg.status = info.state_text;
	status_msg.width = info.position;
	status_msg.speed = info.speed;
	status_msg.acc = acc;
	status_msg.force = info.f_motor;
	status_msg.force_finger0 = info.f_finger0;
	status_msg.force_finger1 = info.f_finger1;

	g_pub_state.publish(status_msg);
             

	// ==== Joint state msg ====
	sensor_msgs::JointState joint_states;
	joint_states.header.stamp = ros::Time::now();;
	joint_states.header.frame_id = "wsg_50_gripper_base_link";
		joint_states.name.push_back("wsg_50_gripper_base_joint_gripper_left");
	joint_states.name.push_back("wsg_50_gripper_base_joint_gripper_right");
		joint_states.position.resize(2);

	joint_states.position[0] = -info.position/2000.0;
	joint_states.position[1] = info.position/2000.0;
	joint_states.velocity.resize(2);		
	joint_states.velocity[0] = info.speed;
	joint_states.velocity[1] = info.speed;
	joint_states.effort.resize(2);
	joint_states.effort[0] = info.f_motor;
	joint_states.effort[1] = info.f_motor;
	
	g_pub_joint.publish(joint_states);

	// printf("Timer, last duration: %6.1f\n", ev.profile.last_duration.toSec() * 1000.0);
}

void position_cb(const wsg_50_common::Cmd::ConstPtr& msg)
{ g_speed = msg->speed; g_goal_position = msg->pos; }


void speed_cb(const std_msgs::Float32::ConstPtr& msg)
{ g_goal_speed = msg->data; g_speed = msg->data; }

/**
 * The main function
 */

int main( int argc, char **argv )
{
   ros::init(argc, argv, "wsg_50");

   ros::NodeHandle nh("~");
   std::string ip;
   int port;
   double rate, grasping_force;

   nh.param("ip", ip, std::string("192.168.1.20"));
   nh.param("port", port, 1000);
   nh.param("use_script", g_use_script, false);
   nh.param("rate", rate, 1.0); // With custom script, up to 30Hz are possible
   nh.param("grasping_force", grasping_force, 0.0);

   ROS_INFO("Connecting to %s...", ip.c_str());

   // Connect to device using TCP
   if( cmd_connect_tcp( ip.c_str(), port ) == 0 )
   {
		ROS_INFO("TCP connection stablished");

		// Services
		ros::ServiceServer moveSS = nh.advertiseService("move", moveSrv);
		ros::ServiceServer graspSS = nh.advertiseService("grasp", graspSrv);
		ros::ServiceServer releaseSS = nh.advertiseService("release", releaseSrv);
		ros::ServiceServer homingSS = nh.advertiseService("homing", homingSrv);
		ros::ServiceServer stopSS = nh.advertiseService("stop", stopSrv);
		ros::ServiceServer ackSS = nh.advertiseService("ack", ackSrv);
		ros::ServiceServer incrementSS = nh.advertiseService("move_incrementally", incrementSrv);

		ros::ServiceServer setAccSS = nh.advertiseService("set_acceleration", setAccSrv);
		ros::ServiceServer setForceSS = nh.advertiseService("set_force", setForceSrv);

		// Subscriber
		ros::Subscriber sub_position = nh.subscribe("goal_position", 5, position_cb);
		ros::Subscriber sub_speed = nh.subscribe("goal_speed", 5, speed_cb);

		// Publisher
		g_pub_state = nh.advertise<wsg_50_common::Status>("status", 1000);
		g_pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
		g_pub_moving = nh.advertise<std_msgs::Bool>("moving", 10);

		ROS_INFO("Ready to use, homing now...");
		homing();

		if (grasping_force > 0.0) {
			ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
			setGraspingForceLimit(grasping_force);
		}

		ROS_INFO("Target rate %.1f, using custom script: %s", rate, (g_use_script)? "yes":"no");
		ros::Timer t = nh.createTimer(ros::Duration(1.0/rate), timer_cb);
		ros::spin();

	} else {
		ROS_ERROR("Unable to connect via TCP, please check the port and address used.");
	}

	// Disconnect - won't be executed atm. as the endless loop in test()
	// will never return.
	cmd_disconnect();

	return 0;

}


//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
