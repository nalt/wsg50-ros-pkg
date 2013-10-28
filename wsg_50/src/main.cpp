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
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include "std_srvs/Empty.h"
#include "wsg_50_common/Conf.h"
#include "wsg_50_common/Incr.h"

#include "sensor_msgs/JointState.h"


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
		res.error = move(req.width, req.speed);
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	}else{
	        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		res.error = move(req.width, req.speed);
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
				move(nextWidth,20);
				currentWidth = nextWidth;
			}else if( nextWidth >= GRIPPER_MAX_OPEN){
				//grasp(GRIPPER_MAX_OPEN, 1);
				move(GRIPPER_MAX_OPEN,1);
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
				move(nextWidth,20);
				currentWidth = nextWidth;
			}else if( nextWidth <= GRIPPER_MIN_OPEN){
				//grasp(GRIPPER_MIN_OPEN, 1);
				move(GRIPPER_MIN_OPEN,1);
				currentWidth = GRIPPER_MIN_OPEN;
			}
		}
	}
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



/**
 * The main function
 */

int main( int argc, char **argv )
{
   ros::init(argc, argv, "wsg_50");

   ros::NodeHandle nh;
   std::string ip;
   int port;

   ROS_INFO("WSG 50 - ROS NODE");

   nh.param("wsg_50_tcp/ip", ip, std::string("192.168.1.20"));
   nh.param("wsg_50_tcp/port", port, 1000);

   // Connect to device using TCP
   if( cmd_connect_tcp( ip.c_str(), port ) == 0 )
   {

	ROS_INFO("TCP connection stablished");

	// Services
  	ros::ServiceServer moveSS = nh.advertiseService("wsg_50/move", moveSrv);
  	ros::ServiceServer graspSS = nh.advertiseService("wsg_50/grasp", graspSrv);
  	ros::ServiceServer releaseSS = nh.advertiseService("wsg_50/release", releaseSrv);
  	ros::ServiceServer homingSS = nh.advertiseService("wsg_50/homing", homingSrv);
  	ros::ServiceServer stopSS = nh.advertiseService("wsg_50/stop", stopSrv);
  	ros::ServiceServer ackSS = nh.advertiseService("wsg_50/ack", ackSrv);
	
	ros::ServiceServer incrementSS = nh.advertiseService("wsg_50/move_incrementally", incrementSrv);

	ros::ServiceServer setAccSS = nh.advertiseService("wsg_50/set_acceleration", setAccSrv);
	ros::ServiceServer setForceSS = nh.advertiseService("wsg_50/set_force", setForceSrv);

	// Publisher
  	ros::Publisher state_pub = nh.advertise<wsg_50_common::Status>("wsg_50/status", 1000);
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

	ROS_INFO("Ready to use.");

	homing();

	ros::Rate loop_rate(1); // loop at 1Hz

	while( ros::ok() ){

		//Loop waiting for orders and updating the state
		
		//Create the msg to send
		wsg_50_common::Status status_msg;		

		//Get state values
		const char * aux;
		aux = systemState();
		int op = getOpening();
		int acc = getAcceleration();
		int force = getGraspingForceLimit();

    	std::stringstream ss;
		
		ss << aux;

		status_msg.status = ss.str();
		status_msg.width = op;
		status_msg.acc = acc;
		status_msg.force = force;

		 state_pub.publish(status_msg);
		             
	  sensor_msgs::JointState joint_states;

    joint_states.header.stamp = ros::Time::now();;
 	  joint_states.header.frame_id = "wsg_50_gripper_base_link";
 
		joint_states.name.push_back("wsg_50_gripper_base_joint_gripper_left");
		joint_states.name.push_back("wsg_50_gripper_base_joint_gripper_right");

 		joint_states.position.resize(2);
		joint_states.position[0] = -op/2000.0;
		joint_states.position[1] = op/2000.0;

		//joint_states.velocity.resize(2);		
		//joint_states.velocity[0];
		//joint_states.velocity[1];
 		
		joint_states.effort.resize(2);
		joint_states.effort[0] = force;
		joint_states.effort[1] = force;
		
		joint_states_pub.publish(joint_states);
		loop_rate.sleep();
		ros::spinOnce();
	}

   }else{

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
