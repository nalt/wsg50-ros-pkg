/* wsg_50_sim_driver
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
 * \brief WSG-50 sim driver.
 */

#include <ros/ros.h>
#include <wsg_50_simulation/Move.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

ros::Publisher vel_pub_r_, vel_pub_l_;
int tPos;

bool moveSrv(wsg_50_simulation::Move::Request &req, wsg_50_simulation::Move::Response &res)
{
	if ( req.width >= 0.0 && req.width <= 110.0 ){
  		ROS_INFO("Moving to %f position.", req.width);
  	
		double open = req.width / 2;
		
		std_msgs::Float64 lCommand, rCommand;
		
		rCommand.data = open/1000;
		lCommand.data = rCommand.data * -1.0;
		
		vel_pub_r_.publish(rCommand);
		vel_pub_l_.publish(lCommand);
		
		
	}else if (req.width < 0.0 || req.width > 110.0){
		ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		//res.error = 255;
		return false;
	}

	ROS_INFO("Target position reached.");
  	return true;
}


bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_INFO("Homing...");
	
	std_msgs::Float64 lCommand, rCommand;
	
	rCommand.data = lCommand.data = 0.0;
	
	vel_pub_r_.publish(rCommand);
	vel_pub_l_.publish(lCommand);
	
	ROS_INFO("Home position reached.");
	return true;
}


int main(int argc, char** argv){
	
	ros::init(argc, argv, "wsg_50_sim_driver");

	ros::NodeHandle nh;
	tPos = 0;
	
	ros::ServiceServer moveSS = nh.advertiseService("wsg_50_sim_driver/move", moveSrv);
	ros::ServiceServer homingSS = nh.advertiseService("wsg_50_sim_driver/homing", homingSrv);
	vel_pub_r_ = nh.advertise<std_msgs::Float64>("/wsg_50_gr/command", 1000);
    	vel_pub_l_ = nh.advertise<std_msgs::Float64>("/wsg_50_gl/command", 1000);
	
	ros::spin();
	
} 
