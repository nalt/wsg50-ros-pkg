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
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>

#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"
#include "wsg_50/gripper_communication.h"
#include "wsg_50/gripper_action_server.h"
#include "wsg_50/node_state.h"

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
#include <xamla_sysmon_msgs/statuscodes.h>
#include <xamla_sysmon_msgs/HeartBeat.h>
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
ros::Publisher g_pub_state, g_pub_joint, g_pub_moving, g_pub_heartbeat;
bool g_ismoving = false, g_mode_script = false, g_mode_periodic = false,
		g_mode_polling = false;
float g_goal_position = NAN, g_goal_speed = NAN, g_speed = 10.0;
wsg_50_common::Status status_message;
NodeState node_state;
GripperActionServer* action_server = nullptr;

//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------

std::vector<std::string> get_name(XmlRpc::XmlRpcValue controller_list) {
	std::vector<std::string> result { };
	if (controller_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
		std::cout << controller_list << std::endl;
		for (int32_t j = 0; j < controller_list.size(); ++j) {
			std::cout << controller_list[j] << std::endl;

			if (controller_list[j].size() == 4) {
				if (controller_list[j]["name"].getType()
						== XmlRpc::XmlRpcValue::TypeString
						&& controller_list[j]["type"].getType()
								== XmlRpc::XmlRpcValue::TypeString) {
					std::string name =
							static_cast<std::string>(controller_list[j]["name"]);
					std::string action_ns =
							static_cast<std::string>(controller_list[j]["action_ns"]);
					std::string type =
							static_cast<std::string>(controller_list[j]["type"]);
					std::string joint =
							static_cast<std::string>(controller_list[j]["joints"][0]);
					std::cout << "Name : " << name << " -> " << type
							<< std::endl;
					if (type == "WeissGripperCmd") {
						result.push_back(name);
						result.push_back(action_ns);
						result.push_back(type);
						result.push_back(joint);
					}
				}
			}
		}
	}
	return result;
}

//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

bool moveSrv(wsg_50_common::Move::Request &req,
		wsg_50_common::Move::Response &res) {
	if ((req.width >= 0.0 && req.width <= 110.0)
			&& (req.speed > 0.0 && req.speed <= 420.0)) {
		ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
		res.error = move(req.width, req.speed, false);
	} else if (req.width < 0.0 || req.width > 110.0) {
		ROS_ERROR(
				"Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	} else {
		ROS_WARN(
				"Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		res.error = move(req.width, req.speed, false);
	}

	ROS_INFO("Target position reached.");
	return true;
}

bool graspSrv(wsg_50_common::Move::Request &req,
		wsg_50_common::Move::Response &res) {
	if ((req.width >= 0.0 && req.width <= 110.0)
			&& (req.speed > 0.0 && req.speed <= 420.0)) {
		ROS_INFO("Grasping object at %f with %f mm/s.", req.width, req.speed);
		res.error = grasp(req.width, req.speed);
	} else if (req.width < 0.0 || req.width > 110.0) {
		ROS_ERROR(
				"Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	} else {
		ROS_WARN(
				"Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = grasp(req.width, req.speed);
	}

	ROS_INFO("Object grasped correctly.");
	objectGraspped = true;
	return true;
}

bool incrementSrv(wsg_50_common::Incr::Request &req,
		wsg_50_common::Incr::Response &res) {
	if (req.direction == "open") {

		if (!objectGraspped) {

			float currentWidth = getOpening();
			float nextWidth = currentWidth + req.increment;
			if ((currentWidth < GRIPPER_MAX_OPEN)
					&& nextWidth < GRIPPER_MAX_OPEN) {
				//grasp(nextWidth, 1);
				move(nextWidth, 20, true);
				currentWidth = nextWidth;
			} else if (nextWidth >= GRIPPER_MAX_OPEN) {
				//grasp(GRIPPER_MAX_OPEN, 1);
				move(GRIPPER_MAX_OPEN, 1, true);
				currentWidth = GRIPPER_MAX_OPEN;
			}
		} else {
			ROS_INFO("Releasing object...");
			release(GRIPPER_MAX_OPEN, 20);
			objectGraspped = false;
		}
	} else if (req.direction == "close") {

		if (!objectGraspped) {

			float currentWidth = getOpening();
			float nextWidth = currentWidth - req.increment;

			if ((currentWidth > GRIPPER_MIN_OPEN)
					&& nextWidth > GRIPPER_MIN_OPEN) {
				//grasp(nextWidth, 1);
				move(nextWidth, 20, true);
				currentWidth = nextWidth;
			} else if (nextWidth <= GRIPPER_MIN_OPEN) {
				//grasp(GRIPPER_MIN_OPEN, 1);
				move(GRIPPER_MIN_OPEN, 1, true);
				currentWidth = GRIPPER_MIN_OPEN;
			}
		}
	}
	return true;
}

bool releaseSrv(wsg_50_common::Move::Request &req,
		wsg_50_common::Move::Response &res) {
	if ((req.width >= 0.0 && req.width <= 110.0)
			&& (req.speed > 0.0 && req.speed <= 420.0)) {
		ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
		res.error = release(req.width, req.speed);
	} else if (req.width < 0.0 || req.width > 110.0) {
		ROS_ERROR(
				"Imposible to move to this position. (Width values: [0.0 - 110.0] ");
		res.error = 255;
		return false;
	} else {
		ROS_WARN(
				"Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.");
		res.error = release(req.width, req.speed);
	}
	ROS_INFO("Object released correctly.");
	return true;
}

bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
	ROS_INFO("Homing...");
	homing();
	ROS_INFO("Home position reached.");
	return true;
}

bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
	ROS_WARN("Stop!");
	auto& gripperCom = GripperCommunication::Instance();
	try {
		gripperCom.soft_stop();
	} catch (std::runtime_error& ex) {
		ROS_FATAL("Could not send stop command");
		return false;
	}
	ROS_WARN("Stopped.");
	return true;
}

bool setAccSrv(wsg_50_common::Conf::Request &req,
		wsg_50_common::Conf::Response &res) {
	auto& gripperCom = GripperCommunication::Instance();
	try {
		gripperCom.set_acceleration(req.val);
	} catch (std::runtime_error& ex) {
		ROS_ERROR("Could not send acceleration: %s\n", ex.what());
		return false;
	}
	return true;
}

bool setForceSrv(wsg_50_common::Conf::Request &req,
		wsg_50_common::Conf::Response &res) {
	auto& gripperCom = GripperCommunication::Instance();
	try {
		gripperCom.set_force(req.val);
	} catch (std::runtime_error& ex) {
		ROS_FATAL("Could not send grasping force command: %s\n", ex.what());
		return false;
	}
	return true;
}

bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
	auto& gripperCom = GripperCommunication::Instance();
	try {
		gripperCom.acknowledge_error();
	} catch (std::runtime_error& ex) {
		ROS_FATAL("Could not send ack command: %s\n", ex.what());
		return false;
	}
	return true;
}

/** \brief Callback for goal_position topic (in appropriate modes) */
void position_cb(const wsg_50_common::Cmd::ConstPtr& msg) {
	g_speed = msg->speed;
	g_goal_position = msg->pos;
	// timer_cb() will send command to gripper

	if (g_mode_periodic) {
		// Send command to gripper without waiting for a response
		// read_thread() handles responses
		// read/write may be simultaneous, therefore no mutex
		stop(true);
		if (move(g_goal_position, g_speed, false, true) != 0)
			ROS_ERROR("Failed to send MOVE command");
	}
}

/** \brief Callback for goal_speed topic (in appropriate modes) */
void speed_cb(const std_msgs::Float32::ConstPtr& msg) {
	g_goal_speed = msg->data;
	g_speed = msg->data;
	// timer_cb() will send command to gripper
}

/** \brief Loop for state polling in modes script and polling. Also sends command in script mode. */
void timer_cb(const ros::TimerEvent& ev) {
	// ==== Get state values by built-in commands ====
	/*gripper_response info;
	 float acc = 0.0;
	 info.speed = 0.0;

	 if (g_mode_polling) {
	 const char * state = systemState();
	 if (!state)
	 return;
	 info.state_text = std::string(state);
	 info.position = getOpening();
	 acc = getAcceleration();
	 info.f_motor = getForce();        //getGraspingForce();

	 } else if (g_mode_script) {
	 // ==== Call custom measure-and-move command ====
	 int res = 0;
	 if (!std::isnan(g_goal_position)) {
	 ROS_INFO("Position command: pos=%5.1f, speed=%5.1f",
	 g_goal_position, g_speed);
	 res = script_measure_move(1, g_goal_position, g_speed, info);
	 } else if (!std::isnan(g_goal_speed)) {
	 ROS_INFO("Velocity command: speed=%5.1f", g_goal_speed);
	 res = script_measure_move(2, 0, g_goal_speed, info);
	 } else
	 res = script_measure_move(0, 0, 0, info);
	 if (!std::isnan(g_goal_position))
	 g_goal_position = NAN;
	 if (!std::isnan(g_goal_speed))
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
	 } else
	 return;

	 // ==== Status msg ====
	 wsg_50_common::Status status_msg;
	 status_msg.grasping_state = info.state_text;
	 status_msg.width = info.position;
	 status_msg.speed = info.speed;
	 status_msg.acc = acc;
	 status_msg.force = info.f_motor;
	 status_msg.force_finger0 = info.f_finger0;
	 status_msg.force_finger1 = info.f_finger1;

	 g_pub_state.publish(status_msg);

	 // ==== Joint state msg ====
	 // \todo Use name of node for joint names
	 sensor_msgs::JointState joint_states;
	 joint_states.header.stamp = ros::Time::now();
	 ;
	 joint_states.header.frame_id = "wsg50_base_link";
	 joint_states.name.push_back("wsg50_finger_left_joint");
	 joint_states.name.push_back("wsg50_finger_right_joint");
	 joint_states.position.resize(2);

	 joint_states.position[0] = -info.position / 2000.0;
	 joint_states.position[1] = info.position / 2000.0;
	 joint_states.velocity.resize(2);
	 joint_states.velocity[0] = info.speed / 1000.0;
	 joint_states.velocity[1] = info.speed / 1000.0;
	 joint_states.effort.resize(2);
	 joint_states.effort[0] = info.f_motor;
	 joint_states.effort[1] = info.f_motor;

	 g_pub_joint.publish(joint_states);

	 // printf("Timer, last duration: %6.1f\n", ev.profile.last_duration.toSec() * 1000.0);*/
}

/** \brief Reads gripper responses in auto_update mode. The gripper pushes state messages in regular intervals. */
void read_thread(int interval_ms) {
	/**ROS_INFO("Thread started");

	 status_t status;
	 int res;
	 bool pub_state = false;

	 double rate_exp = 1000.0 / (double) interval_ms;
	 std::string names[3] = { "opening", "speed", "force" };

	 // Prepare messages
	 wsg_50_common::Status status_msg;
	 status_msg.grasping_state_id = wsg_50_common::Status::UNKNOWN;
	 status_msg.grasping_state = "UNKNOWN";

	 sensor_msgs::JointState joint_states;
	 joint_states.header.frame_id = "wsg50_base_link";
	 joint_states.name.push_back("wsg50_finger_left_joint");
	 joint_states.name.push_back("wsg50_finger_right_joint");
	 joint_states.position.resize(2);
	 joint_states.velocity.resize(2);
	 joint_states.effort.resize(2);

	 // Request automatic updates (error checking is done below)
	 getOpening(interval_ms);
	 getSpeed(interval_ms);
	 getForce(interval_ms);

	 msg_t msg;
	 msg.id = 0;
	 msg.data = 0;
	 msg.len = 0;
	 int cnt[3] = { 0, 0, 0 };
	 auto time_start = std::chrono::system_clock::now();

	 while (g_mode_periodic) {
	 // Receive gripper response
	 msg_free(&msg);
	 res = msg_receive(&msg);
	 if (res < 0 || msg.len < 2) {
	 ROS_ERROR("Gripper response failure");
	 continue;
	 }

	 float val = 0.0;
	 status = cmd_get_response_status(msg.data);

	 // Decode float for opening/speed/force
	 if (msg.id >= 0x43 && msg.id <= 0x45 && msg.len == 6) {
	 if (status != E_SUCCESS) {
	 ROS_ERROR("Gripper response failure for opening/speed/force\n");
	 continue;
	 }
	 val = convert(&msg.data[2]);
	 }

	 // Handle response types
	 int motion = -1;
	 switch (msg.id) {

	 case 0x43:
	 status_msg.width = val;
	 pub_state = true;
	 cnt[0]++;
	 break;


	 case 0x44:
	 status_msg.speed = val;
	 cnt[1]++;
	 break;


	 case 0x45:
	 status_msg.force = val;
	 cnt[2]++;
	 break;


	 // Move commands are sent from outside this thread
	 case 0x21:
	 if (status == E_SUCCESS) {
	 ROS_INFO("Position reached");
	 motion = 0;
	 } else if (status == E_AXIS_BLOCKED) {
	 ROS_INFO("Axis blocked");
	 motion = 0;
	 } else if (status == E_CMD_PENDING) {
	 ROS_INFO("Movement started");
	 motion = 1;
	 } else if (status == E_ALREADY_RUNNING) {
	 ROS_INFO("Movement error: already running");
	 } else if (status == E_CMD_ABORTED) {
	 ROS_INFO("Movement aborted");
	 motion = 0;
	 } else {
	 ROS_INFO("Movement error");
	 motion = 0;
	 }
	 break;


	 // Stop commands are sent from outside this thread
	 case 0x22:
	 // Stop command; nothing to do
	 break;
	 default:
	 ROS_INFO("Received unknown respone 0x%02x (%2dB)\n", msg.id,
	 msg.len);
	 }

	 // ***** PUBLISH motion message
	 if (motion == 0 || motion == 1) {
	 std_msgs::Bool moving_msg;
	 moving_msg.data = motion;
	 g_pub_moving.publish(moving_msg);
	 g_ismoving = motion;
	 }

	 // ***** PUBLISH state message & joint message
	 if (pub_state) {
	 pub_state = false;
	 g_pub_state.publish(status_msg);

	 joint_states.header.stamp = ros::Time::now();
	 ;
	 joint_states.position[0] = -status_msg.width / 2000.0;
	 joint_states.position[1] = status_msg.width / 2000.0;
	 joint_states.velocity[0] = status_msg.speed / 1000.0;
	 joint_states.velocity[1] = status_msg.speed / 1000.0;
	 joint_states.effort[0] = status_msg.force;
	 joint_states.effort[1] = status_msg.force;
	 g_pub_joint.publish(joint_states);
	 }

	 // Check # of received messages regularly
	 std::chrono::duration<float> t = std::chrono::system_clock::now()
	 - time_start;
	 double t_ = t.count();
	 if (t_ > 5.0) {
	 time_start = std::chrono::system_clock::now();
	 //printf("Infos for %5.1fHz, %5.1fHz, %5.1fHz\n", (double)cnt[0]/t_, (double)cnt[1]/t_, (double)cnt[2]/t_);

	 std::string info = "Rates for ";
	 for (int i = 0; i < 3; i++) {
	 double rate_is = (double) cnt[i] / t_;
	 info += names[i] + ": " + std::to_string((int) rate_is)
	 + "Hz, ";
	 if (rate_is == 0.0)
	 ROS_ERROR("Did not receive data for %s", names[i].c_str());
	 }
	 ROS_DEBUG_STREAM(
	 (info + " expected: " + std::to_string((int )rate_exp)
	 + "Hz").c_str());
	 cnt[0] = 0;
	 cnt[1] = 0;
	 cnt[2] = 0;
	 }

	 }

	 // Disable automatic updates
	 // TODO: The functions will receive an unexpected response
	 getOpening(0);
	 getSpeed(0);
	 getForce(0);

	 ROS_INFO("Thread ended");*/
}

void sigint_handler(int sig) {
	ROS_INFO("Exiting...");
	g_mode_periodic = false;
	g_mode_script = false;
	g_mode_polling = false;
	ros::shutdown();
}

void initialize_cb() {

}

void nominal_cb() {

}

void status_cb(msg_t& message) {
	if (message.len > 0) {
		auto status = (status_t) make_short(message.data[0], message.data[1]);
		if (status == E_SUCCESS) {
			auto s = getStateValues(message.data);
			printf(
					"-- Status Callback: id: %d, len: %d, status: %d, text: %s\n",
					message.id, message.len, status, s);
		}
	}
}

void loop_cb(const ros::TimerEvent& ev) {
	auto& gripperCom = GripperCommunication::Instance();
	try {
		gripperCom.processMessages();
	} catch (std::runtime_error& ex) {
		ROS_ERROR("An error occured while trying to receive messages: %s",
				ex.what());
	}

	xamla_sysmon_msgs::HeartBeat heartbeat_msg;

	auto gripperState = gripperCom.getState();

	switch (node_state.get()) {
	case (NodeStateType::StartingGripValues): {
		if (gripperCom.lastCommandReturnedSuccess(
				(unsigned char) WellKnownMessageId::GRIPPING_STATE) == true) {
			try {
				gripperCom.activateOpeningValueUpdates(100);
				node_state.set(NodeStateType::StartingOpeningValues);
			} catch (...) {
				node_state.set(NodeStateType::Error);
			}
		}
		break;
	}
	case (NodeStateType::StartingOpeningValues): {
		if (gripperCom.lastCommandReturnedSuccess(
				(unsigned char) WellKnownMessageId::OPENING_VALUES) == true) {
			ROS_INFO("Gripper state is nominal");
			node_state.set(NodeStateType::Nominal);
		}
		break;
	}
	case (NodeStateType::Nominal): {
		action_server->doWork();

		heartbeat_msg.header.stamp = ros::Time::now();
		if ((gripperState.grasping_state == wsg_50_common::Status::UNKNOWN)
				|| (gripperState.grasping_state == wsg_50_common::Status::ERROR)) {
			heartbeat_msg.status =
					static_cast<int>(TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR);
			heartbeat_msg.details = "Gripper is in unknown or error state.";
		} else {
			heartbeat_msg.status =
					static_cast<int>(TopicHeartbeatStatus::TopicCode::GO);
			heartbeat_msg.details = "";
		}

		break;
	}
	default: {
		ROS_ERROR(
				"Something went very wrong. This can only be recoverd by restarting the node");
		heartbeat_msg.status =
				static_cast<int>(TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR);
		heartbeat_msg.details = "Drive is in error state.";
		break;
	}
	}

	gripperCom.requestValueUpdate(
			(unsigned char) WellKnownMessageId::FORCE_VALUES);
	gripperCom.requestValueUpdate(
			(unsigned char) WellKnownMessageId::SPEED_VALUES);

	// publish current state
	status_message.grasping_state_id = gripperState.grasping_state;
	status_message.width = gripperState.width;
	status_message.current_force = gripperState.force;
	status_message.current_speed = gripperState.speed;
	g_pub_state.publish(status_message);

	g_pub_heartbeat.publish(heartbeat_msg);

	sensor_msgs::JointState joint_states;
	joint_states.header.frame_id = "wsg50_base_link";
	joint_states.name.push_back("wsg50_finger_left_joint");
	joint_states.name.push_back("wsg50_finger_right_joint");
	joint_states.position.resize(2);
	joint_states.velocity.resize(2);
	joint_states.effort.resize(2);

	joint_states.header.stamp = ros::Time::now();
	joint_states.position[0] = -gripperState.width / 2000.0;
	joint_states.position[1] = gripperState.width / 2000.0;
	joint_states.velocity[0] = gripperState.speed / 1000.0;
	joint_states.velocity[1] = gripperState.speed / 1000.0;
	joint_states.effort[0] = gripperState.force;
	joint_states.effort[1] = gripperState.force;
	g_pub_joint.publish(joint_states);
}

/**
 * The main function
 */

int main(int argc, char **argv) {
	auto& gripperCom = GripperCommunication::Instance();

	ros::init(argc, argv, "wsg_50");
	ros::NodeHandle nh("~");
	signal(SIGINT, sigint_handler);

	std::string ip, protocol, com_mode;
	int port, local_port;
	double rate, grasping_force;
	bool use_udp = false;

	nh.param("ip", ip, std::string("192.168.1.20"));
	nh.param("port", port, 1000);
	nh.param("local_port", local_port, 1501);
	nh.param("protocol", protocol, std::string(""));
	nh.param("com_mode", com_mode, std::string(""));
	nh.param("rate", rate, 1.0); // With custom script, up to 30Hz are possible
	nh.param("grasping_force", grasping_force, 0.0);

	if (protocol == "udp")
		use_udp = true;
	else
		protocol = "tcp";
	if (com_mode == "script")
		g_mode_script = true;
	else if (com_mode == "auto_update")
		g_mode_periodic = true;
	else {
		com_mode = "polling";
		g_mode_polling = true;
	}

	ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(),
			port, protocol.c_str(), com_mode.c_str());

	// Connect to device using TCP/USP
	int res_con;
	if (!use_udp)
		res_con = cmd_connect_tcp(ip.c_str(), port);
	else
		res_con = cmd_connect_udp(local_port, ip.c_str(), port);

	if (res_con == 0) {
		ros::Duration(1).sleep();
		printf("Register callback\n");

		printf("Create message processing timer\n");
		auto tmr = nh.createTimer(ros::Duration(0.005), loop_cb);

		XmlRpc::XmlRpcValue tmp_list;
		nh.getParam("controller_list", tmp_list);
		std::vector<std::string> result = get_name(tmp_list);
		if (result.empty()) {
			ROS_ERROR(
					"Could not find controller_list. This error is not recoverable.");
		} else {
			std::string controller_name = result[0];

			// Open publishers
			g_pub_state = nh.advertise<wsg_50_common::Status>(
					controller_name + "/status", 1000);
			g_pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_states",
					10);
			g_pub_heartbeat = nh.advertise<xamla_sysmon_msgs::HeartBeat>(
					controller_name + "/heartbeat", 1);

			xamla_sysmon_msgs::HeartBeat msg;
			msg.header.stamp = ros::Time::now();

			msg.status =
					static_cast<int>(TopicHeartbeatStatus::TopicCode::STARTING);
			msg.details = TopicHeartbeatStatus::generateMessageText(
					TopicHeartbeatStatus::intToStatusCode(msg.status));
			g_pub_heartbeat.publish(msg);

			// Services
			ros::ServiceServer setAccSS, setForceSS, stopSS, ackSS;
			setAccSS = nh.advertiseService("set_acceleration", setAccSrv);
			setForceSS = nh.advertiseService("set_force", setForceSrv);
			stopSS = nh.advertiseService("soft_stop", stopSrv);
			ackSS = nh.advertiseService("acknowledge_error", ackSrv);

			// Open action server
			action_server = new GripperActionServer(nh,
					controller_name + "/command", gripperCom, node_state);
			action_server->start();

			try {
				gripperCom.subscribe(
						(unsigned char) WellKnownMessageId::FORCE_VALUES,
						[&](msg_t& m) {gripperCom.forceCallback(m);});
				gripperCom.subscribe(
						(unsigned char) WellKnownMessageId::SPEED_VALUES,
						[&](msg_t& m) {gripperCom.speedCallback(m);});
				gripperCom.activateGripStateUpdates(100);
				gripperCom.activateOpeningValueUpdates(105);
				printf("Spinning\n");
				node_state.set(NodeStateType::Nominal);

				// Start ROS loop
				ros::spin();
			} catch (const MessageQueueFull& e) {
				printf("queue full\n");
			} catch (const MessageSendFailed& e) {
				printf("send failed\n");
			} catch (const std::runtime_error& re) {
				ROS_ERROR(
						"Could not request grip state updates from gripper. This error is not recoverable.");
				printf("%s\n", re.what());
			}

			status_message.grasping_state_id = wsg_50_common::Status::UNKNOWN;
			g_pub_state.publish(status_message);

			action_server->shutdown();
			delete action_server;
			action_server = nullptr;
			setAccSS.shutdown();
			setForceSS.shutdown();
			stopSS.shutdown();
			ackSS.shutdown();
			g_pub_state.shutdown();
			g_pub_joint.shutdown();
		}
	} else {
		ROS_ERROR(
				"Could not open connection to gripper. This error is not recoverable.");
	}

	ROS_INFO("Exiting...");

	g_mode_periodic = false;
	g_mode_script = false;
	g_mode_polling = false;
	sleep(1);
	cmd_disconnect();

	return 0;
}

/*int main( int argc, char **argv )
 {
 ros::init(argc, argv, "wsg_50");
 ros::NodeHandle nh("~");
 signal(SIGINT, sigint_handler);

 std::string ip, protocol, com_mode;
 int port, local_port;
 double rate, grasping_force;
 bool use_udp = false;

 nh.param("ip", ip, std::string("192.168.1.20"));
 nh.param("port", port, 1000);
 nh.param("local_port", local_port, 1501);
 nh.param("protocol", protocol, std::string(""));
 nh.param("com_mode", com_mode, std::string(""));
 nh.param("rate", rate, 1.0); // With custom script, up to 30Hz are possible
 nh.param("grasping_force", grasping_force, 0.0);

 if (protocol == "udp")
 use_udp = true;
 else
 protocol = "tcp";
 if (com_mode == "script")
 g_mode_script = true;
 else if (com_mode == "auto_update")
 g_mode_periodic = true;
 else {
 com_mode = "polling";
 g_mode_polling = true;
 }

 ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(), port, protocol.c_str(), com_mode.c_str());

 // Connect to device using TCP/USP
 int res_con;
 if (!use_udp)
 res_con = cmd_connect_tcp( ip.c_str(), port );
 else
 res_con = cmd_connect_udp(local_port, ip.c_str(), port );

 if (res_con == 0 ) {
 ROS_INFO("Gripper connection stablished");

 // Services
 ros::ServiceServer moveSS, graspSS, releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS, setForceSS;

 if (g_mode_script || g_mode_polling) {
 moveSS = nh.advertiseService("move", moveSrv);
 graspSS = nh.advertiseService("grasp", graspSrv);
 releaseSS = nh.advertiseService("release", releaseSrv);
 homingSS = nh.advertiseService("homing", homingSrv);
 stopSS = nh.advertiseService("stop", stopSrv);
 ackSS = nh.advertiseService("ack", ackSrv);
 incrementSS = nh.advertiseService("move_incrementally", incrementSrv);

 setAccSS = nh.advertiseService("set_acceleration", setAccSrv);
 setForceSS = nh.advertiseService("set_force", setForceSrv);
 }

 // Subscriber
 ros::Subscriber sub_position, sub_speed;
 if (g_mode_script || g_mode_periodic)
 sub_position = nh.subscribe("goal_position", 5, position_cb);
 if (g_mode_script)
 sub_speed = nh.subscribe("goal_speed", 5, speed_cb);

 // Publisher
 g_pub_state = nh.advertise<wsg_50_common::Status>("status", 1000);
 g_pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
 if (g_mode_script || g_mode_periodic)
 g_pub_moving = nh.advertise<std_msgs::Bool>("moving", 10);

 ROS_INFO("Ready to use. Homing and taring now...");
 homing();
 ros::Duration(0.5).sleep();
 doTare();

 if (grasping_force > 0.0) {
 ROS_INFO("Setting grasping force limit to %5.1f", grasping_force);
 setGraspingForceLimit(grasping_force);
 }

 ROS_INFO("Init done. Starting timer/thread with target rate %.1f.", rate);
 std::thread th;
 ros::Timer tmr;
 if (g_mode_polling || g_mode_script)
 tmr = nh.createTimer(ros::Duration(1.0/rate), timer_cb);
 if (g_mode_periodic)
 th = std::thread(read_thread, (int)(1000.0/rate));

 ros::spin();

 } else {
 ROS_ERROR("Unable to connect, please check the port and address used.");
 }

 ROS_INFO("Exiting...");
 g_mode_periodic = false;
 g_mode_script = false;
 g_mode_polling = false;
 sleep(1);
 cmd_disconnect();

 return 0;

 }*/

//------------------------------------------------------------------------
// Testing functions
//------------------------------------------------------------------------
