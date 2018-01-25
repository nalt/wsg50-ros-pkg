#include "wsg_50/gripper_action_server.h"

GripperActionServer::GripperActionServer(ros::NodeHandle& node_handle, std::string base_name, GripperCommunication& gripper_com, NodeState& node_state, wsg_50_common::Status& gripper_status):
	node_handle(node_handle),
	gripper_com(gripper_com),
	node_state(node_state),
	gripper_status(gripper_status)
{
	this->base_name = base_name;
	this->action_server_started = false;
	this->action_server = nullptr;
	this->commandSubscriber = nullptr;
	this->has_active_goal = false;
}

void GripperActionServer::start() {
	this->action_server = new ActionServer(
		this->node_handle,
		this->base_name,
		boost::bind(&GripperActionServer::goalCallback, this, _1),
		boost::bind(&GripperActionServer::cancelCallback, this, _1),
		false
	);
	this->action_server->start();
	this->action_server_started = true;
}

void GripperActionServer::shutdown() {
	if (this->action_server_started == true) {
		delete this->action_server;
		this->action_server = nullptr;
	}
}

void GripperActionServer::goalCallback(GoalHandle goal_handle) {
	if (this->gripper_com.acceptsCommands() == false) {
		wsg_50_common::CommandResult result;
		result.status = gripper_status;
		goal_handle.setRejected(result, "Gripper is already processing a command");
	} else if (this->commandSubscriber != nullptr) {
		wsg_50_common::CommandResult result;
		result.status = gripper_status;
		goal_handle.setRejected(result, "The action server is still waiting for a callback of a command send to the gripper");
	} else {
		if (this->has_active_goal == true) {
			ROS_WARN("Aborting current goal. This is not supposed to happen.");
			this->current_goal_handle.setAborted();
		}
		goal_handle.setAccepted("Accept new gripper command");
		this->current_goal_handle = goal_handle;
		this->has_active_goal = true;
		auto goal = goal_handle.getGoal();
		this->handleCommand(goal->command, goal_handle);
	}
}


void GripperActionServer::handleCommand(wsg_50_common::Command command, GoalHandle& goal_handle) {
	switch (command.command_id) {
		case wsg_50_common::Command::MOVE: {
			try {
				this->gripper_com.move(command.width, command.speed, false, [&](msg_t& message){
					this->moveCommandCallback(message);
				});
			} catch (...) {
				wsg_50_common::CommandResult result;
				result.status = gripper_status;
				goal_handle.setAborted(result, "Error while sending the move command to the gripper");
			}
		}
	}
}


void GripperActionServer::moveCommandCallback(msg_t& message) {
	if (this->has_active_goal == true) {
		this->current_goal_handle.setSucceeded();
		this->has_active_goal = false;
	}
}

void GripperActionServer::cancelCallback(GoalHandle goal_handle) {
	printf("CANCEL CALLBACK\n");
}

