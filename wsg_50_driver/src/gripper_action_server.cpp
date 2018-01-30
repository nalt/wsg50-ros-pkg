#include "wsg_50/gripper_action_server.h"
#define NO_EXPECTATION -10

GripperActionServer::GripperActionServer(ros::NodeHandle& node_handle,
		std::string base_name, GripperCommunication& gripper_com,
		NodeState& node_state) :
		node_handle(node_handle), gripper_com(gripper_com), node_state(
				node_state) {
	this->base_name = base_name;
	this->action_server_started = false;
	this->action_server = nullptr;
	this->action_state.state = ActionStateCode::NO_GOAL;
	this->action_state.return_code = -1;
	this->action_state.expected_grasping_state = NO_EXPECTATION;

	grasping_state_subscription = this->gripper_com.subscribe(
			(unsigned char) WellKnownMessageId::GRIPPING_STATE,
			[&](msg_t& message) {this->graspingStateCallback(message);});
	soft_stop_subscription = this->gripper_com.subscribe(
			(unsigned char) WellKnownMessageId::SOFT_STOP,
			[&](msg_t& message) {this->stopCallback(message);});
	emergency_stop_subscription = this->gripper_com.subscribe(
			(unsigned char) WellKnownMessageId::EMERGENCY_STOP,
			[&](msg_t& message) {this->stopCallback(message);});
}

void GripperActionServer::doWork() {
	if (this->action_state.state == ActionStateCode::DONE) {
		wsg_50_common::CommandResult result;
		result.status = this->fillStatus();
		result.status.return_code = this->action_state.return_code;
		if (result.status.return_code == E_SUCCESS) {
			if (this->action_state.expected_grasping_state == NO_EXPECTATION) {
				this->current_goal_handle.setSucceeded(result, "Goal reached");
			} else {
				if (this->action_state.expected_grasping_state == result.status.grasping_state_id) {
					this->current_goal_handle.setSucceeded(result, "Goal reached");
				} else {
					this->current_goal_handle.setAborted(result, "Goal aborted, wrong grasping state");
				}
			}
		} else {
			this->current_goal_handle.setAborted(result, "Goal aborted, command did not return success code");
		}

		this->action_state.state = ActionStateCode::NO_GOAL;
		this->action_state.expected_grasping_state = NO_EXPECTATION;
		this->action_state.return_code = -1;
	}
}

void GripperActionServer::start() {
	this->action_server = new ActionServer(this->node_handle, this->base_name,
			boost::bind(&GripperActionServer::goalCallback, this, _1),
			boost::bind(&GripperActionServer::cancelCallback, this, _1), false);
	this->action_server->start();
	this->action_server_started = true;
}

void GripperActionServer::shutdown() {
	if (this->action_server_started == true) {
		this->gripper_com.unregisterListener(soft_stop_subscription.messageId,
				soft_stop_subscription.listenerId);
		this->gripper_com.unregisterListener(emergency_stop_subscription.messageId,
				emergency_stop_subscription.listenerId);
		this->gripper_com.unregisterListener(grasping_state_subscription.messageId,
				grasping_state_subscription.listenerId);
		delete this->action_server;
		this->action_server = nullptr;
	}
}

void GripperActionServer::goalCallback(GoalHandle goal_handle) {
	auto goal = goal_handle.getGoal();

	if (this->gripper_com.acceptsCommands() == false) {
		wsg_50_common::CommandResult result;
		result.status = this->fillStatus();
		goal_handle.setRejected(result,
				"Gripper is already processing a command");
	} else {
		if (this->action_state.state != ActionStateCode::NO_GOAL) {
			ROS_WARN("Aborting current goal. This is not supposed to happen.");
			this->current_goal_handle.setAborted();
		}
		goal_handle.setAccepted("Accept new gripper command");
		ROS_INFO("Accepted goal");

		this->current_goal_handle = goal_handle;
		this->action_state.state = ActionStateCode::AWAIT_COMMAND;
		this->action_state.expected_grasping_state = NO_EXPECTATION;
		this->handleCommand(goal->command, goal_handle);
	}
}

void GripperActionServer::handleCommand(wsg_50_common::Command command,
		GoalHandle& goal_handle) {
	printf("handle command\n");
	switch (command.command_id) {
	case (wsg_50_common::Command::MOVE): {
		try {
			/*printf("Create command queue\n");
			auto message = this->gripper_com.createMoveMessage(command.width, command.speed, command.stop_on_block);
			this->action_state.message_queue.push(message);
			this->executeNextCommand();*/
			this->action_state.expected_grasping_state = wsg_50_common::Status::IDLE;
			this->gripper_com.move(command.width, command.speed, command.stop_on_block,
								[&](msg_t& message) {
									this->commandCallback(message);
								});
		} catch (std::runtime_error& ex) {
			wsg_50_common::CommandResult result;
			result.status = this->fillStatus();
			printf("Error while sending gripper command: %s\n", ex.what());
			goal_handle.setAborted(result,
					"Error while sending the move command to the gripper");
		}
		break;
	}
	case (wsg_50_common::Command::GRASP): {
		try {
			this->action_state.expected_grasping_state = wsg_50_common::Status::HOLDING;
			this->gripper_com.grasp(command.width, command.speed,
					[&](msg_t& message) {
						this->commandCallback(message);
					});
		} catch (...) {
			wsg_50_common::CommandResult result;
			result.status = this->fillStatus();
			goal_handle.setAborted(result,
					"Error while sending the move command to the gripper");
		}
		break;
	}
	case (wsg_50_common::Command::RELEASE): {
		try {
			this->action_state.expected_grasping_state = wsg_50_common::Status::IDLE;
			this->gripper_com.release(command.width, command.speed,
					[&](msg_t& message) {
						this->commandCallback(message);
					});
		} catch (...) {
			wsg_50_common::CommandResult result;
			result.status = this->fillStatus();
			goal_handle.setAborted(result,
					"Error while sending the move command to the gripper");
		}
		break;
	}
	case (wsg_50_common::Command::HOMING): {
		try {
			this->gripper_com.homing(
					[&](msg_t& message) {
						this->commandCallback(message);
					});
		} catch (...) {
			wsg_50_common::CommandResult result;
			result.status = this->fillStatus();
			goal_handle.setAborted(result,
					"Error while sending the move command to the gripper");
		}
		break;
	}
	}
}

void GripperActionServer::commandCallback(msg_t& message) {
	if (this->action_state.state == ActionStateCode::AWAIT_COMMAND) {
		this->action_state.return_code = this->gripper_com.decodeStatus(
				message);
		this->action_state.state = ActionStateCode::AWAIT_STATUS_UPDATE;
	}
}

void GripperActionServer::stopCallback(msg_t& message) {
	if (this->action_state.state != ActionStateCode::NO_GOAL) {
		ROS_WARN("Stop received. Aborting goal");
		wsg_50_common::CommandResult result;
		result.status = this->fillStatus();
		this->current_goal_handle.setAborted(result,
				"Received stop");
		this->action_state.state = ActionStateCode::NO_GOAL;
	}
}

void GripperActionServer::cancelCallback(GoalHandle goal_handle) {
	printf("CANCEL CALLBACK\n");
}

wsg_50_common::Status GripperActionServer::fillStatus() {
	wsg_50_common::Status status;
	auto gripperState = this->gripper_com.getState();
	status.grasping_state_id = gripperState.grasping_state;
	status.width = gripperState.width;

	return status;
}

void GripperActionServer::graspingStateCallback(msg_t& message) {
	if (this->action_state.state == ActionStateCode::AWAIT_STATUS_UPDATE) {
		this->action_state.state = ActionStateCode::DONE;
	}
}

