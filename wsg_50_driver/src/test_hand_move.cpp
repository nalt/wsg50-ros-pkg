#include <ros/ros.h>
#include "wsg_50/hand.h"

/**
 * @brief read_hand_params: Reads hand parameters from parameter server
 * @param nh: Node handle used for accessing server
 * @param param_prefix: Prefix for parameter keys
 * @param hand_params: The data structure to store loaded values into
 * @return True if all required params were loaded, else false
**/
bool read_hand_params(ros::NodeHandle& nh, std::string param_prefix, Hand::hand_params_t& hand_params) {

    if(!nh.getParam(param_prefix+"/ip", hand_params.ip)) {
        ROS_ERROR("Did not get hand ip, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/port", hand_params.port)) {
        ROS_ERROR("Did not get hand port, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/local_port", hand_params.local_port)) {
        ROS_ERROR("Did not get hand local port, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/use_tcp", hand_params.use_tcp)) {
        ROS_ERROR("Did not get hand comm method, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/finger0_type", hand_params.finger0_type)) {
        ROS_ERROR("Did not get type of finger 0, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/finger1_type", hand_params.finger1_type)) {
        ROS_ERROR("Did not get type of finger 1, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/hand_data_buffer_size", hand_params.hand_data_buffer_size)) {
        ROS_ERROR("Did not get hand state buffer max size, aborting");
        return false;
    }
    if(!nh.getParam(param_prefix+"/hand_read_rate", hand_params.hand_read_rate)) {
        ROS_ERROR("Did not get hand read rate, aborting");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {

    double width = 50.0;
    double speed = 30.0;

    ros::init(argc, argv, "test_hand_move");

    ros::NodeHandle nh;

    ROS_INFO("Reading hand params...");

    Hand::hand_params_t hand_params;
    if(!read_hand_params(nh, "hand0", hand_params)) {
        return -1;
    }

    WeissFinger::weiss_finger_params_t* finger0_params = NULL;
    WeissFinger::weiss_finger_params_t* finger1_params = NULL;

    ROS_INFO("Initializing hand...");
    Hand hand(&hand_params, finger0_params, finger1_params);

    ROS_INFO("Commanding hand to move: width = %f, speed = %f", width, speed);
    hand.move_hand(width,speed);

    ROS_INFO("...done");
    ros::spin();
}
