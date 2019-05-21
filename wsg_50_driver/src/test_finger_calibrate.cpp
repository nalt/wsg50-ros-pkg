#include <ros/ros.h>
#include "wsg_50/hand.h"

#include <iostream>
#include <fstream>

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

/**
 * @brief read_fmf_finger_params: Reads fmf finger parameters from parameter server
 * @param nh: Node handle used for accessing server
 * @param param_prefix: Prefix for parameter keys
 * @param finger_params: The data structure to store loaded values into
 * @return True if all required params were loaded, else false
**/
bool read_fmf_finger_params(ros::NodeHandle& nh, std::string param_prefix, FMFWeissFinger::fmf_weiss_finger_params_t* finger_params) {

    // Get parameters for basic operation
    int finger_id;
    if(!nh.getParam(param_prefix+"/finger_id", finger_id)) {
        ROS_ERROR("Could not find finger id");
        return false;
    }
    if(finger_id > 2 || finger_id < 0) {
        ROS_ERROR("Finger id must be 0 or 1, abort");
        return false;
    }
    finger_params->finger_id = finger_id;
    if(!nh.getParam(param_prefix+"/finger_read_rate", finger_params->finger_read_rate)) {
        ROS_ERROR("Did not receive finger read rate, abort");
        return false;;
    }
    if(!nh.getParam(param_prefix+"/finger_data_buffer_size", finger_params->finger_data_buffer_size)) {
        ROS_ERROR("Did not receive max buffer size");
        return false;;
    }

    // Get parameters for performing calibration
    if(!nh.getParam(param_prefix+"/calibration_path", finger_params->calibration_path)) {
        ROS_WARN("Did not receive calibration path, cannot calibrate");
        return false;;
    }

    if(!nh.getParam(param_prefix+"/calib_samples", finger_params->calib_samples)) {
        ROS_WARN("Did not receive the number of calibration samples to perform, cannot calibrate");
        return false;;
    }
    if(finger_params->calib_samples < 0) {
        ROS_WARN("Number of samples must be greater than 0");
        return false;;
    }
    if(!nh.getParam(param_prefix+"/calib_target", finger_params->calib_target)) {
        ROS_WARN("Did not receive the calibration target, cannot calibrate");
        return false;;
    }

    return true;

}

bool read_optical_finger_params(ros::NodeHandle& nh, std::string param_prefix, OpticalWeissFinger::optical_weiss_finger_params_t* finger_params) {

    // Get parameters for basic operation
    int finger_id;
    if(!nh.getParam(param_prefix+"/finger_id", finger_id)) {
        ROS_ERROR("Could not find finger id");
        return false;
    }
    if(finger_id > 2 || finger_id < 0) {
        ROS_ERROR("Finger id must be 0 or 1, abort");
        return false;
    }
    finger_params->finger_id = finger_id;

    int devices;
    if(!nh.getParam(param_prefix+"/devices", devices)) {
        ROS_ERROR("Could not get number of devices");
        return false;
    }
    finger_params->devices = devices;

    if(!nh.getParam(param_prefix+"/reg_addrs_str", finger_params->reg_addrs)) {
        ROS_ERROR("Could not get register adresses");
        return false;
    }

    if(!nh.getParam(param_prefix+"/reg_lengths_str", finger_params->reg_lengths)) {
        ROS_ERROR("Could not get register lengths");
        return false;
    }

    if(!nh.getParam(param_prefix+"/finger_read_rate", finger_params->finger_read_rate)) {
        ROS_ERROR("Did not receive finger read rate");
        return false;
    }

    if(!nh.getParam(param_prefix+"/finger_data_buffer_size", finger_params->finger_data_buffer_size)) {
        ROS_ERROR("Did not receive max data buffer size");
        return false;
    }

    if(!nh.getParam(param_prefix+"/sensor_to_surface_mm", finger_params->sensor_to_surface_mm)) {
        ROS_ERROR("Did not receive sensor_to_surface_mm");
        return false;
    }
    if(!nh.getParam(param_prefix+"/target_surface_offset_mm", finger_params->target_surface_offset_mm)) {
        ROS_ERROR("Did not receive target_surface_offset_mm");
        return false;
    }

    // Get parameters for performing calibration
    if(!nh.getParam(param_prefix+"/calibration_path", finger_params->calibration_path)) {
        ROS_WARN("Did not receive calibration path");
    }

    if(!nh.getParam(param_prefix+"/calib_see_through_samples", finger_params->calib_see_through_samples)) {
        ROS_WARN("Did not receive calib_see_through_samples");
    }

    if(!nh.getParam(param_prefix+"/calib_see_through_target", finger_params->calib_see_through_target)) {
        ROS_WARN("Did not receive calib_see_through_target");
    }

    if(!nh.getParam(param_prefix+"/calib_offset_samples", finger_params->calib_offset_samples)) {
        ROS_WARN("Did not receive calib_offset_samples");
    }

    if(!nh.getParam(param_prefix+"/calib_offset_target", finger_params->calib_offset_target)) {
        ROS_WARN("Did not receive calib_offset_target");
    }

    if(!nh.getParam(param_prefix+"/calib_cross_talk_samples", finger_params->calib_cross_talk_samples)) {
        ROS_WARN("Did not receive calib_cross_talk_samples");
    }

    if(!nh.getParam(param_prefix+"/calib_cross_talk_target", finger_params->calib_cross_talk_target)) {
        ROS_WARN("Did not receive calib_cross_talk_target");
    }

    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_hand_read");

    ros::NodeHandle nh;

    Hand::hand_params_t hand_params;
    if(!read_hand_params(nh, "hand0", hand_params)) {
        return -1;
    }

    WeissFinger::weiss_finger_params_t* finger0_params;
    WeissFinger::weiss_finger_params_t* finger1_params;

    // Read the finger parameters
    if(hand_params.finger0_type.compare("optical") == 0) {
        finger0_params = new OpticalWeissFinger::optical_weiss_finger_params_t();
        if(!read_optical_finger_params(nh, "hand0/finger0", (OpticalWeissFinger::optical_weiss_finger_params_t*)finger0_params)) {
            return -1;
        }
    } else if(hand_params.finger0_type.compare("fmf") == 0) {
        finger0_params = new FMFWeissFinger::fmf_weiss_finger_params_t();
        if(!read_fmf_finger_params(nh, "hand0/finger0", (FMFWeissFinger::fmf_weiss_finger_params_t*)finger0_params)) {
            return -1;
        }
    } else if(hand_params.finger0_type.compare("dsa") == 0) {
        // Not implemented
        ROS_ERROR("DSA Finger not implemented, finger 0 diabled");
    } else {
       ROS_WARN("Finger 0 not active");
    }

    if(hand_params.finger1_type.compare("optical") == 0) {
        finger1_params = new OpticalWeissFinger::optical_weiss_finger_params_t();
        if(!read_optical_finger_params(nh, "hand0/finger1", (OpticalWeissFinger::optical_weiss_finger_params_t*)finger1_params)) {
            return -1;
        }
    } else if(hand_params.finger1_type.compare("fmf") == 0) {
        finger1_params = new FMFWeissFinger::fmf_weiss_finger_params_t();
        if(!read_fmf_finger_params(nh, "hand0/finger1", (FMFWeissFinger::fmf_weiss_finger_params_t*)finger1_params)) {
            return -1;
        }
    } else if(hand_params.finger1_type.compare("dsa") == 0) {
        // Not implemented
        ROS_ERROR("DSA Finger not implemented, finger 1 diabled");
    } else {
       ROS_WARN("Finger 1 not active");
    }


    Hand hand(&hand_params, finger0_params, finger1_params);

    hand.do_calibration(0);
    hand.do_calibration(1);

    hand.load_calibration(0);
    hand.load_calibration(1);

    hand.start_reading();
    hand.pause_hand_reading();
    hand.pause_finger_reading(1);

    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(30);
    while(ros::ok()) {
        WeissFinger::weiss_finger_data_t finger0_sample = hand.get_finger_sample(0);
        WeissFinger::weiss_finger_data_t finger1_sample = hand.get_finger_sample(1);

        unsigned int finger0_print_length = finger0_sample.data_shape.size() > 1 ? finger0_sample.data_shape[1] : finger0_sample.data_shape[0];
        for(unsigned int i = 0; i < finger0_print_length; i++) {
            std::cout << finger0_sample.data[i] << " ";
        }
        unsigned int finger1_print_length = finger1_sample.data_shape.size() > 1 ? finger1_sample.data_shape[1] : finger1_sample.data_shape[0];
        for(unsigned int i = 0; i < finger1_print_length; i++) {
            std::cout << finger1_sample.data[i] << " ";
        }
        std::cout << std::endl;

        loop_rate.sleep();
    }

    if(finger0_params) {
        delete finger0_params;
    }
    if(finger1_params) {
        delete finger1_params;
    }
}

