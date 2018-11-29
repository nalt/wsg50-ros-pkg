#include "wsg_50/optical_weiss_finger.h"
#include "wsg_50/functions.h"
#include "wsg_50/cmd.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#define CMD_SET_REG 0 // SPI command to set which register of optical sensor to read from.
#define CMD_ADD_REG 1 // SPI command to add a register of optical sensor to read
#define CMD_DEL_REG 2 // SPI command to stop reading from a register of optical sensor
#define CMD_READ_REG 3 // SPI command to read from all of the current registers
#define CMD_WRITE_REG 4 // SPI command to write to all of the current registers of a specific optical sensor
#define CMD_CALIB_RESULT 5 // SPI command to obtain the result of the most recent calibration
#define CMD_CALIB_SEE_THROUGH 6 // SPI command to initiate calibration of the detection threshold
#define CMD_CALIB_SEE_DARK 7 // Not used
#define CMD_CALIB_OFFSET 8 // SPI command to initiate calibration of the distance offset
#define CMD_CALIB_CROSS_TALK 9 // SPI command to initiate calibration of color/luminance compensation

#define RANGE_RATE_THRESH_REG 0x0026 // Optical sensor register that contains intensity measurements
#define RANGE_OFFSET_REG 0x0024 // Optical sensor register that contains the offset calibration value
#define CROSSTALK_COMPENSATION_REG 0x001E // Optical sensor register that contains the cross talk calibration value

#define MAX_REGISTERS 10 // The maximum number of registers that can be read from (enforced due to uController mem limits)

// Tokenize the passed string str using the passed char delim. Returns a list of the resulting tokens
std::vector<std::string> split_string(std::string str, char delim);

// Converts a string that represents a hex value into an integer. String str should not be prefixed with '0x'
unsigned int hex_str_to_val(std::string str);

// Converts a value into the corresponding hex string. Returned string will not be prefixed with '0x'
std::string hex_val_to_str(unsigned int val);

/**
 * @brief OpticalWeissFinger: Constructor
 * @param nh: Node handle for grabbing params
 * @param param_prefix: Prefix for param keys
*/
OpticalWeissFinger::OpticalWeissFinger(ros::NodeHandle& nh, std::string param_prefix):
                                                                                        nh_(nh),
                                                                                        initialized_(false),
                                                                                        can_calibrate_(false),
                                                                                        data_buff_(0),
                                                                                        read_finger_thread_(NULL),
                                                                                        read_finger_alive_(false),
                                                                                        paused_(true),
                                                                                        pub_topic_(""){
    // Get parameters for basic operation
    // Should be specified in handX.yaml
    int finger_id;
    if(!nh.getParam(param_prefix+"/finger_id", finger_id)) {
        ROS_ERROR("Could not find finger id");
        return;
    }
    finger_id_ = finger_id;

    int devices;
    if(!nh.getParam(param_prefix+"/devices", devices)) {
        ROS_ERROR("Could not get number of devices");
        return;
    }
    devices_ = devices;

    std::string reg_addrs_str;
    if(!nh.getParam(param_prefix+"/reg_addrs_str", reg_addrs_str)) {
        ROS_ERROR("Could not get register adresses");
        return;
    }
    std::vector<std::string> reg_addrs_toks = split_string(reg_addrs_str, ' ');
    reg_addrs_.clear();
    for(unsigned int i = 0; i < reg_addrs_toks.size(); i++) {
        if(i >= MAX_REGISTERS) {
            ROS_INFO("Can only add %d registers, the rest will not be added", MAX_REGISTERS);
            break;
        }
        reg_addrs_.push_back((uint16_t) hex_str_to_val(reg_addrs_toks[i]));
    }
    if(reg_addrs_.size() <= 0) {
        ROS_ERROR("Did not receive any register addresses, abort");
        return;
    }

    std::string reg_lengths_str;
    if(!nh.getParam(param_prefix+"/reg_lengths_str", reg_lengths_str)) {
        ROS_ERROR("Could not get register lengths");
        return;
    }
    std::vector<std::string> reg_lengths_toks = split_string(reg_lengths_str, ' ');
    reg_lengths_.clear();
    for(unsigned int i = 0; i < reg_lengths_toks.size(); i++) {
        if(i >= MAX_REGISTERS) {
            ROS_INFO("Can only add %d registers, the rest will not be added", MAX_REGISTERS);
            break;
        }
        reg_lengths_.push_back((uint8_t) hex_str_to_val(reg_lengths_toks[i]));
        if(reg_lengths_[i] > 4) {
            ROS_WARN("Register lengths greater than 4 can cause overflow");
        }
    }
    if(reg_lengths_.size() <= 0) {
        ROS_ERROR("Did not receive any register lengths, abort");
        return;
    }

    if(reg_addrs_.size() != reg_lengths_.size()) {
        ROS_ERROR("Unequal amount of register adresesses and lengths given, abort");
        return;
    }

    if(!nh.getParam(param_prefix+"/finger_read_rate", finger_read_rate_)) {
        ROS_ERROR("Did not receive finger read rate");
        return;
    }

    if(!nh.getParam(param_prefix+"/data_buff_max_size", data_buff_max_size_)) {
        ROS_ERROR("Did not receive max data buffer size");
        return;
    }

    initialized_ = true;

    bool publish = false;
    if(!nh.getParam(param_prefix+"/publish", publish)) {
        ROS_WARN("Did not get publish param, will not publish");
    }
    if(publish) {
      pub_topic_ = param_prefix + "/finger"+std::to_string(finger_id_);
    }

    if(!nh.getParam(param_prefix+"/sensor_to_surface_mm", sensor_to_surface_mm_)) {
        ROS_ERROR("Did not receive sensor_to_surface_mm");
        return;
    }
    if(!nh.getParam(param_prefix+"/target_surface_offset_mm", target_surface_offset_mm_)) {
        ROS_ERROR("Did not receive target_surface_offset_mm");
        return;
    }

    // Get parameters for performing calibration
    if(!nh.getParam(param_prefix+"/calibration_path", calibration_path_)) {
        ROS_ERROR("Did not receive calibration path");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_see_through_samples", calib_see_through_samples_)) {
        ROS_ERROR("Did not receive calib_see_through_samples");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_see_through_target", calib_see_through_target_)) {
        ROS_ERROR("Did not receive calib_see_through_target");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_offset_samples", calib_offset_samples_)) {
        ROS_ERROR("Did not receive calib_offset_samples");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_offset_target", calib_offset_target_)) {
        ROS_ERROR("Did not receive calib_offset_target");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_cross_talk_samples", calib_cross_talk_samples_)) {
        ROS_ERROR("Did not receive calib_cross_talk_samples");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_cross_talk_target", calib_cross_talk_target_)) {
        ROS_ERROR("Did not receive calib_cross_talk_target");
        return;
    }

    can_calibrate_ = true;

}

/**
 * @brief ~FMFWeissFinger: Destructor
*/
OpticalWeissFinger::~OpticalWeissFinger() {
    read_finger_alive_ = false; // Tell read_finger_thread_ to stop
    if(read_finger_thread_) {
        read_finger_thread_->join(); // Waits for read_finger_thread_ to stop
        delete read_finger_thread_;
    }

}

/**
 * @brief do_calibration: Performs calibration
 * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
 * @return Whether or not the calibration was successful
*/
bool OpticalWeissFinger::do_calibration(std::mutex* cmd_mutex) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return false;
    }
    if(!can_calibrate_) {
        ROS_ERROR("Cannot perform calibration, some params were not provided");
        return false;
    }

    // Containers for sending commands and receiving responses
    unsigned char payload[256];
    uint8_t payload_length;
    unsigned char* resp;
    unsigned int resp_len;

    // Get the offset calibration command
    double offset_target_width;
    OpticalWeissFinger::calib_offset(finger_id_, calib_offset_samples_, calib_offset_target_,
                                     sensor_to_surface_mm_, target_surface_offset_mm_, &offset_target_width,
                                     payload, &payload_length);

    ROS_INFO("Prepare target for offset calibration and press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    OpticalWeissFinger::move_hand(*cmd_mutex, offset_target_width); // Move the hand to the expected position
    ros::Duration(2.0).sleep();
    ROS_INFO("Beginning offset calibration...");
    {
        // Initiate data collection for offset calibration
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &resp, &resp_len);
    }

    // Generate command for retrieving offset calibration result
    OpticalWeissFinger::calib_result(finger_id_, devices_, payload, &payload_length);
    // Get the offset calibration result
    unsigned char* offset_resp;
    unsigned int offset_resp_len;
    {
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &offset_resp, &offset_resp_len);
    }
    ROS_INFO("offset resp length: %d", offset_resp_len);

    // Get the cross talk calibration command
    double cross_talk_target_width;
    OpticalWeissFinger::calib_cross_talk(finger_id_, calib_cross_talk_samples_, calib_cross_talk_target_,
                                         sensor_to_surface_mm_, target_surface_offset_mm_, &cross_talk_target_width,
                                         payload, &payload_length);
    ROS_INFO("Prepare target for cross talk calibration and press enter to continue");
    OpticalWeissFinger::move_hand(*cmd_mutex, cross_talk_target_width); // Move the hand to the expected distance
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    ROS_INFO("Beginning cross talk calibration...");
    {
        // Initiate data collection for cross talk calibration
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &resp, &resp_len);
    }

    // Generate command for retrieving cross talk calibration result
    OpticalWeissFinger::calib_result(finger_id_, devices_, payload, &payload_length);
    // Get the cross talk calibration result
    unsigned char* cross_talk_resp;
    unsigned int cross_talk_resp_len;
    {
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &cross_talk_resp, &cross_talk_resp_len);
    }

    ROS_INFO("crosstalk resp length: %d", cross_talk_resp_len);

    // Get the see through calibration command
    double see_through_target_width;
    OpticalWeissFinger::calib_see_through(finger_id_, calib_see_through_samples_, calib_see_through_target_,
                                          sensor_to_surface_mm_, target_surface_offset_mm_, &see_through_target_width,
                                          payload, &payload_length);
    ROS_INFO("Prepare target for see through calibration and press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    OpticalWeissFinger::move_hand(*cmd_mutex, see_through_target_width); // Move the hand to the expected distance
    ros::Duration(2.0).sleep();
    ROS_INFO("Beginning see through calibration...");
    {
        // Initiate data collection for see through calibration
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &resp, &resp_len);
    }

    // Generate command for retrieving the see through calibration result
    OpticalWeissFinger::calib_result(finger_id_, devices_, payload, &payload_length);
    // Get the see through calibration result
    unsigned char* see_through_resp;
    unsigned int see_through_resp_len;
    {
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &see_through_resp, &see_through_resp_len);
    }
    ROS_INFO("see through resp length: %d", see_through_resp_len);

    ROS_INFO("About to save data...");
    OpticalWeissFinger::save_cal_data(calibration_path_, devices_, see_through_resp, offset_resp, cross_talk_resp);
    ROS_INFO("...calibration complete");

    return true;

}

/**
 * @brief load_calibration: Load calibration, including sending params to hand if necessary
 * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
 * @return Whether or not the calibration was successfully loaded
*/
bool OpticalWeissFinger::load_calibration(std::mutex* cmd_mutex) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return false;
    }

    std::vector<std::vector<unsigned char>> calib_payloads;
    // Generate commands to push calibration data to the optical sensor
    if(OpticalWeissFinger::load_cal_data(calibration_path_, finger_id_, calib_payloads)) {
        // Execute each of the constructed commands
        for(unsigned int i = 0; i < calib_payloads.size(); i++) {
            unsigned char* resp;
            unsigned int resp_len;
            std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
            cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, &calib_payloads[i][0], calib_payloads[i].size(), true, &resp, &resp_len);
        }

        // Verify the calibration data
        unsigned char verify_payload[256]; // Will contain commands to retrieve loaded calibration data from optical sensor
        unsigned char expected_resp[256]; // Will contain what calibration data is expected to be returned from optical sensor
        uint8_t verify_payload_length, expected_resp_length;
        unsigned char* resp;
        unsigned int resp_len;
        // Construct commands to retrieve loaded calibration data and expected output
        OpticalWeissFinger::verify_cal_data(calibration_path_, finger_id_, devices_, verify_payload, &verify_payload_length, expected_resp, &expected_resp_length);
        {
            // Get the loaded calibration data
            std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
            cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, verify_payload, verify_payload_length, true, &resp, &resp_len);
        }

        // Verify that the returned data is the same as what was expected
        bool cal_correct = true;
        if(resp_len != expected_resp_length) {
            ROS_ERROR("Verify calibration failed, expected %d bytes, got %d", expected_resp_length, resp_len);
            cal_correct= false;
        } else {
            ROS_INFO("Checking calibration data...");

            for(unsigned int i = 0; i < resp_len; i++) {
                if(resp[i] != expected_resp[i]) {
                    cal_correct = false;
                    std::cout << "\n Expected " << (int) expected_resp[i] <<", got " << (int) resp[i] << '\n';
                } else {
                    std::cout << (int) resp[i] << ' ';
                }
            }
            std::cout << '\n';
        }

        if(cal_correct) {
            ROS_INFO("...calibration verified as correct");
        } else {
            ROS_ERROR("...calibration is invalid, will not continue any operations");
            initialized_ = false;
            return false;
        }
    } else {
        ROS_ERROR("Loading calibration data failed, will not continue any operations");
        initialized_ = false;
        return false;
    }

    return true;
}

/**
 * @brief read_finger: Function used by read_finger_thread_
 * @param cmd_mutex: Must acquire this mutex before communicating with hand
*/
void OpticalWeissFinger::read_finger(std::mutex* cmd_mutex) {
    ROS_INFO("Finger %d read thread started", finger_id_);

    unsigned char read_payload[256]; // Will hold command to read from the optical sensor
    uint8_t read_payload_length;
    unsigned char* resp; // Will hold data returned from optical sensor
    unsigned int resp_len;

    ros::Publisher data_pub;
    if(pub_topic_.length() > 0) {
      data_pub  = nh_.advertise<wsg_50_common::WeissFingerData>(pub_topic_, 1);
    }

    // Construct command to read from optical sensor
    OpticalWeissFinger::read_reg(finger_id_, devices_, reg_lengths_, read_payload, &read_payload_length);

    // Lock for mutex, doesn't yet acquire mutex
    std::unique_lock<std::mutex> cmd_lock(*cmd_mutex, std::defer_lock);
    paused_ = false;
    ros::Rate read_rate(finger_read_rate_);
    while(ros::ok() && read_finger_alive_)  {
        if(!paused_) { // Check whether we are paused
            // Try to get the lock
            cmd_lock.try_lock();
            if(!cmd_lock.owns_lock()) {
                continue;
            }
            ros::Time sample_time = ros::Time::now();

            // Get the data
            cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, read_payload, read_payload_length, true, &resp, &resp_len);
            cmd_lock.unlock();

            // Check for correct data reception, if successful push data into buffer
            if(resp_len < 2) {
                 ROS_ERROR("Did not receive response from gripper");
            } else {
                 wsg_50_common::WeissFingerData wfd = OpticalWeissFinger::cmd_response_to_msg(resp, resp_len, devices_, reg_lengths_, sample_time);
                std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Will be released upon losing scope
                data_buff_.push_front(wfd);
                while(data_buff_.size() > data_buff_max_size_) {
                    data_buff_.pop_back();
                }

                if(data_pub) {
                    /*for(unsigned int i = 0; i < wfd.data_shape[0]; i++) {
                      wfd.data[i*wfd.data_shape[1]+0] = remove_finger_measurement_offset(wfd.data[i*wfd.data_shape[1]+0]);
                    }*/
                    data_pub.publish(wfd);
                }
            }
        }
        read_rate.sleep();
    }

}

/**
 * @brief start_reading: Initializes and begins reading from this finger
 * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
 * @return Whether or not reading was successfully started
*/
bool OpticalWeissFinger::start_reading(std::mutex* cmd_mutex) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return false;
    }

    if(read_finger_alive_ || read_finger_thread_) {
        ROS_ERROR("Already reading...");
        return false;
    }

    unsigned char payload[256];
    uint8_t payload_length;

    // Setup registers to read from
    for(unsigned int i = 0; i < reg_addrs_.size(); i++) {
        unsigned char* resp;
        unsigned int resp_len;
        // Construct commands to add registers
        if(i == 0) {
            OpticalWeissFinger::set_reg(finger_id_, reg_addrs_[i], reg_lengths_[i], payload, &payload_length);
        } else {
            OpticalWeissFinger::add_reg(finger_id_, reg_addrs_[i], reg_lengths_[i], payload, &payload_length);
        }
        // Submit commands to the optical sensor
        std::lock_guard<std::mutex> cmd_lock(*cmd_mutex);
        cmd_submit(OpticalWeissFinger::FINGER_CMD_ID, payload, payload_length, true, &resp, &resp_len);
    }

    read_finger_alive_ = true; // Indicate read_finger thread is running
    read_finger_thread_ = new std::thread(&OpticalWeissFinger::read_finger, this, cmd_mutex); // Launch thread for reading data

    return true;

}

/**
 * @brief pause_reading: Stop reading data from this finger. Should only be called
 * after start_reading()
 * @return Whether or not reading was successfully paused
*/
bool OpticalWeissFinger::pause_reading() {
  if(!initialized_) {
      ROS_ERROR("Not properly initialized, abort");
      return false;
  }
  paused_ = true;
  return true;
}

/**
 * @brief restart_reading: Resume reading data from this finger. Should only be
 * called after pause_reading().
 * @return  Whether or not not reading was successfully resumed.
*/
bool OpticalWeissFinger::restart_reading() {
  if(!initialized_) {
      ROS_ERROR("Not properly initialized, abort");
      return false;
  }
  if(!read_finger_alive_) {
    ROS_ERROR("Optical Weiss finger not restarted, start_reading was never called");
    return false;
  }
  paused_ = false;
  return true;
}

/**
 * @brief clear_sample_buffer: Remove previous samples from the sample buffer
 * @return Whether or not the sample buffer was successfully cleared
*/
bool OpticalWeissFinger::clear_sample_buffer() {
  if(!initialized_) {
      ROS_ERROR("Not properly initialized, abort");
      return false;
  }
  std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Will be released upon losing scope
  data_buff_.erase(data_buff_.begin(), data_buff_.end());
  return true;
}

/**
 * @brief get_sample: Get the latest data sample
 * @return The latest data sample
*/
wsg_50_common::WeissFingerData OpticalWeissFinger::get_sample() {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return wsg_50_common::WeissFingerData();
    }
    std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Will be released upon losing scope
    if(data_buff_.size() <= 0) {
        ROS_ERROR("Finger data buffer is empty");
        return wsg_50_common::WeissFingerData();
    }
    wsg_50_common::WeissFingerData result = data_buff_[0]; // Get most recent sample
    return result;
}

/**
 * @brief get_latest_samples: Get the most recent n samples, ordered from newest to oldest
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one sample was returned, otherwise False
*/
bool OpticalWeissFinger::get_latest_samples(unsigned int n_msgs, std::vector<wsg_50_common::WeissFingerData>& buff) {
  if(!initialized_) {
      ROS_ERROR("Not properly initialized, abort");
      return false;
  }
  std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Will be released upon losing scope
  if(data_buff_.size() <= 0) {
      ROS_ERROR("Finger data buffer is empty");
      return false;
  }

  // Return all messages
  if(n_msgs <= 0) {
    n_msgs = data_buff_.size();
  }

  // Decrease n_msgs if too many messages requested
  if(n_msgs > data_buff_.size()) {
    ROS_WARN("Requested %u messages, but only returning %lu latest", n_msgs, data_buff_.size());
    n_msgs = data_buff_.size();
  }

  // Load messages into buffer
  buff.clear();
  for(unsigned int i = 0; i < n_msgs; i++) {
    buff.push_back(data_buff_[i]);
  }

  return true;
}

/**
 * @brief set_reg: Generate command to set which register of optical sensor to read from. Will cause any registers that were previously
 *        read from to stop being read from
 * @param finger_id: The id of this finger
 * @param reg_addr: The address of the register to read from
 * @param reg_length: The length of the register to read from
 * @param payload: Container for the command
 * @param payload_length: Length of the resulting command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::set_reg(unsigned int finger_id, uint16_t reg_addr, uint8_t reg_length, unsigned char* payload, uint8_t* payload_length) {

    payload[0] = finger_id;
    payload[1] = CMD_SET_REG;
    payload[2] = 0; // TODO: Specify array
    payload[3] = reg_addr >> 8;
    payload[4] = reg_addr & 0x00FF;
    payload[5] = reg_length;
    *payload_length = 6;

    return true;
}

/**
 * @brief add_reg: Generate command to add a register to be read from optical sensor
 * @param finger_id: The id of this finger
 * @param reg_addr: The address of the register to read from
 * @param reg_length: The length of the register to read from
 * @param payload: Container for the command
 * @param payload_length: Length of the resulting command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::add_reg(unsigned int finger_id, uint16_t reg_addr, uint8_t reg_length, unsigned char* payload, uint8_t* payload_length) {

    payload[0] = finger_id;
    payload[1] = CMD_ADD_REG;
    payload[2] = 0; // TODO: Specify array
    payload[3] = reg_addr >> 8;
    payload[4] = reg_addr & 0x00FF;
    payload[5] = reg_length;
    *payload_length = 6;

    return true;
}

/**
 * @brief del_reg: Generate command to stop reading an optical sensor register
 * @param finger_id: The id of this finger
 * @param reg_addr: The address of the register that should no longer be read
 * @param reg_length: The length of the register that should no longer be read
 * @param payload: Container for the command
 * @param payload_length: Length of the resulting command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::del_reg(unsigned int finger_id, uint16_t reg_addr, unsigned char* payload, uint8_t* payload_length) {

    payload[0] = finger_id;
    payload[1] = CMD_DEL_REG;
    payload[2] = 0; // TODO: Specify array
    payload[3] = reg_addr >> 8;
    payload[4] = reg_addr & 0x00FF;
    *payload_length = 5;

    return true;
}

/**
 * @brief read_reg: Generate command to read all of the current registers
 * @param finger_id: The id of this finger
 * @param devices: The number of optical sensors this finger has
 * @param reg_lengths: A vector containing the length of each register to be read
 * @param payload: Container for the command
 * @param payload_length: Length of the resulting command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::read_reg(unsigned int finger_id, unsigned int devices,  std::vector<uint8_t>& reg_lengths, unsigned char* payload, uint8_t* payload_length) {

    int read_length = 0;
    // Compute number of data bytes that need to be read per device
    for(unsigned int i = 0; i < reg_lengths.size(); i++) {
      read_length += reg_lengths[i];
    }
    payload[0] = finger_id;
    payload[1] = CMD_READ_REG;
    payload[2] = 0; // TODO: Specify array
    // Because spi transfer is bi-directional, need an element for each read
    for(unsigned int i = 0; i < devices; i++) {
        for(unsigned int j = 0; j < read_length; j++) {
            payload[3+i*read_length+j] = 0; // This can be any value
        }
    }
    *payload_length = 3+devices*read_length;

    return true;
}

/**
 * @brief write_reg: Generate command to write to a register of a specific optical sensor
 * @param finger_id: The id of this finger
 * @param device_id: The id of the optical sensor
 * @param tx_data: A vector of bytes to write to the register
 * @param payload: Container for the command
 * @param payload_length: Length of the resulting command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::write_reg(unsigned int finger_id, uint8_t device_id, const std::vector<uint8_t>& tx_data, unsigned char* payload, uint8_t* payload_length) {

    payload[0] = finger_id;
    payload[1] = CMD_WRITE_REG;
    payload[2] = 0; // TODO: Specify array
    payload[3] = device_id;
    for(unsigned int i = 0; i < tx_data.size(); i++) {
        payload[4+i] = tx_data[i];
    }
    *payload_length = 4 + tx_data.size();

    return true;

}

/**
 * @brief calib_result: Generate command to get the result of the most recent calibration
 * @param finger_id: The id of this finger
 * @param devices: The number of optica sensors this finger has
 * @param payload: Container for the command
 * @param payload_length: Length of the resulting command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::calib_result(unsigned int finger_id, unsigned int devices, unsigned char* payload, uint8_t* payload_length) {

    payload[0] = finger_id;
    payload[1] = CMD_CALIB_RESULT;
    payload[2] = 0; // TODO: Specify array
    // Expecting to read two bytes per device
    // Because spi is bi-directional, need element an element for each read
    for(unsigned int i = 0; i < 2*devices; i++) {
        payload[3+i] = 0; // This can be any value
    }
    *payload_length = 3+2*devices;

    return true;
}

/**
 * @brief calib_see_through: Generate command to initiate see through calibration, as well as the required hand width.
 *                           See through calibration determines the detection threshold for each sensor
 * @param finger_id: The id of this finger
 * @param calib_see_through_samples: The number of samples to collect
 * @param calib_see_through_target: The expected distance from the top of the optical sensors to the target
 * @param sensor_to_surface_mm: The offset from the top of the optical sensors to the surface of the finger
 * @param target_surface_offset_mm: The offset of the target from the surface of the other finger (on which the target is mounted)
 * @param target_width_mm: The required width of the hand will be stored here
 * @param payload: The command to initial see through calibration
 * @param payload_length: The length of the command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::calib_see_through(unsigned int finger_id, uint8_t calib_see_through_samples, uint8_t calib_see_through_target,
                                           double sensor_to_surface_mm, double target_surface_offset_mm,
                                           double* target_width_mm, unsigned char* payload, uint8_t* payload_length) {

    payload[0] = finger_id;
    payload[1] = CMD_CALIB_SEE_THROUGH;
    payload[2] = 0; // TODO: Specify array
    payload[3] = calib_see_through_samples;
    payload[4] = calib_see_through_target;
    *payload_length = 5;
    *target_width_mm = calib_see_through_target+target_surface_offset_mm-sensor_to_surface_mm;

    return true;

}

/*
bool OpticalFinger::calib_see_dark(uint8_t calib_samples, uint8_t calib_target, unsigned char* payload, uint8_t* payload_length) {
    payload[0] = finger_id_;
    payload[1] = CMD_CALIB_SEE_DARK;
    payload[2] = 0; // TODO: Specify array
    payload[3] = calib_samples;
    payload[4] = calib_target;
    *payload_length = 5;

    return true;
}
*/

/**
 * @brief calib_offset: Generate command to initiate offset calibration, as well as the required hand width.
 *                      Offset calibration determines the distance offset for each optical sensor
 * @param finger_id: The id of this finger
 * @param calib_offset_samples: The number of samples to collect
 * @param calib_offset_target: The expected distance from the top of the optical sensors to the target
 * @param sensor_to_surface_mm: The offset from the top of the optical sensors to the surface of the finger
 * @param target_surface_offset_mm: The offset of the target from the surface of the other finger (on which the target is mounted)
 * @param target_width_mm: The required width of the hand will be stored here
 * @param payload: The command to initial see through calibration
 * @param payload_length: The length of the command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::calib_offset(unsigned int finger_id,  uint8_t calib_offset_samples, uint8_t calib_offset_target,
                                      double sensor_to_surface_mm, double target_surface_offset_mm,
                                      double* target_width_mm, unsigned char* payload, uint8_t* payload_length) {
    payload[0] = finger_id;
    payload[1] = CMD_CALIB_OFFSET;
    payload[2] = 0; // TODO: Specify array
    payload[3] = calib_offset_samples;
    payload[4] = calib_offset_target;
    *payload_length = 5;
    *target_width_mm = calib_offset_target+target_surface_offset_mm-sensor_to_surface_mm;

    return true;
}

/**
 * @brief calib_cross_talk: Generate command to initiate cross talk calibration, as well as the required hand width.
 *                          Cross talk calibration reduces sensor measurement variance due to changes in target reflectivity/luminance
 * @param finger_id: The id of this finger
 * @param calib_cross_talk_samples: The number of samples to collect
 * @param calib_cross_talk_target: The expected distance from the top of the optical sensors to the target
 * @param sensor_to_surface_mm: The offset from the top of the optical sensors to the surface of the finger
 * @param target_surface_offset_mm: The offset of the target from the surface of the other finger (on which the target is mounted)
 * @param target_width_mm: The required width of the hand will be stored here
 * @param payload: The command to initial see through calibration
 * @param payload_length: The length of the command
 * @return Whether or not this command was successfully constructed
*/
bool OpticalWeissFinger::calib_cross_talk(unsigned int finger_id, uint8_t calib_cross_talk_samples, uint8_t calib_cross_talk_target,
                                     double sensor_to_surface_mm, double target_surface_offset_mm,
                                     double* target_width_mm, unsigned char* payload, uint8_t* payload_length) {
    payload[0] = finger_id;
    payload[1] = CMD_CALIB_CROSS_TALK;
    payload[2] = 0; // TODO: Specify array
    payload[3] = calib_cross_talk_samples;
    payload[4] = calib_cross_talk_target;
    *payload_length = 5;
    *target_width_mm = calib_cross_talk_target+target_surface_offset_mm-sensor_to_surface_mm;

    return true;
}

/**
 * @brief save_cal_data: Saves the calibration data to disk. After each calibration, perform the command
 *                        returned by calib_result(), and then pass the result (held in the payload parameter) to this
 *                        function.
 * @param path: The path to save the calibration data to
 * @param devices: The number of optical sensors in this finger
 * @param see_through_result: The calibration result after see_through calibration
 * @param offset_result: The calibration result after offset calibration
 * @param cross_talk_result: The calibration result after cross talk calibration
 * @return Whether or not the calibration data was successfully saved
*/
bool OpticalWeissFinger::save_cal_data(std::string path,
                                       unsigned int devices,
                                       unsigned char* see_through_result,
                                       unsigned char* offset_result,
                                       unsigned char* cross_talk_result) {
    if(!see_through_result || !offset_result || !cross_talk_result) {
        ROS_ERROR("Must provide all calibration data");
        return false;
    }
    // Store calibration data
    // Each line has data for a single device
    std::ofstream cal_file;
    cal_file.open(path);
    for(unsigned int i = 0; i < devices; i++) {
        ROS_INFO("Dev %d, val %d, writing see through msb",i, see_through_result[3+2*i]);
        cal_file << hex_val_to_str(see_through_result[3+2*i]);
        ROS_INFO("Wrote msb");
        cal_file << ',';
        ROS_INFO("Dev %d,val %d, writing see through lsb",i,see_through_result[3+2*i+1]);
        cal_file << hex_val_to_str(see_through_result[3+2*i+1]);
        cal_file << ',';

        ROS_INFO("Dev %d,val %d, writing offset",i,offset_result[3+2*i+1]);
        cal_file << hex_val_to_str(offset_result[3+2*i+1]);
        cal_file << ',';

        ROS_INFO("Dev %d,val %d, writing crosstalk lsb",i,cross_talk_result[3+2*i]);
        cal_file << hex_val_to_str(cross_talk_result[3+2*i]);
        cal_file << ',';
        ROS_INFO("Dev %d,val %d, writing cross talk msb",i,cross_talk_result[3+2*i+1]);
        cal_file << hex_val_to_str(cross_talk_result[3+2*i+1]);
        cal_file << '\n';
    }
    cal_file.close();
    return true;
}

/**
 * @brief open_cal_data: Load calibration data from file
 * @param path: The path to the file containing calibration data
 * @param calib_data: Container for the calibration data
 * @return Whether or not calibration data was successfully loaded
*/
bool OpticalWeissFinger::open_cal_data(std::string path, std::vector<std::vector<unsigned char> >& calib_data) {
    std::ifstream cal_file;
    cal_file.open(path);
    if(!cal_file) {
        ROS_ERROR("Could not open cal file: %s", path.c_str());
        return false;
    }

    // Parse the csv file for calibration data
    // Each line represents the calibration data for a single device
    std::string line;
    while(std::getline(cal_file, line)) {
        std::vector<unsigned char> device_data;
        std::vector<std::string> tokens = split_string(line, ',');
        unsigned char see_through_msb = hex_str_to_val(tokens[0]);
        unsigned char see_through_lsb = hex_str_to_val(tokens[1]);
        unsigned char offset = hex_str_to_val(tokens[2]);
        unsigned char cross_talk_msb = hex_str_to_val(tokens[3]);
        unsigned char cross_talk_lsb = hex_str_to_val(tokens[4]);
        device_data.push_back(see_through_msb);
        device_data.push_back(see_through_lsb);
        device_data.push_back(offset);
        device_data.push_back(cross_talk_msb);
        device_data.push_back(cross_talk_lsb);
        calib_data.push_back(device_data);
    }

    cal_file.close();

    return true;
}

/**
 * @brief load_cal_data: Opens the calibration data and constructs commands to upload them to the sensor
 * @param path: The file that calibration data should be loaded from
 * @param finger_id: The id of this finger
 * @param calib_payloads: Vector of commands that should be sent to the hand in order to upload the calibration
 * @return Whether or not the commands were successfully constructed
*/
bool OpticalWeissFinger::load_cal_data(std::string path,
                                       unsigned int finger_id,
                                       std::vector<std::vector<unsigned char>>& calib_payloads) {

    calib_payloads.clear();
    std::vector<std::vector<unsigned char> > calib_data;
    // Get the data from disk
    if(!open_cal_data(path, calib_data)) {
        ROS_ERROR("Could not load calibration data");
        return false;
    }
    unsigned char char_payload[64];
    uint8_t char_payload_length;

    // Command to write see_through calibration register
    std::vector<unsigned char> payload;
    OpticalWeissFinger::set_reg(finger_id, RANGE_RATE_THRESH_REG, 2, char_payload, &char_payload_length);
    payload.assign(char_payload, char_payload+char_payload_length);
    calib_payloads.push_back(payload);

    // Command to write offset calibration register
    payload.clear();
    OpticalWeissFinger::add_reg(finger_id, RANGE_OFFSET_REG, 1, char_payload, &char_payload_length);
    payload.assign(char_payload, char_payload+char_payload_length);
    calib_payloads.push_back(payload);

    // Command to write cross talk calibration register
    payload.clear();
    OpticalWeissFinger::add_reg(finger_id, CROSSTALK_COMPENSATION_REG, 2, char_payload, &char_payload_length);
    payload.assign(char_payload, char_payload+char_payload_length);
    calib_payloads.push_back(payload);

    // Each command corresponds to writing calibration data to one of the optical sensors
    for(unsigned int i = 0; i < calib_data.size(); i++) {
        payload.clear();
        OpticalWeissFinger::write_reg(finger_id, i, calib_data[i], char_payload, &char_payload_length);
        payload.assign(char_payload, char_payload+char_payload_length);
        calib_payloads.push_back(payload);
    }

    return true;
}

/**
 * @brief verify_cal_data: Verify that calibration data was correctly loaded into the sensor. This function
 *                         will generate the command to send to the hand to retrieve the calibration data
 *                         loaded into the sensor, as well as the expected response. One can then verify
 *                         the data by checking that the expected and retrieved calibration data exactly match
 *                         It should be called after load_data, but before submitting any subsequent commands
 * @param path: Path to file that contains the calibration data
 * @param finger_id: The id of this finger
 * @param devices: The number of optical sensors that this finger has
 * @param verify_payload: Container for command to retrieve
 * @param verify_payload_length: The length of the retrieval command
 * @param expected_resp: The expected response of the verify payload command (if calibration is correct)
 * @param expected_resp_length: The length of the expected response
 * @return Whether or not the command and expected response were successfully constructed
*/
bool OpticalWeissFinger::verify_cal_data(std::string path,
                                         unsigned int finger_id,
                                         unsigned int devices,
                                         unsigned char* verify_payload,
                                         uint8_t* verify_payload_length,
                                         unsigned char* expected_resp,
                                         uint8_t* expected_resp_length) {
    // Read the calibration data from disk
    std::vector<std::vector<unsigned char> > calib_data;
    if(!open_cal_data(path, calib_data)) {
        ROS_ERROR("Could not load calibration data");
        return false;
    }


    // Construct command to read from current registers (which should be the calibration registers)
    std::vector<uint8_t> cal_reg_lengths;
    cal_reg_lengths.push_back(2);
    cal_reg_lengths.push_back(1);
    cal_reg_lengths.push_back(2);
    OpticalWeissFinger::read_reg(finger_id, devices, cal_reg_lengths, verify_payload, verify_payload_length);

    // Format calibration data from disk as the response that is expected to be received
    expected_resp[0] = 1;
    expected_resp[1] = 1;
    expected_resp[2] = 43;
    *expected_resp_length = 3;
    for(unsigned int i = 0; i < calib_data.size(); i++) {
        for(unsigned int j = 0; j < calib_data[i].size(); j++) {

            expected_resp[*expected_resp_length] = calib_data[i][j];
            *expected_resp_length += 1;
        }
    }

    return true;

}

/**
 * @brief cmd_response_to_msg: De-serialize sensor data from hand into a message
 * @param resp: The response received from reading data
 * @param resp_len: The length of the response
 * @param devices: The number of optical sensors in the finger
 * @param reg_lengths: The length of each register that was read
 * @param stamp: The stamp to use for the returned message, defaults to the time at which function was called
 * @return Message with data read from the hand
*/
wsg_50_common::WeissFingerData OpticalWeissFinger::cmd_response_to_msg(unsigned char* resp, unsigned int resp_len,
                                                                       unsigned int devices, std::vector<uint8_t>& reg_lengths,
                                                                       ros::Time stamp) {
    wsg_50_common::WeissFingerData result;
    result.stamp = stamp;

    // Compute the number of bytes per device
    unsigned int total_reg_len = 0;
    for(unsigned int i = 0; i < reg_lengths.size(); i++) {
        total_reg_len += reg_lengths[i];
        if(reg_lengths[i] > 4) {
            ROS_ERROR("cmd_response_to_msg will have unpredictable results for reg lengths greater than 4");
            return result;
        }
    }

    // Verify that the correct amount of data was received
    unsigned int exp_length = 3+(total_reg_len)*devices;
    if(resp_len != exp_length) {
        ROS_ERROR("Expected %d bytes, got %d", exp_length, resp_len);
        return result;
    }

    // De-serialize data into the message
    result.data.clear();
    int resp_idx = 3;
    for(unsigned int i = 0; i < devices; i++) {
        for(unsigned int j = 0; j < reg_lengths.size(); j++) {
            uint32_t val = 0;
            for(unsigned int k = 0; k < reg_lengths[j]; k++) {
                val = val << 8;
                val += resp[resp_idx];
                resp_idx += 1;
            }
            result.data.push_back((double) val);
        }

    }
    result.data_shape.resize(2);
    result.data_shape[0] = devices;
    result.data_shape[1] = reg_lengths.size();

    return result;
}

/**
 * @brief hand_width_to_finger_tip_separation: Computes the distance from this finger to the target on
 *        the other finger.
 * @param width: The current width of the hand
 * @return The distance between the surface of this finger and the target on the other fingertip
*/
double OpticalWeissFinger::hand_width_to_finger_tip_separation(double width) {
    return width - target_surface_offset_mm_;
}

/**
 * @brief remove_finger_measurement_offset: Given a raw range measurement from an optical sensor,
 *                                          converts to the measured distance from the surface of the finger
 * @param measurement: Raw range measurement
 * @return The distance measured from the surface of the finger to the object
*/
double OpticalWeissFinger::remove_finger_measurement_offset(double measurement) {
    return measurement - sensor_to_surface_mm_;
}

/**
 * @brief move_hand: Command the hand to move
 * @param cmd_mutex: Must acquire this mutex before communicating with hand
 * @param width: The hand width to move to
 * @param speed: The speed with which to move
 * @return True if hand movement was successful, else False
 */
bool OpticalWeissFinger::move_hand(std::mutex& cmd_mutex, double width, unsigned int speed) {
    int res;
    if ( (width >= 0.0 && width <= 110.0) && (speed > 0.0 && speed <= 420.0) ){
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex);
        res =  move(width, speed, false);
    }else if (width < 0.0 || width > 110.0){
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        return false;
    }else{
        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex);
        res = move(width, speed, false);
    }

    return res >= 0;

}

/**
 * @brief split_string: Tokenize the passed string using the passed char.  Returns
 * @param str: The string to tokenize
 * @param delim: The delimitter to use in tokenization
 * @return A list of the resulting tokens
*/
std::vector<std::string> split_string(std::string str, char delim) {
    std::stringstream ss(str);
    std::string token;
    std::vector<std::string> tokens;
    while(getline(ss, token, delim)) {
        tokens.push_back(token);
    }
    return tokens;
}

/**
 * @brief hex_str_to_val: Converts a string that represents a hex value into an integer.
 *                        String should not be prefixed with '0x'
 * @param str: The string to be converted
 * @return The converted value
 */
unsigned int hex_str_to_val(std::string str) {
    unsigned int result = 0;
    for(unsigned int i = 0; i < str.size(); i++) {
        unsigned int char_val = 0;
        if(str[i] >= 'a' && str[i] <= 'f') {
            char_val = str[i]-'a'+10;
        } else if(str[i] >= 'A' && str[i] <= 'F') {
            char_val = str[i]-'A'+10;
        } else if(str[i] >= '0' && str[i] <= '9'){
            char_val = str[i]-'0';
        } else {
            ROS_ERROR("Recevied non-hex char");
        }
        result += (char_val << 4*(str.size()-1-i));
    }
    return result;

}

/**
 * @brief hex_val_to_str: Converts a value into the corresponding hex string.
 *                        Returned string will not be prefixed with '0x'
 * @param val: The value to be converted
 * @return String representation of hex value
*/
std::string hex_val_to_str(unsigned int val) {
    if(val == 0) {
        return "0";
    }
    std::string result("");
    while(val > 0) {
        unsigned char lsb = val & 0x0F;
        if(lsb <= 9) {
            lsb += '0';
        } else if(lsb <= 15) {
            lsb += 'a'-10;
        } else {
            ROS_ERROR("Error in hex_val_to_str: %d", lsb);
        }
        result += lsb;
        val = val >> 4;
    }
    std::reverse(result.begin(), result.end());
    return result;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "optical_weiss_finger");
    ros::NodeHandle nh;

    unsigned int in_val = 168;
    std::string out_val = hex_val_to_str(in_val);

    ROS_INFO("In val: %d, out_val: %s", in_val, out_val.c_str());

}
