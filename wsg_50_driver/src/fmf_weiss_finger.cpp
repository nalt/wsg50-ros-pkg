#include "wsg_50/fmf_weiss_finger.h"
#include "wsg_50/functions.h"
#include "wsg_50/cmd.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

/**
 * @brief FMFWeissFinger: Constructor
 * @param nh: Node handle for grabbing params
 * @param param_prefix: Prefix for param keys
*/
FMFWeissFinger::FMFWeissFinger(ros::NodeHandle &nh, std::string param_prefix):
                                                                                initialized_(false),
                                                                                can_calibrate_(false),
                                                                                data_buff_(0),
                                                                                read_finger_thread_(NULL),
                                                                                read_finger_alive_(false),
                                                                                paused_(true),
                                                                                force_offset_(0.0){
    // Get parameters for basic operation
    // Should be specified in handX.yaml
    int finger_id;
    if(!nh.getParam(param_prefix+"/finger_id", finger_id)) {
        ROS_ERROR("Did not receive finger id, abort");
        return;
    }
    finger_id_ = finger_id;

    if(finger_id_ > 2 || finger_id_ < 0) {
        ROS_ERROR("Finger id must be 0 or 1, abort");
        return;
    }
    if(!nh.getParam(param_prefix+"/finger_read_rate", finger_read_rate_)) {
        ROS_ERROR("Did not receive finger read rate, abort");
        return;
    }
    if(!nh.getParam(param_prefix+"/data_buff_max_size", data_buff_max_size_)) {
        ROS_ERROR("Did not receive max buffer size");
        return;
    }

    initialized_ = true;

    // Get parameters for performing calibration
    if(!nh.getParam(param_prefix+"/calibration_path", calibration_path_)) {
        ROS_WARN("Did not receive calibration path, cannot calibrate");
        return;
    }

    if(!nh.getParam(param_prefix+"/calib_samples", calib_samples_)) {
        ROS_WARN("Did not receive the number of calibration samples to perform, cannot calibrate");
        return;
    }
    if(calib_samples_ < 0) {
        ROS_WARN("Number of samples must be greater than 0");
        return;
    }
    if(!nh.getParam(param_prefix+"/calib_target", calib_target_)) {
        ROS_WARN("Did not receive the calibration target, cannot calibrate");
        return;
    }

    can_calibrate_ = true;

}

/**
 * @brief ~FMFWeissFinger: Destructor
*/
FMFWeissFinger::~FMFWeissFinger() {
    read_finger_alive_ = false; // Tell read_finger_thread_ to stop
    if(read_finger_thread_) {
        read_finger_thread_->join(); // Waits for read_finger_thread_ to stop
        delete read_finger_thread_;
    }
}

/**
 * @brief bytes_to_float: Converts a number represented by four bytes into a float
 * @param b0 First byte, lsb
 * @param b1
 * @param b2
 * @param b3 Last byte, msb
 * @return Float representation of number
*/
float FMFWeissFinger::bytes_to_float(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {
      unsigned char byte_array[] = { b3, b2, b1, b0 };
      float result;
      std::copy(reinterpret_cast<const char*>(&byte_array[0]),
                reinterpret_cast<const char*>(&byte_array[4]),
                reinterpret_cast<char*>(&result));
      return result;
}

/**
 * @brief get_data: Get one data sample from the finger
 * @param cmd_mutex: Must acquire this mutex before communicating with hand
 * @param data: Container for data from finger
 * @return True if data was successfully retrieved, else False
*/
bool FMFWeissFinger::get_data(std::mutex& cmd_mutex, std::vector<double>& data) {
    int cmd;
    unsigned char* resp;
    unsigned int resp_len;

    if(finger_id_ == 0) {
        cmd = FINGER0_READ_CMD;
    } else if(finger_id_ == 1) {
        cmd = FINGER1_READ_CMD;
    } else {
        ROS_ERROR("Cannot get data, finger id out of range");
        return false;
    }
    {
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex); // Get the lock, will release upon exiting block
        cmd_submit(cmd, NULL, 0, true, &resp, &resp_len); // Get data from finger
    }

    if(resp_len < 2) {
        ROS_ERROR("Did not receive response from gripper");
        return false;
    } else {
        data.clear();
        data.push_back(bytes_to_float(resp[5],resp[4],resp[3],resp[2]));
    }

    // Check that data is in-bounds
    if(std::isnan(data[0]) || data[0] > 1000.0) {
        return false;
    }

    return true;
}

/**
 * @brief do_calibration: Performs calibration
 * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
 * @return Whether or not the calibration was successful
*/
bool FMFWeissFinger::do_calibration(std::mutex* cmd_mutex) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return false;
    }
    if(!can_calibrate_) {
        ROS_ERROR("Did not receive all parameters necessary for calibration, abort");
        return false;
    }

    // Move the hand to some non-zero target so no force is applied
    if(!FMFWeissFinger::move_hand(*cmd_mutex, calib_target_)) {
        ROS_ERROR("Could not move hand, aborting calibration");
        return false;
    }

    ros::Duration(0.5).sleep();

    ROS_INFO("Collecting calibration data");
    force_offset_ = 0.0;
    std::vector<double> data;
    ros::Rate sample_rate(finger_read_rate_);
    unsigned int sample_count = 0;
    while(sample_count < calib_samples_ && ros::ok()) {
        if(get_data(*cmd_mutex, data)) {
            force_offset_ += data[0];
            sample_count += 1;
        }

        sample_rate.sleep();
    }
    force_offset_ = -1*force_offset_ / sample_count; // Compute the offset

    ROS_INFO("FORCE OFFSET: %f", force_offset_);

    // Write the calibration result to file
    std::ofstream cal_file;
    cal_file.open(calibration_path_);
    cal_file << force_offset_ << '\n';
    cal_file.close();

    return true;

}

/**
 * @brief load_calibration: Load calibration, including sending params to hand if necessary
 * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
 * @return Whether or not the calibration was successfully loaded
*/
bool FMFWeissFinger::load_calibration(std::mutex* cmd_mutex) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return false;
    }

    // Attempt to open the file
    std::ifstream cal_file;
    cal_file.open(calibration_path_);
    if(!cal_file) {
        ROS_ERROR("Could not open cal file: %s", calibration_path_.c_str());
        return false;
    }

    // Get the calibration value
    std::string line;
    std::getline(cal_file, line);

    // Convert to float
    force_offset_ = std::atof(line.c_str());

    cal_file.close();

    return true;
}

/**
 * @brief read_finger: Function used by read_finger_thread_
 * @param cmd_mutex: Must acquire this mutex before communicating with hand
*/
void FMFWeissFinger::read_finger(std::mutex* cmd_mutex) {

    ros::Rate read_rate(finger_read_rate_);
    paused_ = false;

    // Get the command to send to finger for reading
    int cmd;
    if(finger_id_ == 0) {
        cmd = FINGER0_READ_CMD;
    } else if(finger_id_ == 1) {
        cmd = FINGER1_READ_CMD;
    } else {
        ROS_ERROR("Cannot get data, finger id out of range");
        return;
    }

    // Lock for mutex, doesn't yet acquire mutex
    std::unique_lock<std::mutex> cmd_lock(*cmd_mutex, std::defer_lock);
    while(ros::ok() && read_finger_alive_) {
        if(!paused_) { // Check whether we are paused
            // Try to get the lock
            cmd_lock.try_lock();
            if(!cmd_lock.owns_lock()) {
                continue;
            }

            // Create message to store result
            wsg_50_common::WeissFingerData wfd;
            wfd.stamp = ros::Time::now();

            // Read from the hand
            unsigned char* resp;
            unsigned int resp_len;
            cmd_submit(cmd, NULL, 0, true, &resp, &resp_len);
            cmd_lock.unlock(); // Release the lock

            // Push result into message
            if(resp_len < 2) {
                ROS_ERROR("Did not receive response from gripper");
                continue;
            } else {
                wfd.data.clear();
                wfd.data.push_back(bytes_to_float(resp[5],resp[4],resp[3],resp[2]));
            }

            // Check if data is in bounds
            if(std::isnan(wfd.data[0]) || wfd.data[0] > 1000.0) {
                ROS_ERROR("Data out of bounds");
                continue;
            }

            wfd.data[0] += force_offset_; // Apply force offset
            wfd.data_shape.resize(2); // Setup data shape
            wfd.data_shape[0] = 1;
            wfd.data_shape[1] = 1;

            // Push message into the buffer
            std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
            data_buff_.push_front(wfd);
            while(data_buff_.size() > data_buff_max_size_) { // Remove old messages from buffer
               data_buff_.pop_back();
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
bool FMFWeissFinger::start_reading(std::mutex* cmd_mutex) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return false;
    }

    if(read_finger_alive_ || read_finger_thread_) {
        ROS_ERROR("Already reading...");
        return false;
    }

    read_finger_alive_ = true;
    read_finger_thread_ = new std::thread(&FMFWeissFinger::read_finger, this, cmd_mutex);

    return true;
}

/**
 * @brief pause_reading: Stop reading data from this finger. Should only be called
 * after start_reading()
 * @return Whether or not reading was successfully paused
*/
bool FMFWeissFinger::pause_reading() {

  if(!initialized_) {
    ROS_ERROR("Not properly initialized, aborting pause");
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
bool FMFWeissFinger::restart_reading() {

  if(!initialized_) {
    ROS_ERROR("Not properly initialized, aborting restart reading");
    return false;
  }

  if(!read_finger_alive_) {
    ROS_ERROR("FMF Finger not started, call start_reading first");
    return false;
  }

  paused_ = false;
  return true;
}

/**
 * @brief clear_sample_buffer: Remove previous samples from the sample buffer
 * @return Whether or not the sample buffer was successfully cleared
*/
bool FMFWeissFinger::clear_sample_buffer() {


  if(!initialized_) {
    ROS_ERROR("Not properly initialized, aborting restart reading");
    return false;
  }
  std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
  data_buff_.erase(data_buff_.begin(), data_buff_.end()); // Clear the buffer

  return true;
}

/**
 * @brief get_sample: Get the latest data sample
 * @return The latest data sample
*/
wsg_50_common::WeissFingerData FMFWeissFinger::get_sample() {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, abort");
        return wsg_50_common::WeissFingerData();
    }
    if(data_buff_.size() <= 0) {
        ROS_ERROR("Finger data buffer is empty");
        return wsg_50_common::WeissFingerData();
    }
    std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
    wsg_50_common::WeissFingerData result = data_buff_[0];
    return result;
}

/**
 * @brief get_latest_samples: Get the most recent n samples, ordered from newest to oldest
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one sample was returned, otherwise False
*/
bool FMFWeissFinger::get_latest_samples(unsigned int n_msgs, std::vector<wsg_50_common::WeissFingerData>& buff) {

  if(!initialized_) {
    ROS_ERROR("Not properly intialized, abort");
    return false;
  }

  std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
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
 * @brief move_hand: Command the hand to move
 * @param cmd_mutex: Must acquire this mutex before communicating with hand
 * @param width: The hand width to move to
 * @param speed: The speed with which to move
 * @return True if hand movement was successful, else False
 */
bool FMFWeissFinger::move_hand(std::mutex& cmd_mutex, double width, unsigned int speed) {
    int res;
    if ( (width >= 0.0 && width <= 110.0) && (speed > 0.0 && speed <= 420.0) ){
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex); // Mutex will be released upon losing scope
        res =  move(width, speed, false);
    }else if (width < 0.0 || width > 110.0){
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        return false;
    }else{
        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex); // Mutex will be released upon losing scope
        res = move(width, speed, false);
    }

    return res >= 0;

}
