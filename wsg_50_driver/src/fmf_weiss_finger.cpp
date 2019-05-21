#include "wsg_50/fmf_weiss_finger.h"
#include "wsg_50/functions.h"
#include "wsg_50/cmd.h"
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cfloat>
#include <unistd.h>
#include <ctime>

/**
 * @brief FMFWeissFinger: Constructor
 * @param finger_params: Struct containing finger initialization params
*/
FMFWeissFinger::FMFWeissFinger(fmf_weiss_finger_params_t *finger_params):
                                                                                initialized_(false),
                                                                                can_calibrate_(false),
                                                                                data_buff_(0),
                                                                                read_finger_thread_(NULL),
                                                                                read_finger_alive_(false),
                                                                                paused_(true),
                                                                                force_offset_(0.0){
    // Abort if no parameters were provided
    if(finger_params == NULL) {
        return;
    }

    // Get parameters for basic operation
    finger_id_ = finger_params->finger_id;
    finger_data_buffer_size_ = finger_params->finger_data_buffer_size;
    finger_read_rate_ = finger_params->finger_read_rate;

    // Finger id must be 0 or 1
    if(finger_id_ > 2 || finger_id_ < 0) {
        return;
    }

    initialized_ = true;

    // Get parameters for performing calibration
    calibration_path_ = finger_params->calibration_path;
    calib_samples_ = finger_params->calib_samples;
    calib_target_ = finger_params->calib_target;

    // Number of samples must be greater than 0
    if(calib_samples_ < 0) {
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
        std::cout << "[fmf_weiss_finger] Cannot get data, finger id out of range" << std::endl;
        return false;
    }
    {
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex); // Get the lock, will release upon exiting block
        cmd_submit(cmd, NULL, 0, true, &resp, &resp_len); // Get data from finger
    }

    if(resp_len < 2) {
        std::cout << "[fmf_weiss_finger] Did not receive response from gripper" << std::endl;
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
        std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
        return false;
    }
    if(!can_calibrate_) {
        std::cout << "[fmf_weiss_finger] Did not receive all parameters necessary for calibration, abort" << std::endl;
        return false;
    }

    // Move the hand to some non-zero target so no force is applied
    if(!FMFWeissFinger::move_hand(*cmd_mutex, calib_target_)) {
        std::cout << "[fmf_weiss_finger] Could not move hand, aborting calibration" << std::endl;
        return false;
    }

    sleep(1);

    std::cout << "[fmf_weiss_finger] Collecting calibration data" << std::endl;
    force_offset_ = 0.0;
    std::vector<double> data;
    double finger_read_period = 1.0/finger_read_rate_;
    unsigned int sample_count = 0;
    while(sample_count < calib_samples_) {
        timespec start = weiss_finger_data_t::get_stamp();

        if(get_data(*cmd_mutex, data)) {
            force_offset_ += data[0];
            sample_count += 1;
        }

        timespec end = weiss_finger_data_t::get_stamp();
        long int sleep_length_us = (long int) (1000000 * finger_read_period - ((end.tv_sec + end.tv_nsec/1000000000.0) - (start.tv_sec + start.tv_nsec/1000000000.0)));
        if(sleep_length_us > 0) {
            usleep(sleep_length_us);
        }
    }
    force_offset_ = -1*force_offset_ / sample_count; // Compute the offset

    std::cout << "[fmf_weiss_finger] FORCE OFFSET: " << force_offset_;

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
        std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
        return false;
    }

    // Attempt to open the file
    std::ifstream cal_file;
    cal_file.open(calibration_path_);
    if(!cal_file) {
        std::cout << "[fmf_weiss_finger] Could not open cal file: " << calibration_path_.c_str() << std::endl;
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

    paused_ = false;

    // Get the command to send to finger for reading
    int cmd;
    if(finger_id_ == 0) {
        cmd = FINGER0_READ_CMD;
    } else if(finger_id_ == 1) {
        cmd = FINGER1_READ_CMD;
    } else {
        std::cout << "[fmf_weiss_finger] Cannot get data, finger id out of range" << std::endl;
        return;
    }

    double finger_read_period = 1.0/finger_read_rate_;

    // Lock for mutex, doesn't yet acquire mutex
    std::unique_lock<std::mutex> cmd_lock(*cmd_mutex, std::defer_lock);
    while(read_finger_alive_) {
        timespec start = weiss_finger_data_t::get_stamp();
        if(!paused_) { // Check whether we are paused
            // Try to get the lock
            cmd_lock.try_lock();
            if(!cmd_lock.owns_lock()) {
                continue;
            }

            // Create message to store result
            weiss_finger_data_t wfd;
            wfd.stamp = weiss_finger_data_t::get_stamp();

            // Read from the hand
            unsigned char* resp;
            unsigned int resp_len;
            cmd_submit(cmd, NULL, 0, true, &resp, &resp_len);
            cmd_lock.unlock(); // Release the lock

            // Push result into message
            if(resp_len < 2) {
                std::cout << "[fmf_weiss_finger] Did not receive response from gripper" << std::endl;
                continue;
            } else {
                wfd.data.clear();
                wfd.data.push_back(bytes_to_float(resp[5],resp[4],resp[3],resp[2]));
            }

            // Check if data is in bounds
            if(std::isnan(wfd.data[0]) || wfd.data[0] > 1000.0) {
                std::cout << "[fmf_weiss_finger] Data out of bounds" << std::endl;
                continue;
            }

            wfd.data[0] += force_offset_; // Apply force offset
            wfd.data_shape.resize(2); // Setup data shape
            wfd.data_shape[0] = 1;
            wfd.data_shape[1] = 1;

            // Push message into the buffer
            std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
            data_buff_.push_front(wfd);
            while(data_buff_.size() > finger_data_buffer_size_) { // Remove old messages from buffer
               data_buff_.pop_back();
            }

        }
        timespec end = weiss_finger_data_t::get_stamp();
        long int sleep_length_us = (long int) (1000000 * finger_read_period - ((end.tv_sec + end.tv_nsec/1000000000.0) - (start.tv_sec + start.tv_nsec/1000000000.0)));
        if(sleep_length_us > 0) {
            usleep(sleep_length_us);
        }
    }
}

/**
 * @brief start_reading: Initializes and begins reading from this finger
 * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
 * @return Whether or not reading was successfully started
*/
bool FMFWeissFinger::start_reading(std::mutex* cmd_mutex) {
    if(!initialized_) {
        std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
        return false;
    }

    if(read_finger_alive_ || read_finger_thread_) {
        std::cout << "[fmf_weiss_finger] Already reading from finger..." << std::endl;
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
    std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
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
    std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
    return false;
  }

  if(!read_finger_alive_) {
    std::cout << "[fmf_weiss_finger] FMF Finger not started, call start_reading first" << std::endl;
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
    std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
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
WeissFinger::weiss_finger_data_t FMFWeissFinger::get_sample() {
    if(!initialized_) {
        std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
        return weiss_finger_data_t();
    }
    if(data_buff_.size() <= 0) {
        std::cout << "[fmf_weiss_finger] Finger data buffer is empty" << std::endl;
        return weiss_finger_data_t();
    }
    std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
    weiss_finger_data_t result = data_buff_[0];
    return result;
}

/**
 * @brief get_latest_samples: Get the most recent n samples, ordered from newest to oldest
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one sample was returned, otherwise False
*/
bool FMFWeissFinger::get_latest_samples(unsigned int n_msgs, std::vector<weiss_finger_data_t> &buff) {

  if(!initialized_) {
    std::cout << "[fmf_weiss_finger] Not properly initialized, abort" << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> buffer_lock(data_buffer_mutex_); // Mutex will be released upon losing scope
  if(data_buff_.size() <= 0) {
    std::cout << "[fmf_weiss_finger] Finger data buffer is empty" << std::endl;
    return false;
  }

  // Return all messages
  if(n_msgs <= 0) {
    n_msgs = data_buff_.size();
  }

  // Decrease n_msgs if too many messages requested
  if(n_msgs > data_buff_.size()) {
    std::cout << "[fmf_weiss_finger] Requested " << n_msgs << " messages, but only returning " << data_buff_.size() << " latest" << std::endl;
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
        std::cout << "[fmf_weiss_finger] Imposible to move to this position. (Width values: [0.0 - 110.0] " << std::endl;
        return false;
    }else{
        std::cout << "[fmf_weiss_finger] Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values." << std::endl;
        std::lock_guard<std::mutex> cmd_lock(cmd_mutex); // Mutex will be released upon losing scope
        res = move(width, speed, false);
    }

    return res >= 0;

}
