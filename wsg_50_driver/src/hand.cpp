#include <limits>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string.h>
#include "wsg_50/hand.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"
#include "wsg_50/cmd.h"

/**
 * @brief Hand: Constructor
 * @param hand_params: Struct containing hand initialization params
 * @param finger0_params: Struct containing finger0 initialization params
 * @param finger1_params: Struct containing finger1 initialization params
*/
Hand::Hand(hand_params_t *hand_params,
           WeissFinger::weiss_finger_params_t *finger0_params,
           WeissFinger::weiss_finger_params_t *finger1_params):
                                                           initialized_(false),
                                                           cmd_mutex_(),
                                                           cmd_lock_(cmd_mutex_, std::defer_lock),
                                                           finger0_(NULL),
                                                           finger1_(NULL),
                                                           hand_state_buffer_(0),
                                                           read_hand_thread_(NULL),
                                                           read_hand_alive_(false),
                                                           hand_reading_paused_(true),
                                                           hand_read_rate_(0.0) {

    // Get parameters for basic operation
    // Should be specified in handX.yaml
    std::string ip(hand_params->ip);
    int port = hand_params->port;
    int local_port = hand_params->local_port;
    bool use_tcp = hand_params->use_tcp;

    std::string finger0_type(hand_params->finger0_type);
    std::string finger1_type(hand_params->finger1_type);

    hand_data_buffer_size_ = hand_params->hand_data_buffer_size;
    hand_read_rate_ = hand_params->hand_read_rate;

    // Setup connection to hand
    int res_con;
    if (use_tcp) {
        res_con = cmd_connect_tcp( ip.c_str(), port );
    } else {
        res_con = cmd_connect_udp(local_port, ip.c_str(), port );
    }

    if (res_con == 0 ) {
         homing();
         sleep(1);
         //doTare();

         std::cout << "[weiss_hand] Setting up fingers" << std::endl;

         // Configure the fingers
         if(finger0_type.compare("optical") == 0 && finger0_params != NULL) {
             finger0_ = new OpticalWeissFinger((OpticalWeissFinger::optical_weiss_finger_params_t*)finger0_params);
         } else if(finger0_type.compare("fmf") == 0 && finger0_params != NULL) {
             finger0_ = new FMFWeissFinger((FMFWeissFinger::fmf_weiss_finger_params_t*)finger0_params);
         } else if(finger0_type.compare("dsa") == 0 && finger0_params != NULL) {
             std::cout << "[weiss_hand] DSA Not implemented" << std::endl;
         } else {
             std::cout << "[weiss_hand] Finger 0 not active" << std::endl;
         }

         if(finger1_type.compare("optical") == 0 && finger1_params != NULL) {
             finger1_ = new OpticalWeissFinger((OpticalWeissFinger::optical_weiss_finger_params_t*)finger1_params);
         } else if(finger1_type.compare("fmf") == 0 && finger1_params != NULL) {
             finger1_ = new FMFWeissFinger((FMFWeissFinger::fmf_weiss_finger_params_t*)finger1_params);
         } else if(finger1_type.compare("dsa") == 0 && finger1_params != NULL) {
             std::cout << "[weiss_hand] DSA Not implemented" << std::endl;
         } else {
             std::cout << "[weiss_hand] Finger 1 not active" << std::endl;
         }

         initialized_ = true;

     } else {
         std::cout << "[weiss_hand] Failed to initialize" << std::endl;
     }

}

/**
 * @brief ~Hand: Destructor
*/
Hand::~Hand() {
    // Shutdown the fingers
    if(finger0_) {
        delete finger0_;
    }
    if(finger1_) {
        delete finger1_;
    }

    read_hand_alive_ = false; // Tell hand thread to stop
    if(read_hand_thread_) {
        read_hand_thread_->join(); // Wait for hand thread to stop
        delete read_hand_thread_;
    }

    if(cmd_is_connected()) {
        cmd_disconnect(); // End connection with hand
    }

}

/**
 * @brief do_calibration: Initiate calibration of one of the fingers
 * @param finger_id: The id of the finger to be calibrated
 * @return Whether or not calibration was successful
*/
bool Hand::do_calibration(unsigned int finger_id) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting calibration " << std::endl;
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting calibration" << std::endl;
        return false;
    }

    if(finger) {
        finger->do_calibration(&cmd_mutex_);
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active" << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief load_calibration: Load calibration data into one of the fingers
 * @param finger_id: The id of the finger to load calibration data into
 * @return Whether or not loading calibration was successful
*/
bool Hand::load_calibration(unsigned int finger_id) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting calibration load" << std::endl;
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting calibration" << std::endl;
        return false;
    }

    if(finger) {
        finger->load_calibration(&cmd_mutex_);
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active" << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief start_reading: Initializes and begins reading from this hand and both of its finger
 * @return Whether or not reading was successfully started
*/
bool Hand::start_reading() {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting read commencement" << std::endl;
        return false;
    }

    // Start reading from the hand
    read_hand_alive_ = true;
    hand_reading_paused_ = false;
    read_hand_thread_ = new std::thread(&Hand::read_hand_state, this);

    // Start reading from the fingers
    bool success = true;
    std::cout << "[weiss_hand] Going to start reading finger 0" << std::endl;
    if(finger0_) {
        if(!finger0_->start_reading(&cmd_mutex_)) {
            success = false;
        }
    }
    std::cout << "[weiss_hand] Going to start reading finger 1" << std::endl;
    if(finger1_) {
        if(!finger1_->start_reading(&cmd_mutex_)) {
            success = false;
        }
    }

    return success;
}

/**
 * @brief pause_hand_reading: Pause reading from the hand (only)
 * @return Whether or not reading was successfully paused
*/
bool Hand::pause_hand_reading() {
  if(!initialized_) {
    std::cout << "[weiss_hand] Not properly initialized, aborting pause" << std::endl;
    return false;
  }
  hand_reading_paused_ = true;
  return true;
}

/**
 * @brief restart_hand_reading: Restart reading from the hand (only)
 * @return Whether or not reading was successfully restarted
*/
bool Hand::restart_hand_reading() {
  if(!initialized_) {
    std::cout << "[weiss_hand] Not properly initialized, aborting read restart" << std::endl;
    return false;
  }
  if(!read_hand_alive_) {
    std::cout << "[weiss_hand] Hand reading never started, call start reading first, abort" << std::endl;
    return false;
  }
  hand_reading_paused_ = false;
  return true;
}

/**
 * @brief pause_finger_reading: Pause reading from one of the fingers
 * @param finger_id: Id of finger that should pause reading
 * @return  Whether or not reading was successfully paused
*/
bool Hand::pause_finger_reading(unsigned int finger_id) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting finger pause" << std::endl;
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting finger pause" << std::endl;
        return false;
    }

    if(finger) {
        return finger->pause_reading();
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active";
        return false;
    }

    return true;
}

/**
 * @brief restart_finger_reading
 * @param finger_id
 * @return
*/
bool Hand::restart_finger_reading(unsigned int finger_id) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting finger restart" << std::endl;
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting finger restart" << std::endl;
        return false;
    }

    if(finger) {
        return finger->restart_reading();
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active" << std::endl;
        return false;
    }

    return true;
}

/**
 * @brief move_hand: Command the hand to move
 * @param width: The hand width to move to
 * @param speed: The speed with which to move
 * @param ignore_response: False if function should block until hand is done moving, true to send command and return
 * @return True if hand movement was successful when ignore_response is false, or when command was successfully sent
 *         when ignore_response is true, else False
*/
bool Hand::move_hand(float width, float speed, bool ignore_response) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting hand movement" << std::endl;
        return false;
    }
    int res;
    if ( (width >= 0.0 && width <= 110.0) && (speed > 0.0 && speed <= 420.0) ){

    }else if (width < 0.0 || width > 110.0){
        std::cout << "[weiss_hand] Imposible to move to this position. (Width values: [0.0 - 110.0] " << std::endl;
        return false;
    }else{
        std::cout << "[weiss_hand] Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values." << std::endl;

    }

    // Setup command to send
    unsigned char payload[9];
    unsigned char* resp;
    unsigned int resp_len;
    memcpy(&payload[0], &width, sizeof(float));
    memcpy(&payload[4], &speed, sizeof(float));
    if(ignore_response) {
      payload[8] = 0;
    } else {
      payload[8] = 1;
    }


    while(!cmd_lock_.owns_lock()) {
      cmd_lock_.try_lock();
    }
    //res = move(width, speed, true, ignore_response);

    // Send command to move
    res = cmd_submit(Hand::MOVE_HAND_CMD_ID,  payload, 9, true, &resp, &resp_len);
    cmd_lock_.unlock();

    return res >= 0;

}

/**
 * @brief hand_is_moving: Reports whether the hand is currently moving
 * @return True if the hand is moving, else false
*/
bool Hand::hand_is_moving() {


  while(hand_state_buffer_.size() <= 0); // Wait until there is hand data

  hand_data_t hand_state;
  while(!get_hand_state(hand_state)) { // Wait until we get hand data
    usleep(100000);
  }

  return std::abs(hand_state.speed) > Hand::NOT_MOVING_SPEED_THRESH;

}

/**
 * @brief stop_hand: Tell the hand to stop moving. (TODO: Blocks until hand stops?)
 * @return If the stop command was succesfully sent
*/
bool Hand::stop_hand() {

    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, will not stop hand movement" << std::endl;
        return false;
    }
    //std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);

    while(!cmd_lock_.owns_lock()) {
      cmd_lock_.try_lock();
    }

    stop(false);

    cmd_lock_.unlock();


    return true;
}

/**
 * @brief clear_hand_state_buffer: Remove previous hand state samples from the buffer
 * @return Whether or not the buffer was succesfully cleared
*/
bool Hand::clear_hand_state_buffer() {
  if(!initialized_) {
      std::cout << "[weiss_hand] Not properly initialized, abort" << std::endl;
      return false;
  }
  std::lock_guard<std::mutex> buffer_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
  hand_state_buffer_.erase(hand_state_buffer_.begin(), hand_state_buffer_.end());
  return true;
}

/**
 * @brief get_hand_state: Get the latest hand state
 * @return  The latest hand state
*/
bool Hand::get_hand_state(hand_data_t &hand_state) {

    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting hand state" << std::endl;
        return false;
    }

    if(hand_state_buffer_.size() <= 0) {
        std::cout << "[weiss_hand] Hand data buffer is empty" << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> hand_state_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
    hand_state = hand_state_buffer_[0];
    return true;
}

/**
 * @brief get_hand_state: Get the latest hand state
 * @param hand_state: Container for the latest hand state
 * @return  Whether the latest hand state was retrieved
*/
Hand::hand_data_t Hand::get_hand_state() {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting get hand state" << std::endl;
        return hand_data_t();
    }

    if(hand_state_buffer_.size() <= 0) {
        std::cout << "[weiss_hand] Hand data buffer is empty" << std::endl;
        return hand_data_t();
    }
    std::lock_guard<std::mutex> hand_state_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
    hand_data_t result = hand_state_buffer_[0];
    return result;
}

/**
 * @brief get_latest_hand_states: Get the most recent n samples, ordered from newest to oldest
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one samples was returned, otherwise False
*/
bool Hand::get_latest_hand_states(unsigned int n_msgs, std::vector<hand_data_t> &buff) {
  if(!initialized_) {
      std::cout << "[weiss_hand] Not properly initialized, abort" << std::endl;
      return false;
  }
  std::lock_guard<std::mutex> buffer_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
  if(hand_state_buffer_.size() <= 0) {
      std::cout << "[weiss_hand] Hand data buffer is empty" << std::endl;
      return false;
  }
  if(n_msgs <= 0) {
    n_msgs = hand_state_buffer_.size();
  }

  if(n_msgs > hand_state_buffer_.size()) {
    std::cout << "[weiss_hand] Requested " << n_msgs <<" messages, but only returning " << hand_state_buffer_.size() << " latest" << std::endl;
    n_msgs = hand_state_buffer_.size();
  }

  buff.clear();
  for(unsigned int i = 0; i < n_msgs; i++) {
    buff.push_back(hand_state_buffer_[i]);
  }

  return true;
}

/**
 * @brief clear_finger_sample_buffer: Remove previous samples from the sample buffer of one of the fingers
 * @param finger_id: The id of the finger whose data buffer should be cleared
 * @return Whether or not the sample buffer was successfully cleared
*/
bool Hand::clear_finger_sample_buffer(unsigned int finger_id) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting finger buffer clear" << std::endl;
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting finger buffer clear" << std::endl;
        return false;
    }

    if(finger) {
        return finger->clear_sample_buffer();
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active" << std::endl;
        return false;
    }
}

/**
 * @brief get_finger_sample: Get the latest data sample from one of the fingers
 * @param: Id of the finger to get a sample from
 * @return The latest data sample from the finger
*/
WeissFinger::weiss_finger_data_t Hand::get_finger_sample(unsigned int finger_id) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting finger sample" << std::endl;
        return WeissFinger::weiss_finger_data_t();
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting finger sample" << std::endl;
        return WeissFinger::weiss_finger_data_t();
    }

    if(finger) {
        return finger->get_sample();
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active" << std::endl;
        return WeissFinger::weiss_finger_data_t();
    }
}

/**
 * @brief get_latest_finger_samples: Get the most recent n samples, ordered from newest to oldest
 * @param finger_id: The id of the finger to get data from
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one sample was returned, otherwise False
*/
bool Hand::get_latest_finger_samples(unsigned int finger_id, unsigned int n_msgs, std::vector<WeissFinger::weiss_finger_data_t>& buff) {
    if(!initialized_) {
        std::cout << "[weiss_hand] Not properly initialized, aborting finger sample" << std::endl;
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        std::cout << "[weiss_hand] Finger id must be 0 or 1, aborting finger sample" << std::endl;
        return false;
    }

    if(finger) {
        return finger->get_latest_samples(n_msgs, buff);
    } else {
        std::cout << "[weiss_hand] Finger " << finger_id << " is not active" << std::endl;
        return false;
    }
}

/**
 * @brief read_hand_state: Function used by read_hand_thread_
*/
void Hand::read_hand_state() {

    std::cout << "[weiss_hand] Hand state thread started" << std::endl;

    // Prepare messages
    hand_data_t status_msg;
    status_msg.status = "UNKNOWN";

    std::unique_lock<std::mutex> thread_cmd_lock(cmd_mutex_, std::defer_lock);

    double hand_read_period = 1.0/hand_read_rate_;
    while (read_hand_alive_) {
        timespec start = hand_data_t::get_stamp();
        if(!hand_reading_paused_) { // Check if we are paused
          {
              //std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
              while(!thread_cmd_lock.owns_lock()) {
                thread_cmd_lock.try_lock();
              }
              status_msg.stamp = hand_data_t::get_stamp();
              status_msg.width = getOpening();
              status_msg.speed = getSpeed();
              status_msg.force = getForce();
              thread_cmd_lock.unlock();
          }
          {
              std::lock_guard<std::mutex> hand_state_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
              hand_state_buffer_.push_front(status_msg);
              while(hand_state_buffer_.size() > hand_data_buffer_size_) {
                  hand_state_buffer_.pop_back();
              }
          }
        }

        timespec end = hand_data_t::get_stamp();
        long int sleep_length_us = (long int) (1000000 * hand_read_period - ((end.tv_sec + end.tv_nsec/1000000000.0) - (start.tv_sec + start.tv_nsec/1000000000.0)));
        if(sleep_length_us > 0) {
            usleep(sleep_length_us);
        }

    }

    // Disable automatic updates


    std::cout <<"[weiss_hand] Hand state thread ended" << std::endl;
}
