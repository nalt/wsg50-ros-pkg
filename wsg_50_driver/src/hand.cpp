#include <ros/ros.h>
#include <limits>
#include "wsg_50/hand.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"
#include "wsg_50/cmd.h"
#include "wsg_50/optical_weiss_finger.h"
#include "wsg_50/fmf_weiss_finger.h"

/**
 * @brief Hand: Constructor
 * @param nh: Node handle for grabbing params
 * @param param_prefix: Prefix for param keys
*/
Hand::Hand(ros::NodeHandle& nh, std::string param_prefix):
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
    std::string ip("");
    int port, local_port;
    bool use_tcp;
    std::string finger0_type, finger1_type;
    if(!nh.getParam(param_prefix+"/ip", ip)) {
        ROS_ERROR("Did not get hand ip, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/port", port)) {
        ROS_ERROR("Did not get hand port, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/local_port", local_port)) {
        ROS_ERROR("Did not get hand local port, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/use_tcp", use_tcp)) {
        ROS_ERROR("Did not get hand comm method, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/finger0_type", finger0_type)) {
        ROS_ERROR("Did not get type of finger 0, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/finger1_type", finger1_type)) {
        ROS_ERROR("Did not get type of finger 1, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/buffer_size_hand_state", hand_state_buffer_max_size_)) {
        ROS_ERROR("Did not get hand state buffer max size, aborting");
        return;
    }
    if(!nh.getParam(param_prefix+"/hand_read_rate", hand_read_rate_)) {
        ROS_ERROR("Did not get hand read rate, aborting");
        return;
    }

    // Setup connection to hand
    int res_con;
    if (use_tcp) {
        res_con = cmd_connect_tcp( ip.c_str(), port );
    } else {
        res_con = cmd_connect_udp(local_port, ip.c_str(), port );
    }

    if (res_con == 0 ) {
         ROS_INFO("Gripper connection stablished");

         ROS_INFO("Ready to use. Homing and taring now...");
         homing();
         ros::Duration(0.5).sleep();
         //doTare();

         // Configure the fingers
         if(finger0_type.compare("optical") == 0) {
             ROS_INFO("Preparing finger 0 as an optical sensor");
             finger0_ = new OpticalWeissFinger(nh, param_prefix+"/finger0");
         } else if(finger0_type.compare("fmf") == 0) {
             ROS_INFO("Preparing finger 0 as an fmf sensor");
             finger0_ = new FMFWeissFinger(nh, param_prefix+"/finger0");
         } else if(finger0_type.compare("dsa") == 0) {
             ROS_ERROR("DSA finger not implemented, finger 0 will not be active");
         } else {
             ROS_INFO("Finger 0 is not active");
         }

         if(finger1_type.compare("optical") == 0) {
             ROS_INFO("Preparing finger 1 as an optical sensor");
             finger1_ = new OpticalWeissFinger(nh, param_prefix+"/finger1");
         } else if(finger1_type.compare("fmf") == 0) {
             ROS_INFO("Preparing finger 1 as an fmf sensor");
             finger1_ = new FMFWeissFinger(nh, param_prefix+"/finger1");
         } else if(finger1_type.compare("dsa") == 0) {
             ROS_ERROR("DSA finger not implemented, finger 1 will not be active");
         } else {
             ROS_INFO("Finger 1 will not be active");
         }

         initialized_ = true;

     } else {
         ROS_ERROR("Unable to connect, please check the port and address used.");
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
        ROS_ERROR("Not properly initialized, aborting calibration");
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting calibration");
        return false;
    }

    if(finger) {
        finger->do_calibration(&cmd_mutex_);
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
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
        ROS_ERROR("Not properly initialized, aborting calibration load");
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting calibration");
        return false;
    }

    if(finger) {
        finger->load_calibration(&cmd_mutex_);
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
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
        ROS_ERROR("Not properly initialized, aborting read commencement");
        return false;
    }

    // Start reading from the hand
    read_hand_alive_ = true;
    hand_reading_paused_ = false;
    read_hand_thread_ = new std::thread(&Hand::read_hand_state, this);

    // Start reading from the fingers
    bool success = true;
    ROS_INFO("Going to start reading finger 0");
    if(finger0_) {
        if(!finger0_->start_reading(&cmd_mutex_)) {
            success = false;
        }
    }
    ROS_INFO("Going to start reading finger 1");
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
    ROS_ERROR("Not properly initialized, aborting pause");
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
    ROS_ERROR("Not properly initialized, aborting read restart");
    return false;
  }
  if(!read_hand_alive_) {
    ROS_ERROR("Hand reading never started, call start reading first, abort");
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
        ROS_ERROR("Not properly initialized, aborting finger pause");
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting finger pause");
        return false;
    }

    if(finger) {
        return finger->pause_reading();
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
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
        ROS_ERROR("Not properly initialized, aborting finger restart");
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting finger restart");
        return false;
    }

    if(finger) {
        return finger->restart_reading();
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
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
        ROS_ERROR("Not properly initialized, aborting hand movement");
        return false;
    }
    int res;
    if ( (width >= 0.0 && width <= 110.0) && (speed > 0.0 && speed <= 420.0) ){

    }else if (width < 0.0 || width > 110.0){
        ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - 110.0] ");
        return false;
    }else{
        ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
        //std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);

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

  wsg_50_common::Status hand_state;
  while(!get_hand_state(hand_state)) { // Wait until we get hand data
    ros::Duration(0.1).sleep();
  }

  return std::abs(hand_state.speed) > Hand::NOT_MOVING_SPEED_THRESH;

}

/**
 * @brief stop_hand: Tell the hand to stop moving. (TODO: Blocks until hand stops?)
 * @return If the stop command was succesfully sent
*/
bool Hand::stop_hand() {

    if(!initialized_) {
        ROS_ERROR("Not properly initialized, aborting hand movement");
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
      ROS_ERROR("Not properly initialized, abort");
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
bool Hand::get_hand_state(wsg_50_common::Status& hand_state) {

    if(!initialized_) {
        ROS_ERROR("Not properly initialized, aborting hand state");
        return false;
    }

    if(hand_state_buffer_.size() <= 0) {
        ROS_ERROR("Hand data buffer is empty");
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
wsg_50_common::Status Hand::get_hand_state() {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, aborting hand state");
        return wsg_50_common::Status();;
    }

    if(hand_state_buffer_.size() <= 0) {
        ROS_ERROR("Hand data buffer is empty");
        return wsg_50_common::Status();
    }
    std::lock_guard<std::mutex> hand_state_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
    wsg_50_common::Status result = hand_state_buffer_[0];
    return result;
}

/**
 * @brief get_latest_hand_states: Get the most recent n samples, ordered from newest to oldest
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one samples was returned, otherwise False
*/
bool Hand::get_latest_hand_states(unsigned int n_msgs, std::vector<wsg_50_common::Status>& buff) {
  if(!initialized_) {
      ROS_ERROR("Not properly initialized, abort");
      return false;
  }
  std::lock_guard<std::mutex> buffer_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
  if(hand_state_buffer_.size() <= 0) {
      ROS_ERROR("Finger data buffer is empty");
      return false;
  }
  if(n_msgs <= 0) {
    n_msgs = hand_state_buffer_.size();
  }

  if(n_msgs > hand_state_buffer_.size()) {
    ROS_WARN("Requested %u messages, but only returning %lu latest", n_msgs, hand_state_buffer_.size());
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
        ROS_ERROR("Not properly initialized, aborting finger buffer clear");
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting finger buffer clear");
        return false;
    }

    if(finger) {
        return finger->clear_sample_buffer();
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
        return false;
    }
}

/**
 * @brief get_finger_sample: Get the latest data sample from one of the fingers
 * @param: Id of the finger to get a sample from
 * @return The latest data sample from the finger
*/
wsg_50_common::WeissFingerData Hand::get_finger_sample(unsigned int finger_id) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, aborting finger sample");
        return wsg_50_common::WeissFingerData();
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting finger sample");
        return wsg_50_common::WeissFingerData();
    }

    if(finger) {
        return finger->get_sample();
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
        return wsg_50_common::WeissFingerData();
    }
}

/**
 * @brief get_latest_finger_samples: Get the most recent n samples, ordered from newest to oldest
 * @param finger_id: The id of the finger to get data from
 * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
 * @param buff: Container for the n samples
 * @return True if at least one sample was returned, otherwise False
*/
bool Hand::get_latest_finger_samples(unsigned int finger_id, unsigned int n_msgs, std::vector<wsg_50_common::WeissFingerData>& buff) {
    if(!initialized_) {
        ROS_ERROR("Not properly initialized, aborting finger sample");
        return false;
    }

    WeissFinger* finger;
    if(finger_id == 0) {
        finger = finger0_;
    } else if(finger_id == 1) {
        finger = finger1_;
    } else {
        ROS_ERROR("Finger id must be 0 or 1, aborting finger sample");
        return false;
    }

    if(finger) {
        return finger->get_latest_samples(n_msgs, buff);
    } else {
        ROS_ERROR("Finger %d is not active", finger_id);
        return false;
    }
}

/**
 * @brief read_hand_state: Function used by read_hand_thread_
*/
void Hand::read_hand_state() {

    ROS_INFO("Hand state thread started");

    // Prepare messages
    wsg_50_common::Status status_msg;
    status_msg.status = "UNKNOWN";

    std::unique_lock<std::mutex> thread_cmd_lock(cmd_mutex_, std::defer_lock);

    ros::Rate rate(hand_read_rate_);
    while (ros::ok() && read_hand_alive_) {
        if(!hand_reading_paused_) { // Check if we are paused
          {
              //std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
              while(!thread_cmd_lock.owns_lock()) {
                thread_cmd_lock.try_lock();
              }
              status_msg.stamp = ros::Time::now();
              status_msg.width = getOpening();
              status_msg.speed = getSpeed();
              status_msg.force = getForce();
              thread_cmd_lock.unlock();
          }
          {
              std::lock_guard<std::mutex> hand_state_lock(hand_state_buffer_mutex_); // Will be released upon losing scope
              hand_state_buffer_.push_front(status_msg);
              while(hand_state_buffer_.size() > hand_state_buffer_max_size_) {
                  hand_state_buffer_.pop_back();
              }
          }
        }
        rate.sleep();

    }

    // Disable automatic updates


    ROS_INFO("Hand state thread ended");
}

#include <iostream>
#include <fstream>
int main(int argc, char** argv) {

    ros::init(argc, argv, "test_hand");
    ros::NodeHandle nh;

    Hand hand(nh, "hand0");

    //hand.do_calibration(0);
    //hand.do_calibration(1);

    hand.load_calibration(0);
    hand.load_calibration(1);

    bool reading = hand.start_reading();
    ROS_INFO("Hand is ready: %d", reading);
    ros::Duration(1.0).sleep();

    /*
    ROS_INFO("Telling the hand to move...");
    hand.move_hand(10,50,true);
    ROS_INFO("...returned");


    hand.clear_hand_state_buffer();
    ros::Duration(0.25).sleep();
    while(hand.hand_is_moving()){
      std::cout << "Hand is moving" << std::endl;
      ros::Duration(0.1).sleep();
    }
    std::cout << "Hand has finished moving" << std::endl;
    */
    ros::Rate rate(30);
    while(ros::ok()) {


      wsg_50_common::WeissFingerData finger0_sample = hand.get_finger_sample(0);

      ROS_INFO("Range %d, intensity %d",
                 (int)finger0_sample.data[7*finger0_sample.data_shape[1]+0],
                 (int)finger0_sample.data[7*finger0_sample.data_shape[1]+1]);

  /*
        unsigned int dev_idx = 7;
        wsg_50_common::Status hand_status = hand.get_hand_state();
        wsg_50_common::WeissFingerData finger0_sample = hand.get_finger_sample(0);
        wsg_50_common::WeissFingerData finger1_sample = hand.get_finger_sample(1);

        //ROS_INFO("Hand status: Pos %f, Speed %f, Force %f", hand_status.width, hand_status.speed, hand_status.force);

        if(finger0_sample.data_shape.size() >= 2 && finger0_sample.data.size() > dev_idx*finger0_sample.data_shape[1]+4) {
          ROS_INFO("Range %d, Return Rate %d, Ref Rate %d, Amb Count %d, Ambient Light %d",
                 (int)finger0_sample.data[dev_idx*finger0_sample.data_shape[1]+0],
                 (int)finger0_sample.data[dev_idx*finger0_sample.data_shape[1]+1],
                 (int)finger0_sample.data[dev_idx*finger0_sample.data_shape[1]+2],
                 (int)finger0_sample.data[dev_idx*finger0_sample.data_shape[1]+3],
                 (int)finger0_sample.data[dev_idx*finger0_sample.data_shape[1]+4]);
        }

        if(finger1_sample.data_shape.size() >= 2 && finger1_sample.data.size() > dev_idx*finger1_sample.data_shape[1]+4) {
          ROS_WARN("Range %d, Return Rate %d, Ref Rate %d, Amb Count %d, Ambient Light %d",
                 (int)finger1_sample.data[dev_idx*finger1_sample.data_shape[1]+0],
                 (int)finger1_sample.data[dev_idx*finger1_sample.data_shape[1]+1],
                 (int)finger1_sample.data[dev_idx*finger1_sample.data_shape[1]+2],
                 (int)finger1_sample.data[dev_idx*finger1_sample.data_shape[1]+3],
                 (int)finger1_sample.data[dev_idx*finger1_sample.data_shape[1]+4]);
        }
        */
        rate.sleep();
    }


}
