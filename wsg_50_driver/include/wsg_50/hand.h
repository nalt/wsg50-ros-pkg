#ifndef HAND_H
#define HAND_H

#include <mutex>
#include <thread>
#include <deque>
#include "wsg_50/weiss_finger.h"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/WeissFingerData.h"

/**
 * @brief The Hand class
*/
class Hand {

public:
    // The default movement speed in mm/sec
    static const int DEFAULT_MOVE_SPEED_MM_PER_SEC = 30;

    // Command to read data from the hand
    static const unsigned char MOVE_HAND_CMD_ID = 0xB0;

    // Threshold for determining if the hand is moving or not
    static constexpr double NOT_MOVING_SPEED_THRESH = 0.01;

    /**
     * @brief Hand: Constructor
     * @param nh: Node handle for grabbing params
     * @param param_prefix: Prefix for param keys
    */
    Hand(ros::NodeHandle& nh, std::string param_prefix);

    /**
     * @brief ~Hand: Destructor
    */
    ~Hand();

    /**
     * @brief do_calibration: Initiate calibration of one of the fingers
     * @param finger_id: The id of the finger to be calibrated
     * @return Whether or not calibration was successful
    */
    bool do_calibration(unsigned int finger_id);

    /**
     * @brief load_calibration: Load calibration data into one of the fingers
     * @param finger_id: The id of the finger to load calibration data into
     * @return Whether or not loading calibration was successful
    */
    bool load_calibration(unsigned int finger_id);

    /**
     * @brief start_reading: Initializes and begins reading from this hand and both of its finger
     * @return Whether or not reading was successfully started
    */
    bool start_reading();

    /**
     * @brief pause_hand_reading: Pause reading from the hand (only)
     * @return Whether or not reading was successfully paused
    */
    bool pause_hand_reading();

    /**
     * @brief restart_hand_reading: Restart reading from the hand (only)
     * @return Whether or not reading was successfully restarted
    */
    bool restart_hand_reading();

    /**
     * @brief pause_finger_reading: Pause reading from one of the fingers
     * @param finger_id: Id of finger that should pause reading
     * @return  Whether or not reading was successfully paused
    */
    bool pause_finger_reading(unsigned int finger_id);

    /**
     * @brief restart_finger_reading
     * @param finger_id
     * @return
    */
    bool restart_finger_reading(unsigned int finger_id);

    /**
     * @brief move_hand: Command the hand to move
     * @param width: The hand width to move to
     * @param speed: The speed with which to move
     * @return True if hand movement was successful, else False
    */
    bool move_hand(float width, float speed=DEFAULT_MOVE_SPEED_MM_PER_SEC, bool ignore_response=false);

    /**
     * @brief hand_is_moving: Reports whether the hand is currently moving
     * @return True if the hand is moving, else false
    */
    bool hand_is_moving();

    /**
     * @brief stop_hand: Tell the hand to stop moving. (TODO: Blocks until hand stops?)
     * @return If the stop command was succesfully sent
    */
    bool stop_hand();

    /**
     * @brief clear_hand_state_buffer: Remove previous hand state samples from the buffer
     * @return Whether or not the buffer was succesfully cleared
    */
    bool clear_hand_state_buffer();

    /**
     * @brief get_hand_state: Get the latest hand state
     * @return  The latest hand state
    */
    wsg_50_common::Status get_hand_state();

    /**
     * @brief get_hand_state: Get the latest hand state
     * @param hand_state: Container for the latest hand state
     * @return  Whether the latest hand state was retrieved
    */
    bool get_hand_state(wsg_50_common::Status& hand_state);

    /**
     * @brief get_latest_hand_states: Get the most recent n samples, ordered from newest to oldest
     * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
     * @param buff: Container for the n samples
     * @return True if at least one samples was returned, otherwise False
    */
    bool get_latest_hand_states(unsigned int n_msgs, std::vector<wsg_50_common::Status>& buff);

    /**
     * @brief clear_finger_sample_buffer: Remove previous samples from the sample buffer of one of the fingers
     * @param finger_id: The id of the finger whose data buffer should be cleared
     * @return Whether or not the sample buffer was successfully cleared
    */
    bool clear_finger_sample_buffer(unsigned int finger_id);

    /**
     * @brief get_finger_sample: Get the latest data sample from one of the fingers
     * @param: Id of the finger to get a sample from
     * @return The latest data sample from the finger
    */
    wsg_50_common::WeissFingerData get_finger_sample(unsigned int finger_id);

    /**
     * @brief get_latest_finger_samples: Get the most recent n samples, ordered from newest to oldest
     * @param finger_id: The id of the finger to get data from
     * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
     * @param buff: Container for the n samples
     * @return True if at least one sample was returned, otherwise False
    */
    bool get_latest_finger_samples(unsigned int finger_id, unsigned int n_msgs, std::vector<wsg_50_common::WeissFingerData>& buff);

    WeissFinger* finger0_; // Pointer to finger 0
    WeissFinger* finger1_; // Pointer to finger 1

private:

    /**
     * @brief read_hand_state: Function used by read_hand_thread_
    */
    void read_hand_state();

    bool initialized_; // Status flag, indicates if object has been initialized

    std::mutex cmd_mutex_; // Controls access to communication with the hand
    std::unique_lock<std::mutex> cmd_lock_; // A persistent lock for cmd_mutex_
    std::deque<wsg_50_common::Status> hand_state_buffer_; // Contains the most recent hand states
    std::mutex hand_state_buffer_mutex_; // Controls access to hand_state_buffer_

    std::thread* read_hand_thread_; // Thread for reading data from the hand
    bool read_hand_alive_; // Indicates that read_hand_thread_ is running
    bool hand_reading_paused_; // Indicates if data if being read from the hand

    int hand_state_buffer_max_size_; // The maximum size of the data buffer
    double hand_read_rate_; // The rate at which hand state data should be read

};

#endif // HAND_H
