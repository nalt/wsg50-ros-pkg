#ifndef HAND_H
#define HAND_H

#ifndef HAS_CLOCK_GETTIME
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)
#endif

#include <mutex>
#include <thread>
#include <deque>
#include <ctime>
#include "wsg_50/weiss_finger.h"
#include "wsg_50/optical_weiss_finger.h"
#include "wsg_50/fmf_weiss_finger.h"

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
     * @brief hand_params_t: Contains parameters for initializing the hand
    **/
    struct hand_params_t {
        std::string ip; // The ip address of the hand
        int port;   // The port to access the hand on
        int local_port; // Port on local computer hand can use to send data (UDP only)
        bool use_tcp; // Whether to use tcp or udp
        std::string finger0_type; // The type of finger 0. Should be 'optical', 'fmf', or 'none'
        std::string finger1_type; // The type of finger 1. Should be 'optical', 'fmf', or 'none'
        int hand_data_buffer_size; // The number of hand state messages to buffer
        double hand_read_rate; // The rate at which to read status data from the hand
    };
    /**
     * @brief hand_data_t: Contains data about the state of the hand
    **/
    struct hand_data_t {
        std::string status; // A string describing the status of the hand
        timespec stamp; // A timestamp indicating when this data was generated
        float width; // The position of the hand
        float speed; // The speed of the hand
        float acc; // The acceleration of the hand
        float force; // The force (estimated by current drawn) applied by the hand

        /**
         * @brief get_stamp: Get the current time
         * @return A timespec that describes the current time
        */
        static timespec get_stamp() {
            timespec stamp;
            #if HAS_CLOCK_GETTIME
            clock_gettime(CLOCK_REALTIME, &stamp);
            #else
            struct timeval timeofday;
            gettimeofday(&timeofday,NULL);
            stamp.tv_sec = timeofday.tv_sec;
            stamp.tv_nsec = timeofday.tv_usec * 1000;
            #endif
            return stamp;
        }
    };

    /**
     * @brief Hand: Constructor
     * @param hand_params: Struct containing hand initialization params
     * @param finger0_params: Struct containing finger0 initialization params
     * @param finger1_params: Struct containing finger1 initialization params
    */
    Hand(hand_params_t* hand_params,
         WeissFinger::weiss_finger_params_t* finger0_params,
         WeissFinger::weiss_finger_params_t* finger1_params);

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
    hand_data_t get_hand_state();

    /**
     * @brief get_hand_state: Get the latest hand state
     * @param hand_state: Container for the latest hand state
     * @return  Whether the latest hand state was retrieved
    */
    bool get_hand_state(hand_data_t& hand_state);

    /**
     * @brief get_latest_hand_states: Get the most recent n samples, ordered from newest to oldest
     * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
     * @param buff: Container for the n samples
     * @return True if at least one samples was returned, otherwise False
    */
    bool get_latest_hand_states(unsigned int n_msgs, std::vector<hand_data_t>& buff);

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
    WeissFinger::weiss_finger_data_t get_finger_sample(unsigned int finger_id);

    /**
     * @brief get_latest_finger_samples: Get the most recent n samples, ordered from newest to oldest
     * @param finger_id: The id of the finger to get data from
     * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
     * @param buff: Container for the n samples
     * @return True if at least one sample was returned, otherwise False
    */
    bool get_latest_finger_samples(unsigned int finger_id, unsigned int n_msgs, std::vector<WeissFinger::weiss_finger_data_t>& buff);

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
    std::deque<hand_data_t> hand_state_buffer_; // Contains the most recent hand states
    std::mutex hand_state_buffer_mutex_; // Controls access to hand_state_buffer_

    std::thread* read_hand_thread_; // Thread for reading data from the hand
    bool read_hand_alive_; // Indicates that read_hand_thread_ is running
    bool hand_reading_paused_; // Indicates if data if being read from the hand

    int hand_data_buffer_size_; // The maximum size of the data buffer
    double hand_read_rate_; // The rate at which hand state data should be read

};

#endif // HAND_H
