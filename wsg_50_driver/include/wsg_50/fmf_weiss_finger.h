#ifndef FMF_WEISS_FINGER_H
#define FMF_WEISS_FINGER_H

#include <stdlib.h>
#include <vector>
#include <mutex>
#include <thread>
#include <deque>
#include "wsg_50/weiss_finger.h"
#include "ros/ros.h"

// The default movement speed in mm/sec
#define DEFAULT_MOVE_SPEED_MM_PER_SEC 30

/**
 * @brief The FMFWeissFinger class
*/
class FMFWeissFinger: public WeissFinger {
public:
    /**
     * @brief FMFWeissFinger: Constructor
     * @param nh: Node handle for grabbing params
     * @param param_prefix: Prefix for param keys
    */
    FMFWeissFinger(ros::NodeHandle& nh, std::string param_prefix);

    /**
     * @brief ~FMFWeissFinger: Destructor
    */
    ~FMFWeissFinger();

    /**
     * @brief do_calibration: Performs calibration
     * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
     * @return Whether or not the calibration was successful
    */
    bool do_calibration(std::mutex* cmd_mutex);

    /**
     * @brief load_calibration: Load calibration, including sending params to hand if necessary
     * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
     * @return Whether or not the calibration was successfully loaded
    */
    bool load_calibration(std::mutex* cmd_mutex);

    /**
     * @brief start_reading: Initializes and begins reading from this finger
     * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
     * @return Whether or not reading was successfully started
    */
    bool start_reading(std::mutex* cmd_mutex);

    /**
     * @brief pause_reading: Stop reading data from this finger. Should only be called
     * after start_reading()
     * @return Whether or not reading was successfully paused
    */
    bool pause_reading();

    /**
     * @brief restart_reading: Resume reading data from this finger. Should only be
     * called after pause_reading().
     * @return  Whether or not not reading was successfully resumed.
    */
    bool restart_reading();

    /**
     * @brief clear_sample_buffer: Remove previous samples from the sample buffer
     * @return Whether or not the sample buffer was successfully cleared
    */
    bool clear_sample_buffer();

    /**
     * @brief get_sample: Get the latest data sample
     * @return The latest data sample
    */
    wsg_50_common::WeissFingerData get_sample();

    /**
     * @brief get_latest_samples: Get the most recent n samples, ordered from newest to oldest
     * @param n_msgs: The number of samples to get. Will return all samples if less than or equal to zero
     * @param buff: Container for the n samples
     * @return True if at least one sample was returned, otherwise False
    */
    bool get_latest_samples(unsigned int n_msgs, std::vector<wsg_50_common::WeissFingerData>& buff);

private:
    static const int FINGER0_READ_CMD = 0x63; // Command to read data from Weiss finger 0 (non-generic finger only)
    static const int FINGER1_READ_CMD = 0x73; // Command to read data from Weiss finger 1 (non-generic finger only)

    bool initialized_; // Status flag, indicates if object has been initialized
    bool can_calibrate_; // Status flag, indicates if object is able to perform calibration
    std::deque<wsg_50_common::WeissFingerData> data_buff_; // Contains the most recent data samples
    std::mutex data_buffer_mutex_; // Mutex that controls access to data_buff_
    std::thread* read_finger_thread_; // Thread for reading data from the finger
    bool read_finger_alive_; // Once thread has started, will shutdown if this is false
    bool paused_; // Indicates if data is being read from the finger
    double force_offset_; // Calibration value, the tare value to calibrate the zero point

    // Parameters read in from hand.yaml
    unsigned int finger_id_; // The id of this finger. Should be 0 or 1
    double finger_read_rate_; // The rate at which data from the finger should be read
    int data_buff_max_size_; // The maximum size of the data buffer
    std::string calibration_path_; // The file to load/save calibration data from/to
    int calib_samples_; // The number of samples to collect when performing calibration
    double calib_target_; // The calibration target

    /**
     * @brief bytes_to_float: Converts a number represented by four bytes into a float
     * @param b0 First byte, lsb
     * @param b1
     * @param b2
     * @param b3 Last byte, msb
     * @return Float representation of number
    */
    float bytes_to_float(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3);

    /**
     * @brief get_data: Get one data sample from the finger
     * @param cmd_mutex: Must acquire this mutex before communicating with hand
     * @param data: Container for data from finger
     * @return True if data was successfully retrieved, else False
    */
    bool get_data(std::mutex& cmd_mutex, std::vector<double>& data);

    /**
     * @brief read_finger: Function used by read_finger_thread_
     * @param cmd_mutex: Must acquire this mutex before communicating with hand
    */
    void read_finger(std::mutex* cmd_mutex);

    /**
     * @brief move_hand: Command the hand to move
     * @param cmd_mutex: Must acquire this mutex before communicating with hand
     * @param width: The hand width to move to
     * @param speed: The speed with which to move
     * @return True if hand movement was successful, else False
     */
    static bool move_hand(std::mutex& cmd_mutex, double width, unsigned int speed=DEFAULT_MOVE_SPEED_MM_PER_SEC);
};

#endif // FMF_WEISS_FINGER_H
