#ifndef WEISS_FINGER_H
#define WEISS_FINGER_H

#include <mutex>
#include "wsg_50_common/WeissFingerData.h"

/**
 * @brief WeissFinger: Abstract class that all fingers should implement
*/
class WeissFinger {
public:
    /**
     * @brief WeissFinger: Contrstructs the Weiss Finger
    */
    WeissFinger() {}

    /**
     * @brief ~WeissFinger: Destroys the Weiss Finger
    */
    virtual ~WeissFinger() {}

    /**
     * @brief do_calibration: Performs calibration
     * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
     * @return Whether or not the calibration was successful
    */
    virtual bool do_calibration(std::mutex* cmd_mutex) = 0;

    /**
     * @brief load_calibration: Load calibration, including sending params to hand if necessary
     * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
     * @return Whether or not the calibration was successfully loaded
    */
    virtual bool load_calibration(std::mutex* cmd_mutex) = 0;

    /**
     * @brief start_reading: Initializes and begins reading from this finger
     * @param cmd_mutex: Need to acquire this mutex before sending commands to hand
     * @return Whether or not reading was successfully started
    */
    virtual bool start_reading(std::mutex* cmd_mutex) = 0;

    /**
     * @brief pause_reading: Stop reading data from this finger. Should only be called
     * after start_reading()
     * @return Whether or not reading was successfully paused
    */
    virtual bool pause_reading() = 0;

    /**
     * @brief restart_reading: Resume reading data from this finger. Should only be
     * called after pause_reading().
     * @return  Whether or not not reading was successfully resumed.
    */
    virtual bool restart_reading() = 0;

    /**
     * @brief clear_sample_buffer: Remove previous samples from the sample buffer
     * @return Whether or not the sample buffer was successfully cleared
    */
    virtual bool clear_sample_buffer() = 0;

    /**
     * @brief get_sample: Get the latest data sample
     * @return The latest data sample
    */
    virtual wsg_50_common::WeissFingerData get_sample() = 0;

    /**
     * @brief get_latest_samples: Get the most recent n samples, ordered from newest to oldest
     * @param n_msgs: The number of samples to get
     * @param buff: Container for the n samples
     * @return True if at least one sample was returned, otherwise False
    */
    virtual bool get_latest_samples(unsigned int n_msgs, std::vector<wsg_50_common::WeissFingerData>& buff)=0;
};

#endif // WEISS_FINGER_H
