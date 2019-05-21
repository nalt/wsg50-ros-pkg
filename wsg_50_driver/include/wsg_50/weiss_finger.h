#ifndef WEISS_FINGER_H
#define WEISS_FINGER_H

#ifndef HAS_CLOCK_GETTIME
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)
#endif

#include <mutex>
#include <ctime>
#include <vector>
/**
 * @brief WeissFinger: Abstract class that all fingers should implement
*/
class WeissFinger {
public:

    /**
     * @brief weiss_finger_params_t: Parameters for initializing a Weiss Finger
    **/
    struct weiss_finger_params_t{
        unsigned int finger_id; // The id of the finger
        double finger_read_rate; // The rate at which to read data from the finger
        int finger_data_buffer_size;

    };

    /**
      * @brief weiss_finger_data_t: Data from the Weiss Finger
    **/
    struct weiss_finger_data_t {
        timespec stamp;
        std::vector<float> data;
        std::vector<int> data_shape;

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
     * @brief WeissFinger: Constructs the Weiss Finger
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
    virtual weiss_finger_data_t get_sample() = 0;

    /**
     * @brief get_latest_samples: Get the most recent n samples, ordered from newest to oldest
     * @param n_msgs: The number of samples to get
     * @param buff: Container for the n samples
     * @return True if at least one sample was returned, otherwise False
    */
    virtual bool get_latest_samples(unsigned int n_msgs, std::vector<weiss_finger_data_t>& buff)=0;
};

#endif // WEISS_FINGER_H
