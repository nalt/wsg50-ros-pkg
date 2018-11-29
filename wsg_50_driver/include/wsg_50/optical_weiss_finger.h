#ifndef OPTICALWEISSFINGER_H
#define OPTICALWEISSFINGER_H

#include <stdlib.h>
#include <vector>
#include <mutex>
#include <thread>
#include <deque>
#include "ros/ros.h"
#include "wsg_50/weiss_finger.h"

// The default movement speed in mm/sec
#define DEFAULT_MOVE_SPEED_MM_PER_SEC 30

/**
 * @brief The OpticalWeissFinger class
*/
class OpticalWeissFinger: public WeissFinger{

public:
    // Command indicating transmission/reception of data to/from hand
    static const unsigned char FINGER_CMD_ID = 0xB3;

    /**
     * @brief OpticalWeissFinger: Constructor
     * @param nh: Node handle for grabbing params
     * @param param_prefix: Prefix for param keys
    */
    OpticalWeissFinger(ros::NodeHandle& nh, std::string param_prefix);

    /**
     * @brief ~FMFWeissFinger: Destructor
    */
    ~OpticalWeissFinger();

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

    /**
     * @brief hand_width_to_finger_tip_separation: Computes the distance from this finger to the target on
     *        the other finger.
     * @param width: The current width of the hand
     * @return The distance between the surface of this finger and the target on the other fingertip
    */
    double hand_width_to_finger_tip_separation(double width);
private:

    ros::NodeHandle nh_; // Node handle used to create publisher of data
    bool initialized_; // Status flag, indicates if object has been initialized
    bool can_calibrate_; // Status flag, indicates if object is able to perform calibration
    std::deque<wsg_50_common::WeissFingerData> data_buff_; // Contains the most recent data samples
    std::mutex data_buffer_mutex_; // Mutex that controls access to data_buff_
    std::thread* read_finger_thread_; // Thread for reading data from the finger
    bool read_finger_alive_; // Once thread has started, will shutdown if this is false
    bool paused_; // Indicates if data is being read from the finger
    std::string pub_topic_; // Topic to publish data to

    unsigned int finger_id_; // The id of this finger. Should be 0 or 1
    unsigned int devices_; // The number of optical sensors that this finger contains
    std::vector<uint16_t> reg_addrs_; // Addresses of the optical sensor registers to read
    std::vector<uint8_t> reg_lengths_; // Lengths of the optical sensor registers to read
    double finger_read_rate_; // The rate at which data from the finger should be read
    int data_buff_max_size_; // The maximum size of the data buffer
    double sensor_to_surface_mm_; // The distance between the top of the optical sensor and the surface of the finger
    double target_surface_offset_mm_; // The offset of the target (during calibration) from the surface of the other finger
    std::string calibration_path_; // The file to load/save calibration from/to
    int calib_see_through_samples_; // The number of samples to collect when determining the minimum detection threshold for the optical sensors
    int calib_see_through_target_; // Not actually used
    int calib_offset_samples_; // The number of samples to collect when determining sensor offset
    int calib_offset_target_; // The expected distance from the top of the optical sensor to the target when determining sensor offset
    int calib_cross_talk_samples_; // The number of samples to collect when determining sensor crosstalk
    int calib_cross_talk_target_; // The expected distance from the top of the optical sensor to the target when determining sensor crosstalk

    /**
     * @brief read_finger: Function used by read_finger_thread_
     * @param cmd_mutex: Must acquire this mutex before communicating with hand
    */
    void read_finger(std::mutex *cmd_mutex);

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
    static bool set_reg(unsigned int finger_id, uint16_t reg_addr, uint8_t reg_length, unsigned char* payload, uint8_t* payload_length);

    /**
     * @brief add_reg: Generate command to add a register to be read from optical sensor
     * @param finger_id: The id of this finger
     * @param reg_addr: The address of the register to read from
     * @param reg_length: The length of the register to read from
     * @param payload: Container for the command
     * @param payload_length: Length of the resulting command
     * @return Whether or not this command was successfully constructed
    */
    static bool add_reg(unsigned int finger_id, uint16_t reg_addr, uint8_t reg_length, unsigned char* payload, uint8_t* payload_length);

    /**
     * @brief del_reg: Generate command to stop reading an optical sensor register
     * @param finger_id: The id of this finger
     * @param reg_addr: The address of the register that should no longer be read
     * @param reg_length: The length of the register that should no longer be read
     * @param payload: Container for the command
     * @param payload_length: Length of the resulting command
     * @return Whether or not this command was successfully constructed
    */
    static bool del_reg(unsigned int finger_id, uint16_t reg_addr, unsigned char* payload, uint8_t* payload_length);

    /**
     * @brief read_reg: Generate command to read all of the current registers
     * @param finger_id: The id of this finger
     * @param devices: The number of optical sensors this finger has
     * @param reg_lengths: A vector containing the length of each register to be read
     * @param payload: Container for the command
     * @param payload_length: Length of the resulting command
     * @return Whether or not this command was successfully constructed
    */
    static bool read_reg(unsigned int finger_id, unsigned int devices, std::vector<uint8_t>& reg_lengths, unsigned char* payload, uint8_t* payload_length);

    /**
     * @brief write_reg: Generate command to write to a register of a specific optical sensor
     * @param finger_id: The id of this finger
     * @param device_id: The id of the optical sensor
     * @param tx_data: A vector of bytes to write to the register
     * @param payload: Container for the command
     * @param payload_length: Length of the resulting command
     * @return Whether or not this command was successfully constructed
    */
    static bool write_reg(unsigned int finger_id, uint8_t device_id, const std::vector<uint8_t>& tx_data, unsigned char* payload, uint8_t* payload_length);

    /**
     * @brief calib_result: Generate command to get the result of the most recent calibration
     * @param finger_id: The id of this finger
     * @param devices: The number of optica sensors this finger has
     * @param payload: Container for the command
     * @param payload_length: Length of the resulting command
     * @return Whether or not this command was successfully constructed
    */
    static bool calib_result(unsigned int finger_id, unsigned int devices, unsigned char* payload, uint8_t* payload_length);

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
    static bool calib_see_through(unsigned int finger_id, uint8_t calib_see_through_samples, uint8_t calib_see_through_target,
                                  double sensor_to_surface_mm, double target_surface_offset_mm,
                                  double* target_width_mm, unsigned char* payload, uint8_t* payload_length);

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
    static bool calib_offset(unsigned int finger_id, uint8_t calib_offset_samples, uint8_t calib_offset_target,
                             double sensor_to_surface_mm, double target_surface_offset_mm,
                             double* target_width_mm, unsigned char* payload, uint8_t* payload_length);

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
    static bool calib_cross_talk(unsigned int finger_id, uint8_t calib_cross_talk_samples, uint8_t calib_cross_talk_target,
                                 double sensor_to_surface_mm, double target_surface_offset_mm,
                                 double* target_width_mm, unsigned char* payload, uint8_t* payload_length);

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
    static bool save_cal_data(std::string path,
                              unsigned int devices,
                              unsigned char* see_through_result,
                              unsigned char* offset_result,
                              unsigned char* cross_talk_result);

    /**
     * @brief load_cal_data: Opens the calibration data and constructs commands to upload them to the sensor
     * @param path: The file that calibration data should be loaded from
     * @param finger_id: The id of this finger
     * @param calib_payloads: Vector of commands that should be sent to the hand in order to upload the calibration
     * @return Whether or not the commands were successfully constructed
    */
    static bool load_cal_data(std::string path,
                              unsigned int finger_id,
                              std::vector<std::vector<unsigned char>>& calib_payloads);


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
    static bool verify_cal_data(std::string path,
                                unsigned int finger_id,
                                unsigned int devices,
                                unsigned char* verify_payload,
                                uint8_t* verify_payload_length,
                                unsigned char* expected_resp,
                                uint8_t* expected_resp_length);

    /**
     * @brief cmd_response_to_msg: De-serialize sensor data from hand into a message
     * @param resp: The response received from reading data
     * @param resp_len: The length of the response
     * @param devices: The number of optical sensors in the finger
     * @param reg_lengths: The length of each register that was read
     * @param stamp: The stamp to use for the returned message, defaults to the time at which function was called
     * @return Message with data read from the hand
    */
    static  wsg_50_common::WeissFingerData cmd_response_to_msg(unsigned char* resp, unsigned int resp_len,
                                                               unsigned int devices, std::vector<uint8_t>& reg_lengths,
                                                               ros::Time stamp=ros::Time::now());

    /**
     * @brief open_cal_data: Load calibration data from file
     * @param path: The path to the file containing calibration data
     * @param calib_data: Container for the calibration data
     * @return Whether or not calibration data was successfully loaded
    */
    static bool open_cal_data(std::string path, std::vector<std::vector<unsigned char> >& calib_data);

    /**
     * @brief move_hand: Command the hand to move
     * @param cmd_mutex: Must acquire this mutex before communicating with hand
     * @param width: The hand width to move to
     * @param speed: The speed with which to move
     * @return True if hand movement was successful, else False
     */
    static bool move_hand(std::mutex& cmd_mutex, double width, unsigned int speed=DEFAULT_MOVE_SPEED_MM_PER_SEC);

    /**
     * @brief remove_finger_measurement_offset: Given a raw range measurement from an optical sensor,
     *                                          converts to the measured distance from the surface of the finger
     * @param measurement: Raw range measurement
     * @return The distance measured from the surface of the finger to the object
    */
    double remove_finger_measurement_offset(double measurement);

};

#endif // OPTICALWEISSFINGER_H
