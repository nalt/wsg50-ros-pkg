#include <ros/ros.h>
#include "wsg_50/hand.h"
#include "wsg_50_common/Status.h"

#include <iostream>
#include <fstream>

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_hand_read");

    ros::NodeHandle nh;

    Hand hand(nh, "hand0");
    hand.start_reading();
    hand.pause_hand_reading();
    hand.pause_finger_reading(1);

    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(30);
    while(ros::ok()) {
        wsg_50_common::WeissFingerData finger0_sample = hand.get_finger_sample(0);

        unsigned int print_length = finger0_sample.data_shape.size() > 1 ? finger0_sample.data_shape[1] : finger0_sample.data_shape[0];
        for(unsigned int i = 0; i < print_length; i++) {
            std::cout << finger0_sample.data[i] << " ";
        }
        std::cout << std::endl;

        loop_rate.sleep();
    }

}
