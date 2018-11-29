#include <ros/ros.h>
#include "wsg_50/hand.h"
#include "wsg_50_common/Status.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_hand_read");

    ros::NodeHandle nh;

    Hand hand(nh, "hand0");
    hand.start_reading();
    hand.pause_finger_reading(0);
    hand.pause_finger_reading(1);

    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(30);
    while(ros::ok()) {
        wsg_50_common::Status hand_status = hand.get_hand_state();
        ROS_INFO("Hand status: Pos %f, Speed %f, Force %f", hand_status.width,
                                                            hand_status.speed,
                                                            hand_status.force);
        loop_rate.sleep();
    }

}
