#include <ros/ros.h>
#include "wsg_50/hand.h"

int main(int argc, char** argv) {

    double width = 10.0;
    double speed = 50.0;

    if(argc > 1) {
        width = std::atof(argv[1]);
    }

    if(argc > 2) {
        speed = std::atof(argv[2]);
    }

    ros::init(argc, argv, "test_hand_move");

    ros::NodeHandle nh;

    Hand hand(nh, "hand0");
    hand.move_hand(width,speed);

    ros::spin();
}
