#include <WeissHand.h>
#include <iostream>
#include "std_msgs/String.h"
#include <ros/rate.h>
#include <sstream>

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("WEISS HARDWARE starting");
    ros::init(argc, argv, "weiss_hardware");
    ros::NodeHandle nh;

    // TODO: How should params even be passed?
    WeissHand robot(nh);
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Ros control rate of 100Hz
    ros::Rate controlRate(100.0);
    while (ros::ok())
    {
        robot.read();
        cm.update(robot.get_time(), robot.get_period());

        robot.write();
        controlRate.sleep();
    }

    return 0;
}
