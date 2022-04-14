#!/usr/bin/python

"""
Subscribes to the gripper status message. Acknowledges error states via
a service call when they are detected
"""

# ros
import rospy
import wsg_50_common.msg
import std_srvs.srv

if __name__ == "__main__":
    rospy.init_node("wsg50_acknowledge_error_node")
    srv_name = "/wsg50_driver/wsg50/acknowledge_error"
    rate_hz = 1
    rospy.wait_for_service(srv_name)

    service_proxy = rospy.ServiceProxy(srv_name, std_srvs.srv.Empty)
    
    def on_gripper_status(msg):
        if msg.grasping_state_id == wsg_50_common.msg.Status.ERROR:
            try:
                rospy.loginfo("attempting to acknowledge error")
                service_proxy()
            except:
                rospy.loginfo("acknowledge FAILED")


    sub = rospy.Subscriber("/wsg50_driver/wsg50/status", wsg_50_common.msg.Status, on_gripper_status, queue_size=1)

    rate = rospy.Rate(rate_hz)
    rospy.spin()
    

