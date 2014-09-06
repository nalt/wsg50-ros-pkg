# wsg50-ros-pkg
Forked from: [https://code.google.com/p/wsg50-ros-pkg](https://code.google.com/p/wsg50-ros-pkg)

See also Wiki: [https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50](https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50)

## Node wsg\_50_tcp

### Parameters
* *ip*: IP address of gripper
* *port*: Port of gripper
* *local_port*: Local port for UDP
* *protocol*: udp or tcp (default)
* *com_mode*: polling (default), script or auto_update. See communication modes below.
* *rate*: Polling rate in Hz.
* *grasping_force*: Set grasping force limit on startup


### Services
See [https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50](https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50). Services currently block the reception of state updates.

### Topics
* *~/goal\_position [IN, wsg_50_common/Cmd], Modes: script, auto_update:* Position goal; send target position in mm and speed
* *~/goal\_speed [IN, std_msgs/Float32], Modes: script:* Velocity goal (in mm/s); positive values open the gripper
* *~/moving [OUT, std_msgs/Bool], Modes: script, auto_update:* Signals a change in the motion state for position control. Can be used to wait for the end of a gripper movement. Signaling does not work correctly get for velocity control, since the gripper state register does not directly provide this information.
* *~/state [OUT, std_msgs/State]:* State information (opening width, speed, forces). Note: Not all fields are available with all communication modes.
* */joint_states [OUT]:* Standard joint state message


### Communication modes (closed-loop control)
Select by *com_mode* parameters.

* **Polling**<br />
Gripper state is polled regularly using built-in commands. Services (e.g. move) block the polling as long as the gripper moves; the topic interface is not available. Up to 15 Hz could be reached with the WSG-50 hardware revision 2.

* **Script**<br />
Allows for closed-loop control with a custom script (see below) that supports up to 2 FMF finger. Gripper state is read synchronously with the specified rate. Up to 30 Hz could be reached with the WSG-50 hardware revision 2. The gripper can be controlled asynchronously by sending position or velocity goals to the topics shown below. Commands will be sent with the next read command in the timer callback timer_cb().<br />
The services can still be used - yet, they are blocking the gripper communication. There are no state updates while the gripper is moved by a service. 

* **Auto_update**<br>
Requests periodic updates of the gripper state (position, speed, force; less than with the script). Up to 140 Hz could be reached with the WSG-50 hardware revision 2. All responses of the gripper must be received by a reading thread which also publishes the ROS messages. Therefore, the commands in functions.cpp cannot be used. Position targets are sent asynchronously to the gripper using the built-in commands. As with the script option, there is a noticeable delay until the movement starts.<br />
The services are disabled.

#### Gripper script
The script *cmd_measure.lua* must be running on the gripper for the script mode. It allows for non-blocking position and velocity control and responds with the current position, speed, motor force and up to two FMF finger forces. The custom commands 0xB0 (read only), 0xB1 (read, goal position and speed), 0xB2 (read, goal speed) are used. Tested with firmware version 2.6.4. There have been minor API changes compared to 1.x.