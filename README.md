# ROS package for Schunk WSG-50 Gripper
Forked from: [https://code.google.com/p/wsg50-ros-pkg](https://code.google.com/p/wsg50-ros-pkg)

Modifications of this repository:
Reading back state with high rates, open-loop control via topics, catkinized, modifications for hydro.
Existing features are not discussed here - see original Wiki: [https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50](https://code.google.com/p/wsg50-ros-pkg/wiki/wsg_50)

Todo: Restructure code


## Node wsg\_50\_ip (was: wsg\_50_tcp)

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
* *~/goal\_position [IN, wsg_50_common/Cmd]*, in modes script, auto_update:<br/>
Position goal; send target position in mm and speed
* *~/goal\_speed [IN, std_msgs/Float32]*, in mode script:<br/>
Velocity goal (in mm/s); positive values open the gripper
* *~/moving [OUT, std_msgs/Bool]*, in modes script, auto_update:<br/>
Signals a change in the motion state for position control. Can be used to wait for the end of a gripper movement. Does not work correctly yet for velocity control, since the gripper state register does not directly provide this information.
* *~/state [OUT, std_msgs/State]:*<br/>
State information (opening width, speed, forces). Note: Not all fields are available with all communication modes.
* */joint_states [OUT, sensor_msgs/JointState]:*<br/>
Standard joint state message


### Communication modes (closed-loop control)
Select by *com_mode* parameter.

* **Polling**<br />
Gripper state is polled regularly using built-in commands (original implementaion). Service calls (e.g. move) block polling as long as the gripper moves. The topic interface is not available. Up to 15 Hz could be reached with the WSG-50 hardware revision 2.

* **Script**<br />
Allows for closed-loop control with a custom script (see below) that supports up to 2 FMF finger. Gripper state is read synchronously with the specified rate. Up to 30 Hz could be reached with the WSG-50 hardware revision 2. The gripper can be controlled asynchronously by sending position or velocity goals to the topics listed above. Commands will be sent with the next read command in the timer callback timer_cb().<br />
The service interface can still be used - yet, they are blocking the gripper communication. There are no state updates while the gripper is moved by a service. 

* **Auto_update**<br>
Requests periodic updates of the gripper state (position, speed, force; less data than with the script). Up to 140 Hz could be reached with the WSG-50 hardware revision 2. All responses of the gripper must be received by a reading thread which also publishes the ROS messages. Therefore, most commands in functions.cpp cannot be used. Position targets are sent asynchronously to the gripper using the built-in commands.<br />
The services are disabled.

#### Gripper script
The script *cmd_measure.lua* must be running on the gripper for the script mode. It allows for non-blocking position and velocity control and responds with the current position, speed, motor force and up to two FMF finger forces. The custom commands 0xB0 (read only), 0xB1 (read, goal position and speed), 0xB2 (read, goal speed) are used. Tested with firmware version 2.6.4. There have been minor API changes since 1.x.


## Node wsg\_50_can

Remains unchanged; new features not implemented here. 