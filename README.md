# ROS package for Schunk WSG-50 Gripper

This package implements a TCP, UDP and CAN drivers for the WSG-50 gripper.

While this package primarily supports the WSG-50 module, it has also worked without modification for the WSG-32 module.

Originally forked from: [https://code.google.com/p/wsg50-ros-pkg](https://code.google.com/p/wsg50-ros-pkg)

## Node wsg\_50\_ip

### Usage 

Launch files are provided for convenience:

```
roslaunch wsg_50_driver wsg_50_ip.launch ip:=<your ip> ... com_mode:=<mode of choice>
```

### Parameters
* *ip*: IP address of gripper
* *port*: Port of gripper
* *local_port*: Local port for UDP
* *protocol*: udp or tcp (default)
* *com_mode*: polling (default), script or auto_update. See communication modes below.
* *rate*: Polling rate in Hz.
* *force*: Set grasping force limit on startup
* *home_at_start*: Home module at start and tare force sensor, true (default) or false

### Services

> Only available when `com_mode:=polling`.
>
> Services will block the reception of state updates.

* *~/ack [std_srvs/Empty]*:<br/>
Acknowledges an error occured.
* *~/grasp [wsg_50_common/Move]*:<br/>
Grasps an object of a specific width at a specific velocity. Normally used setting a zero width object and a low velocity.
* *~/homing [std_srvs/Empty]*:<br/>
Moves fingers to home position (maximum opening).
* *~/move [wsg_50_common/Move]*:<br/>
Moves fingers to an absolute position at a specific velocity. Deprecated by the creation of the next service.
* *~/move_incrementally [wsg_50_common/Incr]
Moves fingers to a specific distance in a specific direction (open/close) regarding the anterior position.
* *~/release [wsg_50_common/Move]*:<br/>
Releases a grasped object opening the fingers to a indicated position.
* *~/set_acceleration [wsg_50_common/Conf]*:<br/>
Set the acceleration with the gripper fingers moves.
* *~/set_force [wsg_50_common/Conf]*:<br/>
Set the force with the gripper grasp objects.
* *~/stop [std_srvs/Empty]*:<br/>
Stops a current action.

### Topics

* *~/goal\_position [IN, wsg_50_common/Cmd]*, in modes script, auto_update:<br/>
Position goal; send target position in mm and speed
* *~/goal\_speed [IN, std_msgs/Float32]*, in mode script:<br/>
Velocity goal (in mm/s); positive values open the gripper
* *~/moving [OUT, std_msgs/Bool]*, in modes script, auto_update:<br/>
Signals a change in the motion state for position control. Can be used to wait for the end of a gripper movement. Does not work correctly yet for velocity control, since the gripper state register does not directly provide this information.
* *~/status [OUT, wsg_50_common/Status]:*<br/>
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
The script *cmd_measure.lua* must be running on the gripper for the script mode. It allows for non-blocking position and velocity control and responds with the current position, speed, motor force and up to two FMF finger forces. 

You can upload the script on the web-based control panel, under 'scripting'.

The custom commands 0xB0 (read only), 0xB1 (read, goal position and speed), 0xB2 (read, goal speed) are used. Tested with firmware version 4.0.9. There have been minor API changes since 1.x.


## Node wsg\_50_can

Remains unchanged; new features not implemented here. 