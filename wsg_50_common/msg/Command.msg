uint32 command_id
float32 width				# Used for GRASP and RELEASE. GRASP: Expected size of the part to pick in mm. RELEASE: Opening width in mm.
float32 speed				# Used for MOVE, GRASP, RELEASE, HOMING. Speed of the fingers in mm/s.
float32 acceleration
float32 force
bool stop_on_block			# Only used for MOVE command. If true the fingers stop when there is a blockage in movement direction
# Command IDs
uint32 SOFT_STOP=100		# Stops movement. Does not require an acknowledgment, instead the next move commando is executed
uint32 MOVE=101
uint32 GRASP=102
uint32 RELEASE=103
uint32 HOMING=104
uint32 ACK_ERROR=105
uint32 EMERGENCY_STOP=106	# Stops movement and puts the gripper into error state. Needs acknowledgment to recover.
uint32 SET_FORCE=107
uint32 SET_ACCELERATION=108 

