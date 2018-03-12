uint32 command_id
float64 width				# Used for GRASP and RELEASE. GRASP: Expected size of the part to pick in m. RELEASE: Opening width in m.
float64 speed				# Used for MOVE, GRASP, RELEASE, HOMING. Speed of the fingers in m/s.
float64 force
bool stop_on_block			# Only used for MOVE command. If true the fingers stop when there is a blockage in movement direction
# Command IDs
uint32 SOFT_STOP=100
uint32 MOVE=101
uint32 GRASP=102
uint32 RELEASE=103
uint32 HOMING=104
uint32 ACKNOWLEDGE_ERROR=105


