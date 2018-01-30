uint32 command_id
float32 width				# Used for GRASP and RELEASE. GRASP: Expected size of the part to pick in mm. RELEASE: Opening width in mm.
float32 speed				# Used for MOVE, GRASP, RELEASE, HOMING. Speed of the fingers in mm/s.
float32 acceleration
float32 force
bool stop_on_block			# Only used for MOVE command. If true the fingers stop when there is a blockage in movement direction
# Command IDs
uint32 MOVE=101
uint32 GRASP=102
uint32 RELEASE=103
uint32 HOMING=104


