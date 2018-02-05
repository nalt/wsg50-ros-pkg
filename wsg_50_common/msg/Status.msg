int32 grasping_state_id
string grasping_state
int32 return_code
int32 connection_state
float32 width
float32 current_speed
float32 current_force
float32 grasping_force
float32 acceleration

# grasping state IDs
int32 IDLE = 0
int32 GRASPING = 1
int32 NO_PART_FOUND=2
int32 PART_LOST=3
int32 HOLDING=4
int32 RELEASING=5
int32 POSITIONING=6
int32 ERROR=7
int32 UNKNOWN = -1

# important return codes
int32 E_UNKNOWN = -1
int32 E_SUCCESS = 0
 
# node state
int32 NOT_CONNECTED = 0
int32 CONNECTED = 1

