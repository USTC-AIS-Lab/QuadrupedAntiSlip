float32 vel_x
float32 vel_y
float32 yaw_rate
float32 body_height
float32 body_yaw
float32 body_roll
float32 body_pitch
float32 reverve_val1
float32 reverve_val2
uint8 gait_code
uint8 cmd_code
bool is_walking
bool proportional_cmd
bool proportional_attitude

uint8 GAIT_EMPTY = 0
uint8 GAIT_MOVE_1 = 1
uint8 GAIT_MOVE_2 = 2
uint8 GAIT_MOVE_3 = 3
uint8 GAIT_MOVE_4 = 4


uint8 CMD_EMPTY = 0
uint8 CMD_STAND_UP = 1
uint8 CMD_LAY_DOWN = 2

uint8 CMD_GAIT_SWITCH_1 = 3
uint8 CMD_GAIT_SWITCH_2 = 4
uint8 CMD_GAIT_SWITCH_3 = 5
uint8 CMD_GAIT_SWITCH_4 = 6