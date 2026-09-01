std_msgs/Header header

uint8 IDLE=0
uint8 RECORDING=1
uint8 REVERSING=2
uint8 SUCCEEDED=3
uint8 FAILED=4

uint8 state
string reason
float32 remaining_distance_m
