std_msgs/Header header

uint8 FOLLOW_ROUTE=0
uint8 WAIT_SIGNAL=1
uint8 ROUTE_COMMITTED=2
uint8 RECOVERY=3
uint8 FAULT=4

uint8 state
string active_lane_id
string active_junction_id
string reason
