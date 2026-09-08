std_msgs/Header header
# Missing/stale inputs or an unsafe selected route block readiness without emergency.
bool ready
# Current predicted motion intersects a physically forbidden area before stopping.
bool emergency
string reason
