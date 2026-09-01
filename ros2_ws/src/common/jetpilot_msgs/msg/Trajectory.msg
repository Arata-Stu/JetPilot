std_msgs/Header header

# Stable human-readable ID (for example "safe_main" or "custom_turn_2").
string line_id
# Human-readable name. Unlike line_id, this may contain spaces and Japanese text.
string display_name

# Content/provenance hash. Publishers must keep this non-empty so geometry and
# speed cannot be paired across different line revisions.
string source_hash

bool closed

# Direction of vehicle motion while following this trajectory. Geometry remains
# ordered from the current/start point toward the goal in both modes.
uint8 MOTION_FORWARD=0
uint8 MOTION_REVERSE=1
uint8 motion_direction

TrajectoryPoint[] points
