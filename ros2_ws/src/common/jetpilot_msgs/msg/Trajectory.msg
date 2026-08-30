std_msgs/Header header

# Stable human-readable ID (for example "safe_main" or "custom_turn_2").
string line_id
# Human-readable name. Unlike line_id, this may contain spaces and Japanese text.
string display_name

# Content/provenance hash. Publishers must keep this non-empty so geometry and
# speed cannot be paired across different line revisions.
string source_hash

bool closed
TrajectoryPoint[] points
