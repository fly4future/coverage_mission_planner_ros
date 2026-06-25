# Header information
std_msgs/Header header

# Fleet Task information
string mission_id
geometry_msgs/Point[] initial_positions

# Flight perimeters mapping
geometry_msgs/Polygon[] fly_zones
geometry_msgs/Polygon[] no_fly_zones
