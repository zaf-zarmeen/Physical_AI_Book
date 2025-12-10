# EnvironmentModel message definition for Physical AI & Humanoid Robotics Textbook

# Represents the robot's understanding of its environment
float64 timestamp
int32 object_detections
geometry_msgs/Pose[] detected_objects
string[] object_labels
int32 map_resolution  # In cells per meter
float64 map_origin_x
float64 map_origin_y
uint8[] map_data  # Occupancy grid data
std_msgs/Header header