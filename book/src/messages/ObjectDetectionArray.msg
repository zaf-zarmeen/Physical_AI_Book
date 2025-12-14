# ObjectDetectionArray message definition for Physical AI & Humanoid Robotics Textbook

# Array of object detections
geometry_msgs/PoseWithCovariance[] detections

# Metadata
string[] object_names
float64[] confidences
int32 count
std_msgs/Header header