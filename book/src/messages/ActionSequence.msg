# ActionSequence message definition for Physical AI & Humanoid Robotics Textbook

# An action sequence is a list of actions for VLA systems
std_msgs/String[] actions

# Additional metadata
string description
float64 estimated_time
string source  # e.g., 'vla_planner', 'user_input'