# Limits for saturation of various parameters attributed to control of the drone

std_msgs/Header header

# Booleans describing which of the saturation limits should be updated
bool update_max_tilt_angle
bool update_max_yaw_rotation_speed
bool update_max_roll_pitch_rotation_speed
bool update_max_vertical_speed

# Saturations limits
float64 max_tilt_angle # interval [1, 40] deg (default 20)
float64 max_yaw_rotation_speed # interval [3, 200] deg/s (default: 70)
float64 max_roll_pitch_rotation_speed # interval [40, 300] deg/s (default: 150)
float64 max_vertical_speed # interval [1, 4] m/s (default: 1)