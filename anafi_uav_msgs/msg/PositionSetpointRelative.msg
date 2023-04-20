# Setpoint for the drone relative to the current position

std_msgs/Header header

# Position setpoint relative to drone body frame
float64 dx                      # forward [m]
float64 dy                      # right [m]
float64 dz                      # down [m]
float64 dpsi                    # [rad] heading angle relative to current angle (positive clockwise)

# Constraints
float64 max_horizontal_speed    # [m/s]
float64 max_vertical_speed      # [m/s]
float64 max_yaw_rotation_speed  # [rad/s]

