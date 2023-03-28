# Setpoint for the attitude of the drone

std_msgs/Header header

# Attitude setpoint
float64 roll          # [rad], positive value will roll right
float64 pitch         # [rad], positive value will pitch up
float64 yaw_rate      # [rad/s]

# Throttle setpoint
float64 climb_rate    # [m/s] vertical speed (negative is down)