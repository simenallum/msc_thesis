# Output from Extended Kalman Filter

std_msgs/Header header

# Drone position relative to the landing pad 
float64 x_r
float64 y_r
float64 z_r

# Drone heading
float64 psi

# Helipad velocity relative to the landing pad
float64 u_r
float64 v_r
float64 w_r

# Covariance
float64[49] covariance
