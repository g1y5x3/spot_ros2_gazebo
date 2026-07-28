std_msgs/Header header

# Centroidal [linear XYZ, angular XYZ] momentum divided by total mass.
float64[6] normalized_momentum

# Base position: X, Y, Z.
float64[3] base_position

# Base orientation: yaw, pitch, roll.
float64[3] base_ypr

# Joint order: FL, FR, RL, RR; hip-X, hip-Y, knee.
float64[12] joint_position
