# Vehicle Longitudinal Velocity and Acceleration Message

std_msgs/Header header

float32 velocity     # meters/sec
float32 acceleration # meters/sec^2
                     # NAN means server can determine based on previous curvature commands
                     # Zero means stable velocity