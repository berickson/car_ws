std_msgs/Header header

float32 curvature           # desired steering curvature (radians/m)
float32 curvature_rate      # desired rate of change for curvature (radians/m/sec)
                            # NAN means server can determine based on previous curvature commands
                            # Zero means stable curvature