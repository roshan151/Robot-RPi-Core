# Is the robot actually doing what it was told? Published by drivetrain_node at
# the encoder rate. See robot_core/odometry.py:MotionHealth for the logic.

string  state               # idle | moving | stalled | slipping | pushed | wrong_direction
string  reason              # human-readable, empty when all is well
float64 v                   # m/s from the encoders
float64 w                   # rad/s from the encoders, CCW positive
float64 sigma_xy            # 1-sigma position drift budget since the last SLAM fix, m
float64 sigma_theta         # 1-sigma heading drift budget, rad
float64 drift_deg_per_m     # heading change per metre on straight moves (EWMA)
bool    visual_known        # a fresh /visual_motion sample was available
bool    visual_moving       # ...and what it said
float64 since_s             # how long `state` has held
