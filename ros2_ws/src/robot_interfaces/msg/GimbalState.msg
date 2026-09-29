std_msgs/Header header
float32 pan_deg           # COMMANDED angles (hobby servos have no feedback)
float32 tilt_deg
bool    settled           # no commanded motion for SETTLE_MS
string  mode
