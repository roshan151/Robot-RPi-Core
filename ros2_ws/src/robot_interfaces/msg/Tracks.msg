# Every track, at TRACK_HZ. Tiny: no pixels ever cross a topic.
std_msgs/Header header
uint32  seq
float32 pan_deg           # head angles the bearings were computed with
float32 tilt_deg
TrackedObject[] tracks
