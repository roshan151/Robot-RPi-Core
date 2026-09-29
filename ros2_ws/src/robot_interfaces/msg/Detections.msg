# One /v1/detect answer. Published at DETECT_HZ, only when a result is applied.
std_msgs/Header header
uint32  seq               # the frame's capture counter (VISION-DESIGN Part 2.1)
Detection[] detections
string  model_version
float32 inference_ms      # server's view
float32 latency_ms        # Pi wall-clock round trip; the difference is the network
