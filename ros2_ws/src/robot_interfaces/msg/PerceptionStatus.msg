std_msgs/Header header
string  state             # OK | DEGRADED | NO_CAMERA
string  detail
string  model_version
float32 track_hz          # measured
uint32  detect_sent
uint32  detect_ok
uint32  detect_dropped_busy
uint32  detect_server_busy
uint32  detect_unavailable
uint32  detect_errors
uint32  detect_out_of_order
uint32  detect_stale
float32 last_latency_ms
float32 last_inference_ms
