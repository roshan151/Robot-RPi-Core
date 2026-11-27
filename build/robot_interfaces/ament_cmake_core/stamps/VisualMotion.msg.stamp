# Camera-based "are we moving" — the third opinion MotionHealth uses to tell a
# stall from a wheel spin. Only published while the head is settled, because a
# slewing head moves the image without the robot moving.

float64 t_monotonic         # CLOCK_MONOTONIC seconds at capture (same host)
bool    moving
float32 shift_px            # global image translation between frames (lores px)
float32 changed_fraction    # share of image cells that changed (0..1)
