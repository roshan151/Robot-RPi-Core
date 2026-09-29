# One track from the Pi-side optical-flow tracker, in the CURRENT frame.
uint32  id
string  cls
string  identity          # "" = not resolved yet, "unknown" = tried and gave up
float32 identity_score
float32 confidence
float32[4] bbox           # normalised 0..1
float32 bearing_deg       # robot-body frame, + = right
float32 aim_elevation_deg # where to point the head, + = up
float32 distance_m        # NaN when unknown
bool    lost
float64 anchor_age_s      # seconds since a detection last confirmed this track
float32 bottom_v          # bbox bottom, normalised (for the follow corridor check)
float32 area_frac
