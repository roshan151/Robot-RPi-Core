# One object from /v1/detect, in the frame it was detected in.
string  cls
float32 confidence
float32[4] bbox           # x1,y1,x2,y2, NORMALISED 0..1 of the image as sent
float32 bearing_deg       # robot-body frame at capture (pan included), + = right
float32 distance_m        # bbox-size estimate; NaN when the class has no known size
