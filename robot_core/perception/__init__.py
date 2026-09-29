"""Layer A perception: everything the vision layer does that is not ROS.

Pure functions and small stateful classes over plain data — no rclpy, no
camera, no network unless you hand one in. That is what lets the whole cascade
(backpressure, tracking, identity, following) be tested on a laptop.
See docs/VISION-DESIGN.md.

    client.py     HTTP client for the vision service on the Mac mini
    stamps.py     FrameStamp + the seq-keyed ring buffer (Part 2.1)
    gateway.py    one-in-flight detect policy with its counters (Part 4.3)
    geometry.py   bbox -> bearing, distance-from-bbox, IOU
    tracker.py    LK optical-flow tracker + re-anchoring (Part 4.2)
    identity.py   sticky per-track identity, two-agreeing-matches rule (Part 6)
    freespace.py  clearest heading from sparse monocular depth
    follow.py     person following as short chained moves (Part 7.2)
    approach.py   drive to a detected object, re-planning each step
    gimbal.py     pan/tilt controller (absolute angles, slew-limited) + reacquire sweep
    behavior.py   the look / follow / approach / reacquire state machine
    scene.py      tracks -> body-frame TargetViews, and one-sentence descriptions
    motion_types.py  Move: the single short step behaviors ask for
    backend.py    VisionBackend: the seam the voice tools call
"""
