"""The Pi's local navigation layer (the Mac plans globally: Vision-And-Planning nav_api).

    reflex.py      50 Hz: every new ToF grid in; brake on anything within 25 cm in the way
    costmap.py     rolling 4 x 4 m occupancy grid round the robot, from every ToF grid
    dwa.py         Dynamic Window Approach: the (v, w) to drive, from the costmap
    aim.py         where the gimbal looks: into the turn, sweeping, or at the person
    controller.py  LocalNav: ties them together at 20 Hz, streams /cmd_vel, asks the Mac for
                   waypoints and carries on without it
    stream.py      the camera, captured continuously and streamed to the Mac for visual SLAM

No ROS: the caller passes motion (drive/turn/stop/velocity), pose, twist and bump callables.
"""
