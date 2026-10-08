"""Does the camera keep streaming when the head servos wake up? Run on the Pi with the
robot stack stopped (sudo systemctl stop robot-voice):

    python tests/hardware/check_camera.py          # camera alone
    python tests/hardware/check_camera.py head     # then open the head and move it

Prints the time each frame took. 'FAIL' = no frames (the log also shows libcamera's
"frontend has timed out"). Camera alone failing = ribbon / sensor / camera power.
Fails only after the head opens or moves = servo power or noise reaching the camera.
"""
import sys
import time

from robot_core.sensors.camera import Camera


def grab(cam, n, label):
    for i in range(n):
        t = time.time()
        try:
            cam.capture()
            print(f"{label} frame {i + 1}: {time.time() - t:.2f}s")
        except RuntimeError as e:
            print(f"{label} frame {i + 1}: FAIL ({e})")
            return False
    return True


def main() -> None:
    cam = Camera(main_size=(640, 480))
    try:
        ok = grab(cam, 10, "camera alone")
        if ok and "head" in sys.argv:
            from robot_core.sensors.gimbal import from_params, params_from_robot_yaml
            g = from_params(params_from_robot_yaml())
            try:
                grab(cam, 10, "head open")
                g.move_to(pan=-20, tilt=0, wait=False)
                grab(cam, 30, "head moving")
            finally:
                g.close()
    finally:
        cam.close()


if __name__ == "__main__":
    main()
