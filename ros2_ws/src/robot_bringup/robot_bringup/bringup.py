"""Every light node, one process, one executor.

rclpy has no intra-process comms, so this does not save serialisation — it saves
a CPython interpreter and a DDS participant per node avoided, which is roughly
40 MB each on a Pi. It also gives you one thing to start, stop and watch.

`voice_node` deliberately stays outside: it owns an asyncio loop and the
microphone, and it should be restartable on its own.

The executor is **MultiThreaded** and that is not a preference. The drivetrain
node parks a thread inside a move for as long as the move takes; with the
default single-threaded executor the e-stop service would queue behind it and
arrive after the move it was meant to interrupt had already finished.
`num_threads` has to leave room for that: one for the move, one for the stop,
and headroom for timers and the encoder publisher.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_drivetrain.drivetrain_node import DrivetrainNode

NUM_THREADS = 6


def main(args=None) -> None:
    rclpy.init(args=args)

    nodes = [DrivetrainNode()]
    # Phase 3 has one node here. battery_node, speech_node, eink_node and
    # obstacle_node join this list as they land — the executor and the launch
    # file do not change.

    executor = MultiThreadedExecutor(num_threads=NUM_THREADS)
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
