"""One call per node: send this process's robot_core.status messages to the display.

    from robot_display.link import connect
    connect(node)            # after robot_log.setup(), which resets the log handlers
"""

import json
import logging

from std_msgs.msg import String

from robot_core import status

TOPIC = "/robot/status"


def connect(node) -> None:
    pub = node.create_publisher(String, TOPIC, 20)
    status.set_sink(lambda msg: pub.publish(String(data=json.dumps(msg))))
    root = logging.getLogger()
    if not any(isinstance(h, status.LogHandler) for h in root.handlers):
        root.addHandler(status.LogHandler())             # warnings/errors -> E-codes
