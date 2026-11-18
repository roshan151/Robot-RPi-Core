"""The suite checks full logging (info lines in logs.json): run it as in debug mode,
whatever robot.yaml's debug_mode says. Tests of the warnings-only file pass debug=False."""

import os

os.environ.setdefault("ROBOT_DEBUG_MODE", "1")
