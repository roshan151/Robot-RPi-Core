"""The Pi side of Monitoring-Service/calibrate.py: JSON lines in, JSON lines out; an
error answers and keeps the session going. No hardware: hello and the fits only."""

import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def test_protocol_hello_fits_and_errors():
    cmds = [{"cmd": "hello"},
            {"cmd": "fit", "args": {"kind": "turn", "commanded": [360], "measured": [350]}},
            {"cmd": "no_such_thing"},
            {"cmd": "fit", "args": {"kind": "trim", "ratios": [1.0], "current": 10}},
            {"cmd": "quit"}]
    out = subprocess.run([sys.executable, "-m", "robot_core.calibrate_pi"], cwd=REPO, capture_output=True, text=True,
                         input="".join(json.dumps(c) + "\n" for c in cmds), timeout=60)
    lines = [json.loads(l) for l in out.stdout.splitlines()]
    assert lines[0] == {"ready": True}
    hello, turn, bad, trim, bye = lines[1:]
    assert hello["ok"] and "ticks_per_cm" in hello["result"]["drivetrain"]
    assert hello["result"]["firmware"]["REVERSE_TRIM_PCT"] is not None
    assert turn["ok"] and turn["result"]["ticks_per_degree"] > 0
    assert not bad["ok"] and "no_such_thing" in bad["error"]
    assert trim["ok"] and trim["result"]["REVERSE_TRIM_PCT"] == 10
    assert bye["ok"]
