#!/usr/bin/env python3
"""
Read logs.json back as something a human can scan.

The robot cannot tell you what went wrong, so this is the debrief.

  python read_log.py                      # last 40 events
  python read_log.py -n 200               # more history
  python read_log.py --evt estop          # just the brakes
  python read_log.py --since-start        # only the current session
  python read_log.py --problems           # warnings, errors, fatals only
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

LEVEL_RANK = {"debug": 0, "info": 1, "warn": 2, "error": 3, "fatal": 4}
MARK = {"debug": "  ", "info": "  ", "warn": " !", "error": " x", "fatal": "XX"}
SKIP_KEYS = {"ts", "lvl", "evt"}


def load(path: Path) -> list:
    if not path.exists():
        sys.exit(f"no log at {path}")
    out = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            continue          # torn final line — expected after a hard kill
    return out


def render(rec: dict) -> str:
    ts = rec.get("ts", "")[11:23]          # HH:MM:SS.mmm
    rest = " ".join(
        f"{k}={v}" for k, v in rec.items()
        if k not in SKIP_KEYS and k != "tb"
    )
    line = f"{ts} {MARK.get(rec.get('lvl'), '  ')} {rec.get('evt', '?'):<16} {rest}"
    if rec.get("tb"):
        line += "\n" + "\n".join(f"{'':<22}  at {f}" for f in rec["tb"])
    return line


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", default="logs.json")
    ap.add_argument("-n", type=int, default=40)
    ap.add_argument("--evt", help="substring match on the event name")
    ap.add_argument("--since-start", action="store_true",
                    help="only events after the last session.start")
    ap.add_argument("--problems", action="store_true",
                    help="warn and above only")
    args = ap.parse_args()

    recs = load(Path(args.path))

    if args.since_start:
        for i in range(len(recs) - 1, -1, -1):
            if recs[i].get("evt") == "session.start":
                recs = recs[i:]
                break
    if args.evt:
        recs = [r for r in recs if args.evt in r.get("evt", "")]
    if args.problems:
        recs = [r for r in recs if LEVEL_RANK.get(r.get("lvl"), 1) >= 2]

    for rec in recs[-args.n:]:
        print(render(rec))

    tail = recs[-args.n:]
    bad = sum(1 for r in tail if LEVEL_RANK.get(r.get("lvl"), 1) >= 2)
    fatal = [r for r in tail if r.get("evt") == "fatal"]
    print(f"\n{len(tail)} events, {bad} at warn+")
    if fatal:
        print(f"CAUSE OF DEATH: {fatal[-1].get('err', '?')} "
              f"({fatal[-1].get('cause', '?')}, thread={fatal[-1].get('thread', '?')})")


if __name__ == "__main__":
    main()
