"""Rule breaks: things that should never happen, counted and logged when they do.

Each is a symptom found in a real run (docs/FAILURE-MODES.md says which and how it is
handled). They go to logs.json as a WARNING "rule" event (kept with debug_mode off too),
at most once per rule every `window_s` with the repeats counted, and their running counts
ride in follow's debug note, so Monitoring-Service's scorecard.py can score a session and
compare it with another without guessing from free text.

    from robot_core.rules import rules
    rules.broke("goal_jump", jump_m=1.2)
    rules.counts          # Counter of every break since rules.reset()
"""

from __future__ import annotations

import logging
import threading
from collections import Counter

from robot_core import robot_log

RULES = {
    "goal_jump": "the tracked person moved further between two readings than anyone can walk",
    "range_refused": "a range on the person's box was nearer than where they were tracked (something in front)",
    "pivot_flip": "turning on the spot went left-right-left (2 direction changes) within 2 s: steering target flipping",
    "trip_repeat": "the reflex braked again within 0.3 m of something it had already braked for",
    "blocked_long": "nav had no safe arc for over 3 s",
    "pose_stale": "a goal was set but there was no fresh odometry pose",
    "tof_stale": "no new ToF grid for over a second while driving",
    "mac_slow": "the Mac took over 2 s to answer a follow step",
    "grid_unpaired": "the ToF grid was too far in time from the camera frame to range the box on it",
    "no_progress": "driving for 4 s without getting 5 cm nearer the goal (wedged): it backs off and plans again",
    "turn_blocked": "turning on the spot, the gyro saw no turn for 0.7 s (a corner held): it turns the other way",
    "turn_hold": "held still 1.5 s because turning would clip a mark: cleared under the body and backed off",
    "same_approach": "a way failed (bump, slip, climb, held turn) where and how one had failed before",
}


class RuleLog:
    def __init__(self, window_s: float = 5.0) -> None:
        self.window_s = window_s
        self.counts: Counter = Counter()
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self.counts.clear()

    def broke(self, rule: str, **detail) -> None:
        with self._lock:
            self.counts[rule] += 1
        robot_log.event_throttled("rule", key=rule, window_s=self.window_s, level=logging.WARNING,
                                  rule=rule, **detail)

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self.counts)


rules = RuleLog()
