"""The vision behavior state machine. VISION-DESIGN.md Part 4.8.

    IDLE ──look──► TRACK        gimbal on a target, base still
      │ ──follow─► FOLLOW       gimbal on a person, base steps to keep standoff
      │ ──approach► APPROACH    gimbal on an object, base steps up to it
      ▼
    REACQUIRE  target lost: sweep the head around the last bearing for
               REACQUIRE_S, re-bind to the same identity/class, else IDLE

Pure: `tick()` takes what perception currently believes and returns what the
gimbal and the base should do next. The ROS node supplies inputs and carries out
the decision; tests drive it with made-up tracks.

Three rules the tick enforces, all from the design doc:
  * At most one move at a time, and a new move only after perception has
    re-anchored the target SINCE the last move ended ("look, then step").
  * Forward moves need a clearance check younger than the last move.
  * Perception DEGRADED -> stop moving, hold the head, tell the agent.
    Perception going dark must never be able to keep the robot moving.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from . import approach as approach_mod
from . import follow as follow_mod
from .freespace import FreeSpace
from .motion_types import Move


class Mode(str, Enum):
    IDLE = "idle"
    TRACK = "track"
    FOLLOW = "follow"
    APPROACH = "approach"
    REACQUIRE = "reacquire"


@dataclass(frozen=True)
class TargetView:
    """One track, as the behavior needs it. Angles are robot-body frame."""
    track_id: int
    cls: str
    identity: Optional[str]
    bearing_deg: float
    aim_elevation_deg: float          # where to point the head (upper body for people)
    distance_m: Optional[float]
    lost: bool
    anchor_t: float                   # when a detection last confirmed this track
    bottom_v_frac: float              # bbox bottom, 0 = top of image, 1 = bottom
    area_frac: float
    confidence: float = 0.0


@dataclass
class Decision:
    gimbal: tuple = ("hold",)             # ("track", pan, tilt) | ("sweep", center) | ("hold",) | ("home",)
    move: Optional[Move] = None
    want_clearance: Optional[str] = None  # None | "corridor" | "full"
    corridor_top_v: Optional[float] = None
    events: list[str] = field(default_factory=list)


@dataclass
class BehaviorConfig:
    reacquire_s: float = 5.0
    find_timeout_s: float = 6.0           # how long follow("roshan") waits for identity
    reacquire_max_bearing_jump_deg: float = 60.0
    # While the base moves, the head may not tilt above this. The clearance
    # check needs floor in view, and a head tilted up at someone's face sees
    # none — so the robot would never be allowed to step. Identity is resolved
    # before that matters (it is sticky per track, and happens at a distance).
    moving_max_tilt_deg: float = 3.0
    follow: follow_mod.FollowConfig = field(default_factory=follow_mod.FollowConfig)
    approach: approach_mod.ApproachConfig = field(default_factory=approach_mod.ApproachConfig)


@dataclass
class _Goal:
    kind: Mode                        # TRACK / FOLLOW / APPROACH
    cls: str
    label: Optional[str]              # identity wanted (follow), or None
    started_t: float
    track_id: Optional[int] = None
    last_bearing: float = 0.0
    last_identity: Optional[str] = None
    lost_t: Optional[float] = None
    travelled_m: float = 0.0
    detours: int = 0
    last_reason: str = ""


class VisionBehavior:
    def __init__(self, config: Optional[BehaviorConfig] = None) -> None:
        self.cfg = config or BehaviorConfig()
        self.mode = Mode.IDLE
        self._goal: Optional[_Goal] = None
        self._move_end_t = -1e9
        self._clear: Optional[FreeSpace] = None
        self._clear_t = -1e9
        self._events: list[str] = []

    # --------------------------------------------------------------- commands

    def look(self, cls: str, now: float) -> None:
        self._begin(_Goal(Mode.TRACK, cls, None, now))

    def follow(self, label: Optional[str], now: float) -> None:
        self._begin(_Goal(Mode.FOLLOW, "person", label or None, now))

    def approach(self, cls: str, now: float) -> None:
        self._begin(_Goal(Mode.APPROACH, cls, None, now))

    def stop(self, reason: str = "stopped") -> None:
        if self._goal is not None:
            self._events.append(f"{self._goal.kind.value} {reason}")
        self._goal = None
        self.mode = Mode.IDLE

    def set_clearance(self, fs: FreeSpace, now: float) -> None:
        self._clear, self._clear_t = fs, now

    def move_finished(self, move: Move, status: str, now: float) -> None:
        """The drivetrain finished (or refused) the last move."""
        self._move_end_t = now
        self._clear = None                         # the world moved; re-check before driving
        g = self._goal
        if g is None:
            return
        if status != "done":
            self._events.append(f"{g.kind.value}: {move.kind} {status}")
            if status in ("rejected", "failed", "cancelled"):
                self.stop(f"stopped: drivetrain {status}")
            return
        if move.kind == "drive":
            g.travelled_m += abs(move.value)

    @property
    def status(self) -> dict:
        g = self._goal
        return {
            "mode": self.mode.value,
            "target": None if g is None else (g.label or g.last_identity or g.cls),
            "track_id": None if g is None else g.track_id,
            "travelled_m": 0.0 if g is None else round(g.travelled_m, 2),
            "last_reason": "" if g is None else g.last_reason,
        }

    # ------------------------------------------------------------------ tick

    def tick(self, targets: list[TargetView], *, motion_busy: bool,
             degraded: bool, now: float) -> Decision:
        d = Decision()
        g = self._goal
        if g is None:
            self.mode = Mode.IDLE
            return self._flush(d)

        if degraded:
            self.stop("stopped: perception degraded (vision service unreachable)")
            d.gimbal = ("hold",)
            return self._flush(d)

        tgt = self._bind(g, targets, now)
        if tgt is None:
            return self._flush(self._reacquire(g, d, now))

        # Target in view.
        if self.mode == Mode.REACQUIRE:
            self._events.append(f"{g.kind.value}: reacquired {self._name(g, tgt)}")
        self.mode = g.kind
        g.lost_t, g.last_bearing = None, tgt.bearing_deg
        if tgt.identity and tgt.identity != "unknown":
            g.last_identity = tgt.identity
        tilt = tgt.aim_elevation_deg
        if g.kind in (Mode.FOLLOW, Mode.APPROACH) and not self._still_identifying(g, tgt):
            tilt = min(tilt, self.cfg.moving_max_tilt_deg)
        d.gimbal = ("track", tgt.bearing_deg, tilt)

        if g.kind == Mode.TRACK or motion_busy:
            return self._flush(d)
        if tgt.anchor_t <= self._move_end_t:
            g.last_reason = "waiting for a fresh look"
            return self._flush(d)

        clear_ahead = None if self._clear is None else self._clear.ahead_clear
        clear_age = now - self._clear_t

        if g.kind == Mode.FOLLOW:
            move = follow_mod.decide(follow_mod.FollowInput(
                tgt.bearing_deg, tgt.distance_m, clear_ahead, clear_age), self.cfg.follow)
            if move.reason == "too close" and g.last_reason != "too close":
                self._events.append(f"follow: {self._name(g, tgt)} is too close; holding")
            g.last_reason = move.reason
            if move.reason == "need clearance check":
                d.want_clearance, d.corridor_top_v = "corridor", tgt.bottom_v_frac
            elif move.kind != "hold":
                d.move = move
            return self._flush(d)

        # APPROACH
        best_open = None
        if self._clear is not None and self._clear.best_bearing_deg is not None:
            best_open = self._clear.best_bearing_deg
        st = approach_mod.step(approach_mod.ApproachInput(
            tgt.bearing_deg, tgt.distance_m, g.travelled_m, g.detours,
            clear_ahead, clear_age, best_open), self.cfg.approach)
        g.last_reason = st.move.reason
        if st.done:
            dist = f" (~{tgt.distance_m:.1f} m)" if tgt.distance_m else ""
            self._events.append(f"approach: arrived at the {g.cls}{dist}")
            self._goal, self.mode = None, Mode.IDLE
            d.gimbal = ("hold",)
        elif st.failed:
            self.stop(f"failed: {st.move.reason}")
        elif st.move.reason == "need clearance check":
            # Full-frame check: an approach wants to know where ELSE is open.
            d.want_clearance = "full"
        elif st.move.kind != "hold":
            if st.detour:
                g.detours += 1
            d.move = st.move
        return self._flush(d)

    # -------------------------------------------------------------- internals

    def _begin(self, g: _Goal) -> None:
        if self._goal is not None:
            self._events.append(f"{self._goal.kind.value} replaced by {g.kind.value}")
        self._goal = g
        self.mode = g.kind
        self._clear = None

    def _bind(self, g: _Goal, targets: list[TargetView], now: float) -> Optional[TargetView]:
        live = [t for t in targets if not t.lost and t.cls == g.cls]
        if g.track_id is not None:
            for t in live:
                if t.track_id == g.track_id:
                    # A track bound before its identity resolved may turn out to
                    # be somebody else. Follow the person asked for.
                    if g.label and t.identity not in (None, g.label):
                        break
                    return t
        # (Re)bind.
        cands = live
        want = g.label or g.last_identity
        if want:
            named = [t for t in cands if t.identity == want]
            if named:
                cands = named
            elif g.label:
                # Asked for someone specific. Accept an unresolved person only if
                # nobody resolved is a better fit; never bind to a known stranger.
                cands = [t for t in cands if t.identity is None]
                if not cands:
                    return None
        if g.track_id is not None:
            # Re-acquiring: stay near where they were.
            cands = [t for t in cands
                     if abs(t.bearing_deg - g.last_bearing) <= self.cfg.reacquire_max_bearing_jump_deg] or \
                    ([t for t in cands if t.identity == want] if want else [])
        if not cands:
            return None
        best = max(cands, key=lambda t: (t.identity == want if want else False,
                                         t.area_frac - 0.002 * abs(t.bearing_deg)))
        if g.track_id != best.track_id:
            if g.track_id is None:
                self._events.append(f"{g.kind.value}: locked on {self._name(g, best)}")
            g.track_id = best.track_id
        return best

    def _reacquire(self, g: _Goal, d: Decision, now: float) -> Decision:
        first_look = g.track_id is None          # never bound yet: "find", not "reacquire"
        if g.lost_t is None:
            g.lost_t = now
            if not first_look:
                self.mode = Mode.REACQUIRE
                self._events.append(f"{g.kind.value}: lost {g.label or g.last_identity or 'the ' + g.cls}; searching")
        limit = self.cfg.find_timeout_s if first_look else self.cfg.reacquire_s
        if now - g.lost_t > limit:
            who = g.label or g.last_identity or f"a {g.cls}"
            self.stop(f"gave up: can't see {who}")
            d.gimbal = ("home",)
            return d
        d.gimbal = ("sweep", g.last_bearing)
        g.last_reason = "searching"
        return d

    @staticmethod
    def _still_identifying(g: _Goal, t: TargetView) -> bool:
        """Following a named person whose track has no identity yet: keep the
        face in frame until it resolves."""
        return bool(g.label) and t.identity is None

    @staticmethod
    def _name(g: _Goal, t: TargetView) -> str:
        if t.identity and t.identity != "unknown":
            return t.identity
        return f"a {t.cls}" if g.cls != "person" else "someone"

    def _flush(self, d: Decision) -> Decision:
        d.events, self._events = self._events, []
        return d
