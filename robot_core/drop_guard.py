"""The TF-Luna, fixed on the front and tilted down, as a drop guard: it reads where
its beam meets the floor about a third of a metre ahead, 10 times a second (via the
Arduino's E frames), whatever the head is doing. Driving forward:

  * the range jumps LONGER than the floor by jump_cm  -> the floor fell away: a drop
  * the range jumps SHORTER by low_cm                 -> something low is in the beam
  * no reading `misses` times in a row                -> the floor is not seen (a drop
                                                         so deep it is out of range, or
                                                         a black floor): stop to be safe
Two readings in a row are needed for the first two, so one noisy reading never brakes.

The floor's range comes from robot.yaml (tfluna_floor_cm), or is learnt while the robot
stands still (0): the median of the first `learn_n` readings. Until there is a floor
range the guard is not armed, so a TF-Luna that is not fitted (every reading -1) or old
firmware (no reading at all) never stops anything. Plain logic: no ROS, no serial.
"""

from __future__ import annotations

import statistics
from typing import List, Optional


class DropGuard:
    def __init__(self, floor_cm: float = 0.0, jump_cm: float = 6.0, low_cm: float = 6.0,
                 misses: int = 3, learn_n: int = 20) -> None:
        self.floor_cm = float(floor_cm) or None
        self.jump_cm, self.low_cm, self.misses, self.learn_n = jump_cm, low_cm, misses, learn_n
        self._learn: List[int] = []
        self._far = self._near = self._none = 0
        self._seen = False                       # any valid reading at all: the sensor is there

    @property
    def armed(self) -> bool:
        return self.floor_cm is not None and self._seen

    def update(self, cm: Optional[int], forward: bool) -> Optional[str]:
        """One reading (None: no valid range). Returns why to brake, or None."""
        if cm is not None:
            self._seen = True
        if not forward:
            self._far = self._near = self._none = 0
            if cm is not None and self.floor_cm is None:
                self._learn.append(cm)
                if len(self._learn) >= self.learn_n:
                    self.floor_cm = float(statistics.median(self._learn))
            return None
        if not self.armed:
            return None
        if cm is None:
            self._none += 1
            self._far = self._near = 0
            return f"floor not seen by the TF-Luna ({self._none} readings)" if self._none >= self.misses else None
        self._none = 0
        self._far = self._far + 1 if cm > self.floor_cm + self.jump_cm else 0
        self._near = self._near + 1 if cm < self.floor_cm - self.low_cm else 0
        if self._far >= 2:
            return f"drop ahead: TF-Luna {cm} cm, floor {self.floor_cm:.0f} cm"
        if self._near >= 2:
            return f"something low ahead: TF-Luna {cm} cm, floor {self.floor_cm:.0f} cm"
        return None
