"""Are we moving? Asked of the camera, for almost nothing.

Two cheap measurements on consecutive lores frames (320x240 grey):

1. **Global shift** — `cv2.phaseCorrelate` finds the whole-image translation.
   Turning, or the head panning, shows up here as several pixels.
2. **Changed-cell fraction** — the image is split into a grid, and a cell
   counts as changed when its mean absolute difference (after removing the
   global brightness change, so auto-exposure and flicker don't count) exceeds
   a threshold. Driving forward changes almost every cell; a plant swaying in a
   draught, or a person walking past, changes a few.

`moving` = large global shift, or most of the image changed. Costs ~1-2 ms per
frame on a Pi 4. Only meaningful while the head is still: feed it frames with
`settled=True` or it will report the head's motion as the robot's.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class VisualMotion:
    moving: bool
    shift_px: float
    changed_fraction: float
    response: float          # phase-correlation peak strength (0..1)


class VisualMotionEstimator:
    def __init__(self, grid=(6, 8), cell_thresh: float = 6.0,
                 moving_fraction: float = 0.45, shift_px: float = 1.5,
                 min_response: float = 0.08) -> None:
        self.grid = grid
        self.cell_thresh = cell_thresh
        self.moving_fraction = moving_fraction
        self.shift_thresh = shift_px
        self.min_response = min_response
        self._prev: Optional[np.ndarray] = None
        self._win = None

    def reset(self) -> None:
        self._prev = None

    def update(self, gray: np.ndarray) -> Optional[VisualMotion]:
        import cv2
        cur = cv2.GaussianBlur(gray, (5, 5), 0).astype(np.float32)
        prev, self._prev = self._prev, cur
        if prev is None or prev.shape != cur.shape:
            return None
        if self._win is None or self._win.shape != cur.shape:
            self._win = cv2.createHanningWindow(cur.shape[::-1], cv2.CV_32F)
        (dx, dy), response = cv2.phaseCorrelate(prev, cur, self._win)
        shift = float(np.hypot(dx, dy))

        diff = np.abs((cur - cur.mean()) - (prev - prev.mean()))
        gh, gw = self.grid
        h, w = diff.shape
        cells = diff[: h - h % gh, : w - w % gw].reshape(gh, h // gh, gw, w // gw).mean(axis=(1, 3))
        changed = float((cells > self.cell_thresh).mean())

        moving = (shift >= self.shift_thresh and response >= self.min_response) \
            or changed >= self.moving_fraction
        return VisualMotion(moving, shift, changed, float(response))
