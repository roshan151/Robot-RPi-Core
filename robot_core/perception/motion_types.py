"""The one motion vocabulary the behaviors speak: a single short move, or none."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Move:
    kind: str            # "turn" | "drive" | "hold"
    value: float = 0.0   # degrees (+ right) for turn, metres (+ forward) for drive
    reason: str = ""

    @staticmethod
    def hold(reason: str) -> "Move":
        return Move("hold", 0.0, reason)

    def __str__(self) -> str:
        if self.kind == "hold":
            return f"hold ({self.reason})"
        unit = "deg" if self.kind == "turn" else "m"
        return f"{self.kind} {self.value:+.2f} {unit} ({self.reason})"
