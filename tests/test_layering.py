"""The Layer A invariant, as a test.

`robot_core/` must never import ROS. Everything else about this rebuild rests
on that: it is what keeps these tests runnable on a laptop with no ROS
installed, what stops a node quietly growing hardware logic, and what makes the
migration reversible.

It is one grep, and it belongs in the suite rather than in a CI script nobody
reads.
"""

from __future__ import annotations

from pathlib import Path

CORE = Path(__file__).resolve().parent.parent / "robot_core"

FORBIDDEN = ("rclpy", "rosidl", "ament", "robot_interfaces", "std_srvs")


def test_layer_a_imports_no_ros() -> None:
    offenders = []
    for path in sorted(CORE.rglob("*.py")):
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            stripped = line.strip()
            if not (stripped.startswith("import ") or stripped.startswith("from ")):
                continue
            if any(name in stripped for name in FORBIDDEN):
                offenders.append(f"{path.relative_to(CORE.parent)}:{lineno}: {stripped}")

    assert not offenders, (
        "robot_core must not import ROS — move this into a node under "
        "ros2_ws/src/:\n  " + "\n  ".join(offenders)
    )


def test_every_core_package_has_an_init() -> None:
    """A missing __init__.py works when you run from the repo root and breaks
    the moment the package is pip-installed, which is a confusing way to find
    out."""
    missing = [
        str(d.relative_to(CORE.parent))
        for d in sorted(CORE.rglob("*"))
        if d.is_dir() and d.name != "__pycache__" and not (d / "__init__.py").exists()
    ]
    assert not missing, f"packages without __init__.py: {missing}"
