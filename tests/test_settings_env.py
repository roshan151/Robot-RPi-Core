"""settings reads secrets from env files (/etc/robot.env, .env), and the real environment wins."""

import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def _key(tmp_path, file_text, env=None):
    f = tmp_path / "robot.env"
    f.write_text(file_text)
    e = {k: v for k, v in os.environ.items() if k not in ("GEMINI_API_KEY", "GOOGLE_API_KEY")}
    e.update(env or {}, ROBOT_ENV_FILE=str(f), PYTHONPATH=str(REPO))
    out = subprocess.run([sys.executable, "-c", "from robot_core import settings; print(settings.GEMINI_API_KEY)"],
                         env=e, capture_output=True, text=True, check=True, cwd=tmp_path)
    return out.stdout.strip()


def test_gemini_key_is_read_from_an_env_file(tmp_path):
    assert _key(tmp_path, "# keys\nexport GEMINI_API_KEY='abc123'\n") == "abc123"
    assert _key(tmp_path, "GOOGLE_API_KEY=xyz\n") == "xyz"


def test_the_real_environment_wins_over_the_file(tmp_path):
    assert _key(tmp_path, "GEMINI_API_KEY=from-file\n", {"GEMINI_API_KEY": "from-env"}) == "from-env"
