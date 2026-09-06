"""Launch the upstream XPU-RT pilot with offscreen video compatibility.

IsaacLab must see ``headless=True`` to select its headless-rendering Kit
experience.  The upstream pilot also uses that same flag to suppress its
matplotlib panel, however, which silently disables ``--save_video``.  This
launcher lets AppLauncher consume the real value and then changes only the
already-parsed Namespace seen by the pilot's plotting code.  The backend source
stays untouched and Xvfb supplies the matplotlib display.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) < 2:
        raise SystemExit("usage: xpurt_pilot_launcher.py PILOT [ARGS ...]")
    pilot = Path(sys.argv[1]).resolve()
    arguments = sys.argv[2:]
    offscreen_video = "--headless" in arguments and "--save_video" in arguments

    from isaaclab.app import AppLauncher

    original_init = AppLauncher.__init__

    def compatible_init(self, launcher_args=None, **kwargs):
        original_init(self, launcher_args, **kwargs)
        if offscreen_video and launcher_args is not None:
            if isinstance(launcher_args, dict):
                launcher_args["headless"] = False
            else:
                launcher_args.headless = False

    AppLauncher.__init__ = compatible_init
    sys.argv = [str(pilot), *arguments]
    runpy.run_path(str(pilot), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
