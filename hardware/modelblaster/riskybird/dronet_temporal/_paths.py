"""Explicit external inputs, and the ModelBlaster checkout these scripts run against.

These scripts live in riskybird-dev's ModelBlaster overlay (hardware/modelblaster),
not inside a ModelBlaster checkout. ROOT is the pinned backend
(backends/modelblaster, or $MB); the overlay's own models are added to
modelblaster.models, and generated or trained outputs go to OUT.
"""
from pathlib import Path
import importlib.util
import os
import sys

OVERLAY = Path(__file__).resolve().parents[2]          # hardware/modelblaster
RB_ROOT = OVERLAY.parents[1]
ROOT = Path(os.environ.get("MB", RB_ROOT / "backends" / "modelblaster")).resolve()
OUT = RB_ROOT / "artifacts" / "modelblaster"
# ModelBlaster uses a namespace package; a checkout need not be named
# modelblaster. Pin this process to ROOT, and let modelblaster.models also
# find the overlay's models (dronet_stack3, gtsrb).
spec = importlib.util.spec_from_loader("modelblaster", loader=None, is_package=True)
package = importlib.util.module_from_spec(spec)
package.__path__ = [str(ROOT)]
sys.modules["modelblaster"] = package
import modelblaster.models as _models  # noqa: E402
_models.__path__.append(str(OVERLAY / "models"))


def required_path(variable, *, directory=False, default=None):
    value = os.environ.get(variable, default)
    if not value:
        raise SystemExit(f"set {variable} to the required {'directory' if directory else 'file'}")
    path = Path(value).expanduser().resolve()
    exists = path.is_dir() if directory else path.is_file()
    if not exists:
        raise SystemExit(f"{variable}: missing {'directory' if directory else 'file'}: {path}")
    return str(path)
