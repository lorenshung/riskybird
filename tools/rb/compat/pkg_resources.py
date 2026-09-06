"""Small compatibility surface needed by the vendored gym-pybullet-drones.

Modern setuptools no longer ships ``pkg_resources``.  The vendored simulator
only calls ``resource_filename``, so carrying the whole deprecated package is
unnecessary.
"""

from __future__ import annotations

import importlib
from pathlib import Path


def resource_filename(package: str, resource: str) -> str:
    module = importlib.import_module(package)
    return str(Path(module.__file__).resolve().parent / resource)
