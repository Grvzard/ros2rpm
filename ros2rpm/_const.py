from pathlib import Path
from typing import Literal

BLUEPRINT_PATH = "blueprint-{rosdistro}.yaml"
LOGS_PATH = "logs"
CACHE_DPATH = Path.home() / ".ros2rpm"
PKGXML_CACHE_DPATH = CACHE_DPATH / "pkg_xml"

Rosdistro = Literal["humble", "iron", "jazzy"]
