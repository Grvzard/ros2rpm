from typing import Literal

BLUEPRINT_PATH = "blueprint-{rosdistro}.yaml"
PKGXML_CACHEDIR_PATH = "_ros2rpm_cache"
LOGS_PATH = "logs"

Rosdistro = Literal["humble", "iron", "jazzy"]
