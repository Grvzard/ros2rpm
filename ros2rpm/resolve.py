from functools import lru_cache
from typing import Iterable, Union

import attrs
from catkin_pkg.package import Dependency
from rosdep2 import ResolutionError, create_default_installer_context
from rosdep2.catkin_support import get_catkin_view


@attrs.define
class ReleaseTriple:
    ros_distro: str
    os_name: str
    os_version: str


class PkgResolver:
    VERSIONS = {
        "version_lt": "<",
        "version_lte": "<=",
        "version_eq": "=",
        "version_gte": ">=",
        "version_gt": ">",
    }

    def __init__(self, triple: ReleaseTriple) -> None:
        self._triple = triple
        self._ctx = create_default_installer_context()
        installer_key = self._ctx.get_default_os_installer_key(triple.os_name)
        self._ctx_installer = self._ctx.get_installer(installer_key)
        self._os_installers = self._ctx.get_os_installer_keys(self._triple.os_name)
        self._view = get_catkin_view(triple.ros_distro, triple.os_name, triple.os_version, False)

    @lru_cache(maxsize=None)
    def resolve_rosdep(self, key: str) -> str:
        # refer to `bloom.generators.common`
        try:
            dep = self._view.lookup(key)
            inst_key, rule = dep.get_rule_for_platform(
                self._triple.os_name,
                self._triple.os_version,
                self._os_installers,
                self._ctx.get_default_os_installer_key(self._triple.os_name),
            )
            assert inst_key in self._os_installers
            return self._ctx_installer.resolve(rule)
        except KeyError:
            raise KeyError(f"Could not resolve rosdep key '{key}'")
        except ResolutionError as e:
            raise KeyError(
                f"Could not resolve rosdep key '{key}' for version {self._triple.os_version}:\n"
                + repr(e)
            )

    def formatted_depnames(self, deps: Iterable[Union[Dependency, str]]):
        res = []
        for dep in deps:
            if isinstance(dep, Dependency):
                for resolved_name in self.resolve_rosdep(dep.name):
                    version_deps = [
                        k for k in self.VERSIONS.keys() if getattr(dep, k) is not None
                    ]
                    if not version_deps:
                        res.append(resolved_name)
                    else:
                        res.extend(
                            [
                                f"{resolved_name} {self.VERSIONS[v]} {getattr(dep, v)}"
                                for v in version_deps
                            ]
                        )
            else:
                res.extend(resolved_name for resolved_name in self.resolve_rosdep(dep.name))

        return res
