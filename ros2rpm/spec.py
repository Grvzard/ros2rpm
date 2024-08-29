from __future__ import annotations

import datetime
from typing import Optional

import attrs
from catkin_pkg.package import Dependency, Package

from ._const import ROS_VERSION_FAMILY, Rosdistro
from .resolve import PkgResolver, ReleaseTriple
from .utils import rosify_pkgname, sanitize_pkgname


# un-rosified name
@attrs.define
class PkgDepends:
    run_deps: set[Dependency]
    build_deps: set[Dependency]
    test_deps: set[Dependency]
    replaces: set[Dependency]
    conflicts: set[Dependency]

    @staticmethod
    def from_pkg(pkg: Package, rosdistro: Rosdistro) -> PkgDepends:
        if pkg.package_format >= 3:
            # based on https://github.com/ros/rosdistro/blob/master/index-v4.yaml
            pkg.evaluate_conditions(
                {
                    "ROS_VERSION": ROS_VERSION_FAMILY[rosdistro],
                    "ROS_DISTRO": rosdistro,
                    "ROS_PYTHON_VERSION": "3",
                }
            )

        run_deps = set(
            dep
            for dep in (pkg.run_depends + pkg.buildtool_export_depends)
            if dep.evaluated_condition is not False
        )
        build_deps = set(
            dep
            for dep in (pkg.build_depends + pkg.buildtool_depends)
            if dep.evaluated_condition is not False
        )
        test_dps = set(dep for dep in pkg.test_depends if dep.evaluated_condition is not False)
        replaces = set(dep for dep in pkg.replaces if dep.evaluated_condition is not False)
        conflicts = set(dep for dep in pkg.conflicts if dep.evaluated_condition is not False)

        return PkgDepends(
            run_deps=run_deps,
            build_deps=build_deps,
            test_deps=test_dps,
            replaces=replaces,
            conflicts=conflicts,
        )


@attrs.define
class Changelog:
    version: str = ""
    date: str = ""
    mt_name: str = ""
    mt_email: str = ""


@attrs.define
class SpecPayload:
    installation_prefix: str = ""
    ros_pkgname: str = ""
    version: str = ""
    rpm_inc: str = ""
    pkgname: str = ""
    license: str = ""
    homepage: Optional[str] = None
    no_arch: Optional[bool] = None
    depends: list[str] = attrs.Factory(list)
    build_depends: list[str] = attrs.Factory(list)
    test_depends: list[str] = attrs.Factory(list)
    conflicts: list[str] = attrs.Factory(list)
    replaces: list[str] = attrs.Factory(list)
    provides: list[str] = attrs.Factory(list)
    supplements: list[str] = attrs.Factory(list)
    description: str = ""
    # license_files: list[str] = attrs.Factory(list)
    changelogs: list[Changelog] = attrs.Factory(list)

    @staticmethod
    def from_pkg(pkg: Package, rosdistro: Rosdistro, os: str, rpm_inc: int) -> SpecPayload:
        triple = ReleaseTriple(rosdistro, *os.split(":"))
        dep_resolver = PkgResolver(triple)

        if pkg.package_format >= 3:
            pkg.evaluate_conditions(
                {
                    "ROS_VERSION": ROS_VERSION_FAMILY[rosdistro],
                    "ROS_DISTRO": rosdistro,
                    # TODO: support python2
                    "ROS_PYTHON_VERSION": "3",
                }
            )

        pkgdeps = PkgDepends.from_pkg(pkg, rosdistro)

        run_depends = dep_resolver.formatted_depnames(pkgdeps.run_deps)
        build_depends = dep_resolver.formatted_depnames(pkgdeps.build_deps)
        test_depends = dep_resolver.formatted_depnames(pkgdeps.test_deps)
        deps_replaces = dep_resolver.formatted_depnames(pkgdeps.replaces)
        deps_conflicts = dep_resolver.formatted_depnames(pkgdeps.conflicts)
        if pkg.get_build_type() == "ament_python":
            build_depends.append("python%{python3_pkgversion}-devel")
        if ROS_VERSION_FAMILY[rosdistro] == "2" and pkg.name not in [
            "ament_cmake_core",
            "ament_package",
            "ros_workspace",
        ]:
            run_depends.append(rosify_pkgname("ros-workspace", rosdistro))
            build_depends.append(rosify_pkgname("ros-workspace", rosdistro))

        url_homepage = ""
        exported_tags = [e.tagname for e in pkg.exports]
        for url in pkg.urls:
            if url.type == "website":
                url_homepage = str(url)
                break
        full_ver = f"{pkg.version}-{rpm_inc}"
        mt = pkg.maintainers[0]

        return SpecPayload(
            installation_prefix=f"/opt/ros/{rosdistro}",
            ros_pkgname=rosify_pkgname(sanitize_pkgname(pkg.name), rosdistro),
            version=pkg.version,
            rpm_inc=rpm_inc,
            pkgname=pkg.name,
            license=" and ".join(pkg.licenses),
            homepage=url_homepage,
            no_arch="metapackage" in exported_tags or "architecture_independent" in exported_tags,
            depends=sorted(run_depends),
            build_depends=sorted(build_depends),
            test_depends=sorted(test_depends),
            conflicts=sorted(deps_conflicts),
            replaces=sorted(deps_replaces),
            provides=[
                f"%{{name}}-{subpackage} = %{{version}}-%{{release}}"
                for subpackage in ("devel", "doc", "runtime")
            ]
            + [
                rosify_pkgname(sanitize_pkgname(g.name), rosdistro) + "(member)"
                for g in pkg.member_of_groups
            ],
            supplements=[
                rosify_pkgname(sanitize_pkgname(g.name), rosdistro) + "(all)"
                for g in pkg.member_of_groups
            ],
            description=pkg.description,
            # license_files=pkg.licenses,
            changelogs=[
                Changelog(
                    full_ver, datetime.datetime.now().strftime("%a %b %d %Y"), mt.name, mt.email
                )
            ],
        )
