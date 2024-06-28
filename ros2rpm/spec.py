import datetime
from typing import Optional

import attrs
from catkin_pkg.package import Package

from ._const import Rosdistro
from .resolve import PkgResolver, ReleaseTriple
from .utils import rosify_pkgname, sanitize_pkgname


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

    @classmethod
    def from_pkg(cls, pkg: Package, rosdistro: Rosdistro, os: str, rpm_inc: int):
        triple = ReleaseTriple(rosdistro, *os.split(":"))
        dep_resolver = PkgResolver(triple)

        if pkg.package_format >= 3:
            pkg.evaluate_conditions(
                {
                    # TODO support ros-1
                    "ROS_VERSION": "2",
                    "ROS_DISTRO": rosdistro,
                    "ROS_PYTHON_VERSION": "3",
                }
            )

        depends = dep_resolver.formatted_depnames(
            dep
            for dep in (pkg.run_depends + pkg.buildtool_export_depends)
            if dep.evaluated_condition is not False
        )
        build_depends = dep_resolver.formatted_depnames(
            dep
            for dep in (pkg.build_depends + pkg.buildtool_depends)
            if dep.evaluated_condition is not False
        )
        test_depends = dep_resolver.formatted_depnames(
            dep for dep in pkg.test_depends if dep.evaluated_condition is not False
        )
        deps_replaces = dep_resolver.formatted_depnames(
            dep for dep in pkg.replaces if dep.evaluated_condition is not False
        )
        deps_conflicts = dep_resolver.formatted_depnames(
            dep for dep in pkg.conflicts if dep.evaluated_condition is not False
        )

        pkgname = sanitize_pkgname(pkg.name)
        ros_pkgname = rosify_pkgname(pkgname, rosdistro)
        url_homepage = ""
        exported_tags = [e.tagname for e in pkg.exports]
        for url in pkg.urls:
            if url.type == "website":
                url_homepage = str(url)
                break
        full_ver = f"{pkg.version}-{rpm_inc}"
        mt = pkg.maintainers[0]

        depends = set(depends)
        build_depends = set(build_depends)
        if pkg.name not in ["ament_cmake_core", "ament_package", "ros_workspace"]:
            workspace_pkg_name = rosify_pkgname("ros-workspace", rosdistro)
            depends.add(workspace_pkg_name)
            build_depends.add(workspace_pkg_name)
        if pkg.get_build_type() == "ament_python":
            build_depends.add("python%{python3_pkgversion}-devel")

        return SpecPayload(
            installation_prefix=f"/opt/ros/{rosdistro}",
            ros_pkgname=ros_pkgname,
            version=pkg.version,
            rpm_inc=rpm_inc,
            pkgname=pkg.name,
            license=" and ".join(pkg.licenses),
            homepage=url_homepage,
            no_arch="metapackage" in exported_tags or "architecture_independent" in exported_tags,
            depends=sorted(depends),
            build_depends=sorted(build_depends),
            test_depends=sorted(set(test_depends)),
            conflicts=sorted(set(deps_conflicts)),
            replaces=sorted(set(deps_replaces)),
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
