import logging
import subprocess
from pathlib import Path
from typing import Optional, Union

import httpx
import networkx as nx
import yaml
from attrs import asdict
from catkin_pkg.package import Package
from catkin_pkg.packages import parse_package_string
from jinja2 import Environment, PackageLoader

from ._const import BLUEPRINT_PATH, LOGS_PATH, PKGXML_CACHEDIR_PATH, Rosdistro
from .spec import SpecPayload

logger = logging.getLogger(__name__)


def gen_spec(
    pkg_dir: Union[Path, str], dst_path: Path, rosdistro: Rosdistro, os: str, rpm_inc: int
):
    if not isinstance(pkg_dir, Path):
        pkg_dir = Path(pkg_dir)

    if pkg_dir.is_dir():
        xml_fpath = pkg_dir / "package.xml"
    else:
        xml_fpath = pkg_dir
    assert xml_fpath.exists()

    pkg: Package = parse_package_string(xml_fpath.read_text())

    spec = SpecPayload.from_pkg(pkg, rosdistro, os, rpm_inc)

    jenv = Environment(loader=PackageLoader("ros2rpm"), lstrip_blocks=True, trim_blocks=True)
    pkg_typ = pkg.get_build_type()
    templ = jenv.get_template(f"{pkg_typ}.spec")

    dst_path.touch(mode=0o666, exist_ok=True)
    dst_path.write_text(templ.render(**asdict(spec)))


def resolve_dist(dist_file: Union[Path, str], rosdistro: str, cache_dir: Optional[str]):
    """distribution.yaml -> blueprint-{rosdistro}.yaml"""
    if not isinstance(dist_file, Path):
        dist_fpath = Path(dist_file)
    else:
        dist_fpath = dist_file
    cache_dpath = Path(cache_dir or PKGXML_CACHEDIR_PATH)
    if not cache_dpath.exists():
        cache_dpath.mkdir(parents=True)
    assert cache_dpath.is_dir()

    http_client = httpx.Client(timeout=15.0, http2=True, http1=False)
    pkg_infos = {}
    skipped_list = []
    pkg_run_deps: dict[str, list] = {}
    pkg_build_deps: dict[str, list] = {}

    dist_dict = yaml.safe_load(dist_fpath.open())
    for repo, info in dist_dict["repositories"].items():
        if (
            "release" not in info
            or "version" not in info["release"]
            or not info["release"]["url"].startswith("https://github.com")
        ):
            skipped_list.append(repo)
            continue
        if "packages" in info["release"]:
            pkgs = info["release"]["packages"]
        else:
            pkgs = [repo]

        assert (
            info["release"]["tags"]["release"] == f"release/{rosdistro}/{{package}}/{{version}}"
        )

        src_host = info["release"]["url"]
        raw_host = src_host.replace("github.com", "raw.githubusercontent.com").rstrip(".git")
        version = info["release"]["version"]
        for pkg_name in pkgs:
            assert pkg_name not in pkg_infos.keys()
            src_path = f"release/{rosdistro}/{pkg_name}/{version}"
            xml_path = f"{src_path}/package.xml"

            cache_file = cache_dpath / xml_path
            if cache_file.exists():
                xml_content = cache_file.read_text()
                logger.info(f"cached: {raw_host}/{xml_path}")
            else:
                xml_url = f"{raw_host}/{xml_path}"
                resp = http_client.get(xml_url)
                if resp.status_code != 200:
                    raise RuntimeError(f"failed while fetching: {xml_url}")
                xml_content = resp.text
                cache_file.parent.mkdir(parents=True)
                cache_file.write_text(xml_content)
                logger.info(f"fetched: {raw_host}/{xml_path}")

            pkg: Package = parse_package_string(xml_content)
            if pkg.package_format >= 3:
                pkg.evaluate_conditions(
                    {
                        # TODO support ros-1
                        "ROS_VERSION": "2",
                        "ROS_DISTRO": rosdistro,
                        "ROS_PYTHON_VERSION": "3",
                    }
                )

            pkg_build_deps[pkg_name] = [
                dep.name
                for dep in (pkg.build_depends + pkg.buildtool_depends + pkg.test_depends)
                if dep.evaluated_condition is not False
            ]
            pkg_run_deps[pkg_name] = [
                dep.name
                for dep in (pkg.run_depends + pkg.buildtool_export_depends)
                if dep.evaluated_condition is not False
            ]
            if pkg_name not in ("ament_package", "ament_cmake_core", "ros_workspace"):
                pkg_build_deps[pkg_name].append("ros_workspace")
                pkg_run_deps[pkg_name].append("ros_workspace")
            pkg_infos[pkg_name] = {
                "git": src_host,
                "branch": src_path,
                "rundeps": None,  # fill later
            }
            logger.info(f"resolved: {pkg_name}")

    dep_graph_data = []
    pkg_build_deps = {
        pkg_name: [dep for dep in deps if dep in pkg_infos.keys()]
        for pkg_name, deps in pkg_build_deps.items()
    }
    pkg_run_deps = {
        pkg_name: [dep for dep in deps if dep in pkg_infos.keys()]
        for pkg_name, deps in pkg_build_deps.items()
    }
    for pkg_name, build_deps in pkg_build_deps.items():
        for builddep_name in build_deps:
            dep_graph_data.append((builddep_name, pkg_name))
            dep_graph_data.extend(
                (rundep_name, pkg_name) for rundep_name in pkg_run_deps[builddep_name]
            )
    for pkg_name, run_deps in pkg_run_deps.items():
        pkg_infos[pkg_name]["rundeps"] = run_deps
    g = {"pkgs": pkg_infos, "graph": list(set(dep_graph_data))}
    (cache_dpath / f"{rosdistro}-skipped.log").write_text(yaml.safe_dump(skipped_list))
    Path(BLUEPRINT_PATH.format(rosdistro=rosdistro)).write_text(yaml.safe_dump(g))


def _earthly_build(target: str, args: list[str]) -> tuple[bool, str]:
    assert target in ("rpm-build", "gen", "build-stage0-repo")
    p = subprocess.run(["earthly", f"+{target}", *args], capture_output=True)
    return p.returncode == 0, p.stderr.decode()


def gen(rosdistro: Rosdistro, retry: bool):
    """generate RPM .spec file and source archive for all packages"""
    logs_dpath = Path(f"{LOGS_PATH}/gen")
    logs_dpath.mkdir(parents=True, exist_ok=True)

    progress_fpath = logs_dpath / "_progress.yaml"
    progress_fpath.touch(exist_ok=True)
    progress = yaml.safe_load(progress_fpath.read_text()) or {"done": set(), "failed": set()}
    if retry:
        progress["failed"] = set()

    bp = yaml.safe_load(Path(BLUEPRINT_PATH.format(rosdistro=rosdistro)).read_text())
    pkg_infos = bp["pkgs"]

    for pkg_name, info in pkg_infos.items():
        logger.info(pkg_name)
        if pkg_name in progress["done"] or pkg_name in progress["failed"]:
            continue
        url = info["git"]
        _, _, _, ver = info["branch"].split("/")

        ok, log = _earthly_build(
            "gen",
            [
                f"--url={url}",
                f"--package={pkg_name}",
                f"--version={ver}",
                f"--rosdistro={rosdistro}",
                "--os=openeuler:24.03",
            ],
        )
        if ok:
            progress["done"].add(pkg_name)
            (logs_dpath / f"{pkg_name}.log").unlink(missing_ok=True)
            logger.info("SUCCESS")
        else:
            progress["failed"].add(pkg_name)
            (logs_dpath / f"{pkg_name}.log").write_text(log)
            logger.info("FAILED")
        progress_fpath.write_text(yaml.safe_dump(progress))


def build_rpms(rosdistro: Rosdistro, retry: bool = False, stage0: bool = False):
    logs_dpath = Path(f"{LOGS_PATH}/build_rpms")
    logs_dpath.mkdir(parents=True, exist_ok=True)

    progress_fpath = logs_dpath / "_progress.yaml"
    progress_fpath.touch(exist_ok=True)
    progress = yaml.safe_load(progress_fpath.read_text()) or {
        "done": set(),
        "failed": set(),
        "skipped": set(),
    }
    progress["skipped"] = set()
    if retry:
        progress["failed"] = set()

    bp = yaml.safe_load(Path(BLUEPRINT_PATH.format(rosdistro=rosdistro)).read_text())
    pkg_infos = bp["pkgs"]

    G = nx.DiGraph()
    if stage0:
        G.add_edges_from(
            (
                ("ament_package", "ament_cmake_core"),
                ("ament_package", "ros_workspace"),
                ("ament_cmake_core", "ros_workspace"),
            )
        )
    else:
        G.add_edges_from(map(tuple, bp["graph"]))
    assert nx.is_directed_acyclic_graph(G)

    Path("RPMS").mkdir(exist_ok=True)
    for pkg_name in nx.topological_sort(G):
        logger.info(pkg_name)
        if pkg_name in progress["done"] or pkg_name in progress["failed"]:
            continue
        if not all(dep in progress["done"] for dep in G.predecessors(pkg_name)):
            progress["skipped"].add(pkg_name)
            progress_fpath.write_text(yaml.safe_dump(progress))
            logger.info("SKIPPED")
            continue
        info = pkg_infos[pkg_name]
        ok, log = _earthly_build(
            "rpm-build",
            [
                f"--url={info['git']}",
                f"--package={pkg_name}",
                f"--version={info['branch'].split('/')[-1]}",
                f"--rosdistro={rosdistro}",
                "--stage0=true" if stage0 else "--stage0=",
            ],
        )
        if ok:
            progress["done"].add(pkg_name)
            (logs_dpath / f"{pkg_name}.log").unlink(missing_ok=True)
            logger.info("SUCCESS")
        else:
            progress["failed"].add(pkg_name)
            (logs_dpath / f"{pkg_name}.log").write_text(log)
            logger.info("FAILED")
        progress_fpath.write_text(yaml.safe_dump(progress))

    if stage0:
        ok, log = _earthly_build("build-stage0-repo", [f"--rosdistro={rosdistro}"])
        assert ok, log


# def log_parse(log: str):
#     tags = {
#         "no_spec": lambda log: "SPECS: no such file or directory" in log,
#     }
#     for tag, test in tags.items():
#         if test(log):
#             return tag
