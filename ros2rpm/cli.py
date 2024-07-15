import os
from pathlib import Path

import click

from ._const import Rosdistro
from .main import build_rpms, gen, gen_spec, resolve_dist

rosdistros: list[Rosdistro] = ["humble", "iron", "jazzy"]


def with_earthly(func):
    @click.argument(
        "workdir", type=click.Path(exists=True, file_okay=False, resolve_path=True), default="."
    )
    def inner(workdir, *args, **kwargs):
        if workdir:
            os.chdir(workdir)
        return func(*args, **kwargs)

    return inner


@click.group()
def cli():
    pass


@cli.command("spec")
@click.argument("pkg_dir")
@click.argument("dst_path")
@click.option("--rosdistro", required=True, type=click.Choice(rosdistros))
@click.option("--os", required=True)
@click.option("--rpm-inc", default=0, type=int)
def cli_spec(pkg_dir: str, dst_path: str, rosdistro: str, os: str, rpm_inc: int):
    gen_spec(Path(pkg_dir), Path(dst_path), rosdistro, os, rpm_inc)


@cli.command("resolve")
@click.argument("dist_file")
@click.option("--rosdistro", required=True, type=click.Choice(rosdistros))
def cli_resolve(dist_file: str, rosdistro: Rosdistro):
    resolve_dist(Path(dist_file), rosdistro)


@cli.command("init")
@with_earthly
def cli_init():
    print(os.getcwd())
    from importlib.resources import as_file, files

    with as_file(files("ros2rpm.earthfile").joinpath("Earthfile")) as fpath:
        Path("Earthfile").write_text(fpath.read_text())
    return


@cli.command("gen")
@click.option("--rosdistro", required=True, type=click.Choice(rosdistros))
@click.option("--retry", is_flag=True, default=False)
@with_earthly
def cli_gen(rosdistro: Rosdistro, retry: bool):
    if not Path("Earthfile").is_file():
        click.echo("error: please init first.", err=True)
        return
    gen(rosdistro, retry)


@cli.command("rpmbuild")
@click.option("--rosdistro", required=True, type=click.Choice(rosdistros))
@click.option("--arch", required=True, type=click.Choice(["arm64", "amd64", "riscv64"]))
@click.option("--stage0", is_flag=True, default=False)
@click.option("--retry", is_flag=True, default=False)
@with_earthly
def cli_rpmbuild(rosdistro: Rosdistro, arch: str, retry: bool, stage0: bool):
    if not Path("Earthfile").is_file():
        click.echo("error: please init first.", err=True)
        return
    build_rpms(rosdistro, arch, retry, stage0)
