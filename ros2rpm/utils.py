def sanitize_pkgname(name: str) -> str:
    return name.replace("_", "-")


def rosify_pkgname(pkgname: str, rosdistro: str) -> str:
    return f"ros-{rosdistro}-{pkgname}"
