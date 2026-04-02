from pathlib import Path

from setuptools import find_packages, setup


package_name = "lktech_motors_testing"


def package_files(source_dir: str, install_root: str) -> list[tuple[str, list[str]]]:
    root = Path(source_dir)
    if not root.exists():
        return []

    collected: dict[str, list[str]] = {}
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if "__pycache__" in path.parts:
            continue
        install_dir = Path(install_root) / path.parent
        collected.setdefault(str(install_dir), []).append(str(path))
    return sorted(collected.items())


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml", "README.md"]),
]
data_files.extend(package_files("launch", f"share/{package_name}"))
data_files.extend(package_files("config", f"share/{package_name}"))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    scripts=["scripts/cansniffer.py"],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="gerardo",
    maintainer_email="gerardodelcid16@gmail.com",
    description="Windows-first LKTech motor test utilities built as an ament_python package.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lktech_monitor = lktech_motors_testing.monitor_node:main",
            "lktech_position_test = lktech_motors_testing.position_test_node:main",
            "lktech_cli_read_angle = lktech_motors_testing.cli_read_angle:main",
            "lktech_cli_position_test = lktech_motors_testing.cli_position_test:main",
            "lktech_can_health_check = lktech_motors_testing.cli_can_health_check:main",
        ],
    },
)
