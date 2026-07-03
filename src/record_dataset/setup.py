from glob import glob
from setuptools import setup

package_name = "record_dataset"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="raphael",
    maintainer_email="alec.bossard@utoulouse.fr",
    description="Launch files for dataset recording.",
    license="Apache-2.0",
)