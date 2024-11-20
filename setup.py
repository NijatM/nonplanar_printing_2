from setuptools import setup, find_packages

import io
from os import path

from setuptools import find_packages
from setuptools import setup

here = path.abspath(path.dirname(__file__))


def read(*names, **kwargs):
    return io.open(
        path.join(here, *names), encoding=kwargs.get("encoding", "utf8")
    ).read()


requirements = read("requirements.txt").split("\n")

setup(
    name="nonplanar_printing_2",  # Replace with your project name
    version="0.1.0",
    description="Planning code for non-planar 3D printing",
    # Automatically find the packages in current directory
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    # Dependencies
    install_requires=requirements,
    # Local packages
    author="Victor Leung",
)
