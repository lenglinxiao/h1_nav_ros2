from glob import glob
from setuptools import find_packages, setup

package_name = "h1_2_voice_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Robot-side H1-2 speech module reader.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "h1_2_voice_control = h1_2_voice_control.h1_2_voice_control:main",
        ],
    },
)
