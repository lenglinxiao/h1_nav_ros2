from glob import glob

from setuptools import find_packages, setup


package_name = "h1_2_bag_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Rosbag regression tools for H1-2 FAST-LIO mapping and localization.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "slam_bag_evaluator = h1_2_bag_benchmark.evaluator:main",
            "pcd_map_compare = h1_2_bag_benchmark.pcd_compare:main",
        ],
    },
)
