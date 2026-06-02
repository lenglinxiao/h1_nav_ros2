from glob import glob
from setuptools import find_packages, setup

package_name = "h1_2_nav_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/path", glob("path/*.txt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="ROS2 navigation helper nodes migrated from H1-2Nav2D.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "path_to_waypoint = h1_2_nav_tools.path_to_waypoint:main",
            "play_teaching_path = h1_2_nav_tools.play_teaching_path:main",
            "record_teaching_path = h1_2_nav_tools.record_teaching_path:main",
            "local2map_odom = h1_2_nav_tools.local2map_odom:main",
            "velocity_smoother_ema = h1_2_nav_tools.velocity_smoother_ema:main",
            "costmap_clear_on_initialpose = h1_2_nav_tools.costmap_clear_on_initialpose:main",
            "pointcloud_to_occupancy_grid = h1_2_nav_tools.pointcloud_to_occupancy_grid:main",
            "simple_grid_navigator = h1_2_nav_tools.simple_grid_navigator:main",
            "slam_grid_navigator = h1_2_nav_tools.slam_grid_navigator:main",
            "cmd_vel_to_sport_mode = h1_2_nav_tools.cmd_vel_to_sport_mode:main",
        ],
    },
)
