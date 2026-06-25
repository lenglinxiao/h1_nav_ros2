from glob import glob

from setuptools import find_packages, setup


package_name = "h1_2_vision"

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
    description="CPU YOLO object detection node for the H1-2 RealSense camera.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_detector = h1_2_vision.yolo_detector:main",
        ],
    },
)
