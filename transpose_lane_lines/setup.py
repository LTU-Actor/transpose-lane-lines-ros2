from glob import glob

from setuptools import setup

package_name = "transpose_lane_lines"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jake",
    maintainer_email="jake@todo.todo",
    description="Lane line transposition node for ROS2/Nav2 visualization",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "single_mask_lines = transpose_lane_lines.single_mask_lines:main",
            "both_mask_lines = transpose_lane_lines.both_mask_lines:main",
        ],
    },
)
