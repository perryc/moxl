from setuptools import setup, find_packages

package_name = "toolpath_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyproj", "shapely", "numpy"],
    zip_safe=True,
    maintainer="Perry",
    maintainer_email="4perryc@gmail.com",
    description="Airstrip mowing toolpath planner for MOXL",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "toolpath_node = nodes.toolpath_node:main",
            "heading_to_imu_node = nodes.heading_to_imu_node:main",
            "engine_controller_node = nodes.engine_controller_node:main",
            "blade_controller_node = nodes.blade_controller_node:main",
            "mission_node = nodes.mission_node:main",
            "safety_monitor_node = nodes.safety_monitor_node:main",
            "sdr_detector_node = nodes.sdr_detector_node:main",
        ],
    },
)
