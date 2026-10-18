from glob import glob

from setuptools import setup

package_name = "pepper_multilaser_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KevinInoCol",
    maintainer_email="kevin.inofuente.colque.27@gmail.com",
    description="IA368 - Fase 1: SLAM con los 3 láseres de Pepper.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scan_fusion = pepper_multilaser_slam.scan_fusion:main",
            "route_driver = pepper_multilaser_slam.route_driver:main",
            "trajectory_recorder = pepper_multilaser_slam.trajectory_recorder:main",
            "evaluate = pepper_multilaser_slam.evaluate:main",
            "odom_noise = pepper_multilaser_slam.odom_noise:main",
        ],
    },
)
