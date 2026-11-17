from glob import glob

from setuptools import setup

package_name = "pepper_cleaning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/models/charging_dock", glob("models/charging_dock/*")),
        ("share/" + package_name + "/models/dirt/materials/textures", glob("models/dirt/materials/textures/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KevinInoCol",
    maintainer_email="kevin.inofuente.colque.27@gmail.com",
    description="IA368 - Limpieza del museo: base de carga, batería, autodocking y piso sucio.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dock_station = pepper_cleaning.dock_station:main",
            "battery_sim = pepper_cleaning.battery_sim:main",
            "autodock = pepper_cleaning.autodock:main",
            "dirt_floor = pepper_cleaning.dirt_floor:main",
            "cleaner = pepper_cleaning.cleaner:main",
        ],
    },
)
