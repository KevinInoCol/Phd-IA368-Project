from glob import glob

from setuptools import setup

package_name = "pepper_fetch"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models/apple", glob("models/apple/model.*")),
        ("share/" + package_name + "/models/apple/meshes", glob("models/apple/meshes/*")),
        ("share/" + package_name + "/models/fruit_table", glob("models/fruit_table/*")),
        ("lib/" + package_name, glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KevinInoCol",
    maintainer_email="kevin.inofuente.colque.27@gmail.com",
    description="IA368 - Pepper va a la mesa, ve la manzana con YOLO y la coge con la mano.",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
