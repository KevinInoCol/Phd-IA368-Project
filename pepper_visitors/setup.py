from setuptools import setup

package_name = "pepper_visitors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/" + package_name, ["scripts/voice_listener"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KevinInoCol",
    maintainer_email="kevin.inofuente.colque.27@gmail.com",
    description="IA368 - Fase 3: llamada por voz, búsqueda y acercamiento a visitantes.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "capture_dataset = pepper_visitors.capture_dataset:main",
        ],
    },
)
