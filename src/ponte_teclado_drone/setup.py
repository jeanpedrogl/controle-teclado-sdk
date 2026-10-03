from setuptools import find_packages, setup

package_name = "ponte_teclado_drone"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sistema.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jean Pedro Grossi Líbero",
    maintainer_email="jeanpedrogrossi@gmail.com",
    description="Pilota o drone DJI Mini SE pelo teclado e mouse do computador (ponte ROS2 para o celular).",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "piloto_teclado_mouse = ponte_teclado_drone.piloto_teclado_mouse:main",
        ],
    },
)
