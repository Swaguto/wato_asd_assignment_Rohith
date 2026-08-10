from setuptools import setup

package_name = "urdf_relay"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rohith",
    maintainer_email="rohith@todo.todo",
    description="Relays the latched /robot_description into a live volatile topic for Foxglove",
    license="MIT",
    entry_points={
        "console_scripts": [
            "urdf_relay = urdf_relay.urdf_relay:main",
        ],
    },
)