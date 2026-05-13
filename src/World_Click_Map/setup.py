from setuptools import setup

package_name = "World_Click_Map"

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
    maintainer="AUV Bachelor Group",
    maintainer_email="auv-bachelor@users.noreply.github.com",
    description="Leaflet map HTTP server for AUV route planning",
    license="MIT",
    entry_points={
        "console_scripts": [
            "World_Click_Map = World_Click_Map.World_Click_Map:main",
        ],
    },
)
