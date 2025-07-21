import os
from glob import glob
from setuptools import setup

package_name = "wave_rover_ros_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas Costa",
    maintainer_email="lucas.costa@ee.ufcg.edu.br",
    description="Wave rover ros driver",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wave_rover_ros_driver = wave_rover_ros_driver.wave_rover_ros_driver:main",
        ],
    },
)
