import os
from glob import glob
from setuptools import setup

package_name = "go_to_goal"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas Costa",
    maintainer_email="lucas.costa@ee.ufcg.edu.br",
    description="Go to goal Mocap",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go_to_goal = go_to_goal.go_to_goal:main",
        ],
    },
)
