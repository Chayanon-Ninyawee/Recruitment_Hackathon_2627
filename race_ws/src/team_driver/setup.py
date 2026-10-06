import os
from glob import glob

from setuptools import setup

package_name = "team_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thunder McKing",
    maintainer_email="C260080@e.ntu.edu.sg",
    description="Hackathon entry.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Keep this entry: the judges run `ros2 run team_driver driver`.
            # Add more of your own alongside it if you like.
            "driver = team_driver.driver:main",
        ],
    },
)
