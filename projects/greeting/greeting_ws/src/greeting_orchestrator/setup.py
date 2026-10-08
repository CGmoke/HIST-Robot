from glob import glob
import os

from setuptools import find_packages, setup

package_name = "greeting_orchestrator"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="moke",
    maintainer_email="3059342114@qq.com",
    description="迎宾编排层：唯一指挥者 / 唯一状态机",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator_node = greeting_orchestrator.orchestrator_node:main",
        ],
    },
)