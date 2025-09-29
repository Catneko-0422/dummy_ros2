from setuptools import setup, find_packages
setup(
    name="dummy_robot_bridge",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dummy_robot_bridge"]),
        ("share/dummy_robot_bridge", ["package.xml", "README.md"]),
        ("share/dummy_robot_bridge/launch", ["launch/bridge.launch.py"]),
        ("share/dummy_robot_bridge/srv", [
            "srv/MoveJ.srv","srv/MoveL.srv","srv/MoveLPartial.srv",
            "srv/SetTwoFloat64.srv","srv/SetUInt32.srv","srv/SetFloat64.srv"
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="You",
    maintainer_email="you@example.com",
    description="Dummy Robot ROS2 Bridge (fibre integrated)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "discover_node.py = scripts.discover_node:main",
            "dummy_node.py = scripts.dummy_node:main",
        ],
    },
)
