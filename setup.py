from setuptools import find_packages, setup


package_name = "ur3_moveit_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UR3 user",
    maintainer_email="user@example.com",
    description="Minimal MoveIt 2 examples for a real UR3.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_cartesian_tool = ur3_moveit_examples.move_cartesian_tool:main",
            "move_to_joint = ur3_moveit_examples.move_to_joint:main",
            "move_to_pose = ur3_moveit_examples.move_to_pose:main",
            "move_relative_tool = ur3_moveit_examples.move_relative_tool:main",
            "robot_state = ur3_moveit_examples.robot_state:main",
            "sequence_demo = ur3_moveit_examples.sequence_demo:main",
        ],
    },
)
