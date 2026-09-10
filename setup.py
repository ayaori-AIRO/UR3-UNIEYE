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
            "approach_retreat_demo = ur3_moveit_examples.demos.approach_retreat_demo:main",
            "arm_sweep_demo = ur3_moveit_examples.demos.arm_sweep_demo:main",
            "eye_in_hand_pick_and_place = ur3_moveit_examples.pick_place.eye_in_hand_pick_and_place:main",
            "fk_pick_and_place_demo = ur3_moveit_examples.pick_place.fk_pick_and_place_demo:main",
            "ik_pick_and_place_demo = ur3_moveit_examples.pick_place.ik_pick_and_place_demo:main",
            "move_cartesian_tool = ur3_moveit_examples.commands.move_cartesian_tool:main",
            "move_to_joint = ur3_moveit_examples.commands.move_to_joint:main",
            "move_to_pose = ur3_moveit_examples.commands.move_to_pose:main",
            "move_relative_tool = ur3_moveit_examples.commands.move_relative_tool:main",
            "pick_and_place_fk_demo = ur3_moveit_examples.pick_place.pick_and_place_fk_demo:main",
            "robot_state = ur3_moveit_examples.commands.robot_state:main",
            "sequence_demo = ur3_moveit_examples.demos.sequence_demo:main",
            "three_move_demo = ur3_moveit_examples.demos.three_move_demo:main",
            "weiss_gripper_read_state = ur3_moveit_examples.gripper.weiss_gripper_read_state:main",
            "weiss_gripper_reference = ur3_moveit_examples.gripper.weiss_gripper_reference:main",
            "weiss_gripper_release = ur3_moveit_examples.gripper.weiss_gripper_release:main",
            "weiss_gripper_grip = ur3_moveit_examples.gripper.weiss_gripper_grip:main",
            "yolo_live_detection = ur3_moveit_examples.vision.yolo_live_detection:main",
        ],
    },
)
