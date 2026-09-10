"""Safety-oriented inspection helpers for planned MoveIt trajectories."""

from moveit_msgs.msg import RobotState, RobotTrajectory


def joint_excursions(
    trajectory_start: RobotState, trajectory: RobotTrajectory
) -> dict[str, float]:
    """Return each joint's maximum absolute travel from the planned start."""
    joint_trajectory = trajectory.joint_trajectory
    if not joint_trajectory.joint_names or not joint_trajectory.points:
        return {}

    start_by_name = dict(
        zip(
            trajectory_start.joint_state.name,
            trajectory_start.joint_state.position,
        )
    )
    first_positions = dict(
        zip(joint_trajectory.joint_names, joint_trajectory.points[0].positions)
    )

    excursions = {}
    for index, name in enumerate(joint_trajectory.joint_names):
        start = start_by_name.get(name, first_positions[name])
        excursions[name] = max(
            abs(point.positions[index] - start)
            for point in joint_trajectory.points
        )
    return excursions


def trajectory_duration_seconds(trajectory: RobotTrajectory) -> float:
    """Return the timestamp of the final joint trajectory point in seconds."""
    points = trajectory.joint_trajectory.points
    if not points:
        return 0.0
    duration = points[-1].time_from_start
    return duration.sec + duration.nanosec * 1.0e-9
