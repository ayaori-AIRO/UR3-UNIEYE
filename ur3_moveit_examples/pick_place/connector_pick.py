"""Taught connector pick: seeded IK then vertical Cartesian descent. Default plan only."""
import argparse
import math
import subprocess

import rclpy
from ur3_moveit_examples.core.ur3_moveit_controller import UR3MoveItController, UR_JOINT_NAMES
from ur3_moveit_examples.pick_place.eye_in_hand_pick_and_place import HOME_JOINTS, HOME_TCP_POSE

PICK = (-0.177157, -0.358597, 0.014096, 0.695640, 0.718364, -0.005236, 0.003284)
PICK_SEED = (1.394760, -2.555081, -1.694334, -0.449862, 1.563661, 1.358398)
PLACE = (-0.288437, -0.419072, 0.016113, 0.689349, 0.723857, -0.027441, 0.008710)
PLACE_SEED = (1.190163, -2.859672, -0.909391, -0.885182, 1.556013, 1.137073)
AFTER_PICK = ('lift', 'place-approach', 'place-descend', 'place-lift', 'home')


def stage_sequence(stage, execute):
    stages = (('home', 'approach', 'descend', *AFTER_PICK) if stage == 'full'
              else AFTER_PICK if stage == 'after-pick' else (stage,))
    return stages if execute else stages[:1]


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('Expected positive finite number')
    return number


def approach_pose(height, reference=PICK):
    return (*reference[:2], reference[2] + height, *reference[3:])


def descent_target(pose, height, reference=PICK, lift=False):
    """Keep measured XY/orientation: never silently add a lateral correction."""
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(map(math.isfinite, values)):
        raise RuntimeError('Invalid TCP pose')
    start_z = reference[2] if lift else reference[2] + height
    if math.hypot(p.x-reference[0], p.y-reference[1]) > .001 or abs(p.z-start_z) > .002:
        raise RuntimeError('Not at approach: XY must be within 1 mm, Z within 2 mm')
    norm = math.sqrt(sum(v*v for v in values[3:]))
    target_norm = math.sqrt(sum(v*v for v in reference[3:]))
    if norm < 1e-9:
        raise RuntimeError('Invalid orientation')
    dot = abs(sum(a*b for a,b in zip(values[3:], reference[3:])) / (norm*target_norm))
    if 2*math.acos(min(1., dot)) > .01:
        raise RuntimeError('Approach orientation differs by more than 0.01 rad')
    return (p.x, p.y, p.z + height if lift else reference[2], q.x, q.y, q.z, q.w)


def already_home(robot, joints):
    # Require the taught joint branch as well as TCP proximity.
    if any(n not in joints or not math.isfinite(joints[n]) or
           abs(joints[n]-target) > robot.verify_joint_tolerance
           for n, target in zip(UR_JOINT_NAMES, HOME_JOINTS)):
        return False
    current = robot.get_current_pose()
    if current is None:
        return False
    stamp = current.header.stamp
    age = (robot.get_clock().now().nanoseconds -
           (stamp.sec * 10**9 + stamp.nanosec)) * 1e-9
    if not 0 <= age <= .5:
        return False
    p, q = current.pose.position, current.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(map(math.isfinite, values)):
        return False
    norm = math.sqrt(sum(v*v for v in values[3:]))
    target_norm = math.sqrt(sum(v*v for v in HOME_TCP_POSE[3:]))
    if norm < 1e-9 or target_norm < 1e-9:
        return False
    position_error = math.dist(values[:3], HOME_TCP_POSE[:3])
    dot = abs(sum(a*b for a,b in zip(values[3:], HOME_TCP_POSE[3:])) / (norm*target_norm))
    angle = 2*math.acos(min(1., dot))
    if position_error > robot.verify_position_tolerance or angle > robot.verify_orientation_tolerance:
        return False
    robot.get_logger().info(f'Already at HOME: skipping motion; position_error={position_error:.6f} m, '
                            f'orientation_error={angle:.6f} rad')
    return True


def run_stage(robot, stage, cli):
    joints = robot._get_fresh_joint_positions()
    if joints is None:
        raise RuntimeError('No fresh joint state')
    if stage == 'home':
        if already_home(robot, joints):
            return
        original_limit = robot.max_joint_travel
        try:
            robot.max_joint_travel = cli.home_max_joint_travel
            robot.get_logger().info(f'HOME-only joint travel limit: {robot.max_joint_travel:.3f} rad')
            ok = robot.move_to_pose(*HOME_TCP_POSE, ik_seed_positions=HOME_JOINTS, execute=cli.execute)
        finally:
            robot.max_joint_travel = original_limit
    elif stage in ('approach', 'place-approach'):
        reference, seed = (PLACE, PLACE_SEED) if stage == 'place-approach' else (PICK, PICK_SEED)
        ok = robot.move_to_pose(*approach_pose(cli.approach_height, reference),
                                ik_seed_positions=seed, execute=cli.execute)
    else:
        pose = robot.get_current_pose()
        if pose is None:
            raise RuntimeError('No current TCP pose')
        if stage not in ('descend', 'lift', 'place-descend', 'place-lift'):
            raise ValueError(f'Unknown stage: {stage}')
        reference = PLACE if stage.startswith('place-') else PICK
        target = descent_target(pose.pose, cli.approach_height, reference,
                                lift=stage in ('lift', 'place-lift'))
        ok = robot.move_cartesian_to_pose(*target, execute=cli.execute,
                                         max_step=.001, minimum_fraction=1.0)
    if not ok:
        raise RuntimeError(f'{stage} failed; no next stage or gripper command')
    if cli.execute and stage in ('descend', 'place-descend'):
        action = 'close' if stage == 'descend' else 'open'
        subprocess.run(['ros2', 'run', 'ur3_moveit_examples', 'zimmer_gripper', action,
                        '--execute', '--hold-time', str(cli.hold_time)], check=True)
        print(f'{action} outputs completed; physical grasp/release NOT measured.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', choices=('home','approach','descend','lift',
                        'place-approach','place-descend','place-lift','after-pick','full'), default='full')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--confirm-clearance', action='store_true',
                        help='Confirm appropriate jaw state, payload clearance and safe initial departure')
    parser.add_argument('--approach-height', type=positive, default=.05)
    parser.add_argument('--velocity-scaling', type=positive, default=.03)
    parser.add_argument('--acceleration-scaling', type=positive, default=.03)
    parser.add_argument('--max-joint-travel', type=positive, default=2.15)
    parser.add_argument('--home-max-joint-travel', type=positive, default=2.25,
                        help='HOME-only IK and trajectory travel limit [rad]')
    parser.add_argument('--hold-time', type=positive, default=1.)
    cli = parser.parse_args()
    if max(cli.velocity_scaling,cli.acceleration_scaling) > 1:
        parser.error('scaling must be <= 1')
    if cli.execute and not cli.confirm_clearance:
        parser.error('Execution requires --confirm-clearance; full needs open jaws, after-pick needs held part')
    rclpy.init()
    robot = UR3MoveItController(node_name='connector_pick', planning_time=10.,
        velocity_scaling=cli.velocity_scaling, acceleration_scaling=cli.acceleration_scaling,
        max_joint_travel=cli.max_joint_travel, ik_joint_tolerance=.00001,
        verify_position_tolerance=.001, verify_orientation_tolerance=.01)
    try:
        print('base_link -> tool0 targets. Full/after-pick ends at HOME; confirm held-part clearance.')
        stages = stage_sequence(cli.stage, cli.execute)
        if not cli.execute:
            print(f'PLAN ONLY: current -> {stages[0]} only; no I/O. Later legs are NOT checked.')
        for stage in stages:
            run_stage(robot, stage, cli)
    except (Exception, KeyboardInterrupt) as exc:
        robot.get_logger().error(f'Stopped: {exc}. Check robot/outputs on pendant; no automatic recovery.')
        raise SystemExit(1)
    finally:
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
