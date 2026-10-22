import ast
import math
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock

import pytest


def module():
    path = Path(__file__).parents[1] / 'ur3_moveit_examples/pick_place/connector_pick.py'
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.Assign) or
             isinstance(n, ast.FunctionDef) and n.name != 'main']
    env = dict(math=math, subprocess=N(run=Mock()), HOME_TCP_POSE=(0,)*7, HOME_JOINTS=(0,)*6)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), env)
    return env


def pose(values):
    return N(position=N(x=values[0], y=values[1], z=values[2]),
             orientation=N(x=values[3], y=values[4], z=values[5], w=values[6]))


def test_vertical_targets():
    m = module()
    for ref in (m['PICK'], m['PLACE']):
        up = m['descent_target'](pose(ref), .05, ref, lift=True)
        assert up[:2] == ref[:2] and up[3:] == ref[3:]
        assert up[2] == pytest.approx(ref[2]+.05)
        assert m['descent_target'](pose(up), .05, ref) == ref
        bad = pose(up)
        bad.position.x += .01
        with pytest.raises(RuntimeError):
            m['descent_target'](bad, .05, ref)


def test_sequence():
    m = module()
    assert m['stage_sequence']('after-pick', False) == ('lift',)
    assert m['stage_sequence']('full', False) == ('home',)
    assert m['stage_sequence']('full', True) == (
        'home','approach','descend','lift','place-approach','place-descend','place-lift','home')


def test_io_only_after_successful_executed_descent():
    m = module()
    cli = N(approach_height=.05, execute=True, hold_time=1.)
    robot = Mock()
    robot.get_current_pose.return_value = N(pose=pose(m['approach_pose'](.05,m['PLACE'])))
    robot.move_cartesian_to_pose.return_value = False
    with pytest.raises(RuntimeError):
        m['run_stage'](robot,'place-descend',cli)
    m['subprocess'].run.assert_not_called()
    robot.move_cartesian_to_pose.return_value = True
    cli.execute = False
    m['run_stage'](robot,'place-descend',cli)
    m['subprocess'].run.assert_not_called()
    cli.execute = True
    m['run_stage'](robot,'place-descend',cli)
    assert m['subprocess'].run.call_args.args[0][4] == 'open'


def test_home_skip_and_fallback():
    m = module()
    m['HOME_TCP_POSE'] = (0., 0., 0., 0., 0., 0., 1.)
    m['UR_JOINT_NAMES'] = list('abcdef')
    robot = Mock()
    robot.verify_joint_tolerance = .01
    robot.verify_position_tolerance = .001
    robot.verify_orientation_tolerance = .01
    robot._get_fresh_joint_positions.return_value = dict.fromkeys('abcdef', .000097)
    robot.get_clock.return_value.now.return_value.nanoseconds = 10**9
    current = N(header=N(stamp=N(sec=1, nanosec=0)), pose=pose((0.,0.,0.,0.,0.,0.,-1.)))
    robot.get_current_pose.return_value = current
    for execute in (False, True):
        m['run_stage'](robot, 'home', N(execute=execute))
    robot.move_to_pose.assert_not_called()
    m['subprocess'].run.assert_not_called()
    current.pose.position.x = .002
    m['run_stage'](robot, 'home', N(execute=False, home_max_joint_travel=2.25))
    robot.move_to_pose.assert_called_once()
    current.pose.position.x = 0.
    current.header.stamp.sec = 0
    assert not m['already_home'](robot, dict.fromkeys('abcdef', 0.))
    current.header.stamp.sec = 1
    assert not m['already_home'](robot, dict.fromkeys('abcdef', 1.))
    current.pose.orientation.w = float('nan')
    assert not m['already_home'](robot, dict.fromkeys('abcdef', 0.))


@pytest.mark.parametrize('failure', [False, True])
def test_home_limit_restored(failure):
    m = module()
    m['already_home'] = lambda robot, joints: False
    robot = Mock()
    robot.max_joint_travel = 2.15
    def move(*args, **kwargs):
        assert robot.max_joint_travel == 2.25
        if failure:
            raise RuntimeError('planning failed')
        return True
    robot.move_to_pose.side_effect = move
    cli = N(execute=False, home_max_joint_travel=2.25)
    if failure:
        with pytest.raises(RuntimeError):
            m['run_stage'](robot, 'home', cli)
    else:
        m['run_stage'](robot, 'home', cli)
    assert robot.max_joint_travel == 2.15
