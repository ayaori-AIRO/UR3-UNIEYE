import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS


def load():
    source = Path(__file__).parents[1] / 'ur3_moveit_examples/demos/joint_repeatability.py'
    tree = ast.parse(source.read_text())
    class Base:
        def _validate_planned_joint_goal(self, *args):
            return self.accept
        def get_logger(self):
            return NS(error=lambda text: None)
    env = {'math': math, 'UR3MoveItController': Base, 'UR_JOINT_NAMES': list('abcdef')}
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
             and n.name in ('endpoint_errors', 'RepeatabilityController')]
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), env)
    return env


def test_errors():
    fn = load()['endpoint_errors']
    planning, execution = fn([0]*6, [1]*6, [3]*6)
    assert planning == [1]*6
    assert execution == [2]*6


def test_endpoint_gate_and_reset():
    robot = load()['RepeatabilityController']()
    robot.accept = True
    trajectory = NS(joint_names=list('fedcba'), points=[NS(positions=[0]*6)])
    result = NS(planned_trajectory=NS(joint_trajectory=trajectory))
    assert robot._validate_planned_joint_goal(result, [0]*6, 1e-6)
    assert robot.planned_endpoint == [0]*6
    trajectory.points[0].positions[0] = 2e-6
    assert not robot._validate_planned_joint_goal(result, [0]*6, 1e-6)
    assert robot.planned_endpoint is None
    robot.accept = False
    assert not robot._validate_planned_joint_goal(result, [0]*6, 1e-6)
