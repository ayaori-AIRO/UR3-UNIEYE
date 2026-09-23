import ast
from pathlib import Path
import unittest

path = Path(__file__).parents[1] / 'ur3_moveit_examples/gripper/zimmer_gripper.py'
namespace = {}
exec(compile(ast.Module(body=[n for n in ast.parse(path.read_text()).body
     if isinstance(n, ast.FunctionDef) and n.name == 'run_command'], type_ignores=[]),
     str(path), 'exec'), namespace)


class Fake:
    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail
    def set_output(self, pin, high):
        self.calls.append((pin,high))
        if len(self.calls) == self.fail:
            raise RuntimeError('injected failure')
    def pause(self, duration):
        self.calls.append(('wait',duration))


class Tests(unittest.TestCase):
    def test_directions(self):
        for action,pin in [('open',1),('close',2)]:
            fake = Fake()
            namespace['run_command'](fake,action,1.0)
            self.assertEqual(fake.calls,[(1,False),(2,False),('wait',.05),
                             (pin,True),('wait',1.0),(1,False),(2,False)])
    def test_failure_cleanup(self):
        for failure in (1,2,4):
            fake = Fake(failure)
            with self.assertRaises(RuntimeError):
                namespace['run_command'](fake,'open',1.0)
            self.assertEqual(fake.calls[-2:],[(1,False),(2,False)])
            if failure < 4:
                self.assertFalse(any(high is True for _,high in fake.calls))
    def test_off(self):
        fake = Fake()
        namespace['run_command'](fake,'off',None)
        self.assertFalse(any(high is True for _,high in fake.calls))


if __name__ == '__main__':
    unittest.main()
