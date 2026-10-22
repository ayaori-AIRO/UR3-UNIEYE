import ast
from pathlib import Path
import statistics
import unittest

p = Path(__file__).parents[1]/'ur3_moveit_examples/demos/joint_repeatability.py'
ns = {'statistics': statistics}
exec(compile(ast.Module(body=[n for n in ast.parse(p.read_text()).body
    if isinstance(n,ast.FunctionDef) and n.name in ('stationary','summarize')],
    type_ignores=[]),str(p),'exec'),ns)


class TestRepeatability(unittest.TestCase):
    def test_stationary(self):
        rows=[(i*100_000_000,[0.]*6) for i in range(11)]
        self.assertEqual(ns['stationary'](rows,1_000_000_000),rows[-1])
        for invalid in ([],rows[:3],rows[::3]):
            with self.assertRaises(RuntimeError):
                ns['stationary'](invalid,1_000_000_000)
        rows[-1][1][0]=.01
        with self.assertRaises(RuntimeError):
            ns['stationary'](rows,1_000_000_000)

    def test_summary(self):
        rows=[[1,'a','b',0,*([0]*6),1.,2.,3.,0,0,0,1,0],
              [2,'a','b',0,*([0]*6),1.002,2.,3.,0,0,0,1,0],
              [0,'a','home',0,*([0]*6),9.,9.,9.,0,0,0,1,0]]
        result=ns['summarize'](rows)
        self.assertEqual(result['b->a']['count'],2)
        self.assertAlmostEqual(result['b->a']['range_xyz_mm'][0],2.)
        self.assertAlmostEqual(result['b->a']['sample_std_xyz_mm'][0],2**.5)
        self.assertIsNone(result['home->a']['sample_std_xyz_mm'][0])


if __name__=='__main__':
    unittest.main()
