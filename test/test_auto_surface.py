"""Pure helper tests without ROS initialization or model loading."""
import ast
from pathlib import Path
import unittest
import cv2
import numpy as np

source = Path(__file__).parents[1] / 'ur3_moveit_examples/vision/scissors_position.py'
tree = ast.parse(source.read_text())
namespace = {'np': np, 'cv2': cv2}
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
     and n.name in ('automatic_pixel', 'stable_auto_point')], type_ignores=[]),
     str(source), 'exec'), namespace)


class AutoTests(unittest.TestCase):
    def test_confidence_history(self):
        f = namespace['stable_auto_point']
        samples = [(i, c, np.zeros(3)) for i,c in enumerate([.6,.4,.4,.6,.4])]
        self.assertIsNotNone(f(samples, 5))
        self.assertIsNotNone(f([None] + samples + [None], 5))
        self.assertIsNone(f(samples[:4], 5))
        self.assertIsNone(f([(i,.4,np.zeros(3)) for i in range(5)], 5))
        self.assertIsNone(f(samples, 20))
        self.assertIsNone(f(samples[:-1]+[(4,.6,np.array([.1,0,0]))],5))

    def test_foreground(self):
        frame = np.full((100,100,3), 240, np.uint8)
        frame[30:70,40:60] = (0,0,180)
        u,v = namespace['automatic_pixel'](frame, [20,20,80,80])
        self.assertTrue(40 <= u < 60 and 30 <= v < 70)

    def test_small_box(self):
        with self.assertRaises(ValueError):
            namespace['automatic_pixel'](np.zeros((30,30,3),np.uint8),[1,1,4,4])


if __name__ == '__main__':
    unittest.main()
