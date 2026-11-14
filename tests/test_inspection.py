import json
from pathlib import Path
import unittest
import numpy as np
from construction_slam.core import transform
from construction_slam.inspection import ElementInspector

DESIGN=json.loads((Path(__file__).resolve().parents[1]/'config/design.json').read_text())


def perimeter(pose):
    x=np.linspace(-.5,.5,60); y=np.linspace(-.35,.35,45)
    points=np.vstack([np.column_stack([x,np.full_like(x,-.35)]),
                      np.column_stack([x,np.full_like(x,.35)]),
                      np.column_stack([np.full_like(y,-.5),y]),
                      np.column_stack([np.full_like(y,.5),y])])
    return transform(points,pose)


class InspectionTests(unittest.TestCase):
    def inspect(self,points):
        inspector=ElementInspector(DESIGN,np.array([[20.,20.],[21.,20.]]))
        for i in range(3): report=inspector.update(points,i)
        return inspector,report

    def test_correct_placement(self):
        _,report=self.inspect(perimeter(DESIGN['design_pose']))
        self.assertEqual(report['status'],'within_tolerance')
        self.assertLess(report['translation_deviation_m'],.001)

    def test_translation_and_rotation(self):
        pose=np.array(DESIGN['design_pose'])+[.12,-.08,np.deg2rad(8)]
        _,report=self.inspect(perimeter(pose))
        self.assertEqual(report['status'],'deviation_detected')
        np.testing.assert_allclose(report['estimated_pose'],pose,atol=.005)

    def test_single_face_is_insufficient(self):
        p=np.column_stack([np.linspace(-.4,.4,70),np.full(70,.35)])
        _,report=self.inspect(transform(p,DESIGN['design_pose']))
        self.assertEqual(report['status'],'insufficient_evidence')
        self.assertIsNone(report['estimated_pose'])

    def test_empty_and_stale(self):
        inspector=ElementInspector(DESIGN,np.array([[20.,20.]]))
        self.assertEqual(inspector.update(np.empty((0,2)),0)['status'],'insufficient_evidence')
        inspector,_=self.inspect(perimeter(DESIGN['design_pose']))
        self.assertEqual(inspector.report(40)['status'],'stale_observation')

    def test_baseline_surfaces_not_elements(self):
        points=perimeter(DESIGN['design_pose'])
        inspector=ElementInspector(DESIGN,points)
        for i in range(4): report=inspector.update(points,i)
        self.assertEqual(report['point_count'],0)
        self.assertEqual(report['status'],'insufficient_evidence')
