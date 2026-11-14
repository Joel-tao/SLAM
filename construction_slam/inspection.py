"""Known-size rectangular element inspection in a pre-aligned design frame.

Single-element geometric association near the design pose, not object recognition.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


def box_distance(points, pose, size):
    c,s=np.cos(pose[2]),np.sin(pose[2])
    local=(points-pose[:2]) @ np.array([[c,-s],[s,c]])
    q=np.abs(local)-np.asarray(size)/2
    return np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)


class ElementInspector:
    def __init__(self, design, baseline):
        self.design=design
        self.target=np.array(design['design_pose'])
        self.baseline=cKDTree(baseline)
        self.points=np.empty((0,2))
        self.observations=0
        self.last_seen=None

    def update(self, points, frame):
        distance,_=self.baseline.query(points)
        roi=np.linalg.norm(points-self.target[:2],axis=1)<1.2
        candidate=points[roi & (distance>.18)]
        if len(candidate)>=3:
            self.observations+=1
            self.last_seen=frame
            merged=np.vstack([self.points,candidate])
            _,ids=np.unique(np.floor(merged/.025).astype(int),axis=0,return_index=True)
            self.points=merged[ids]
        return self.report(frame)

    def report(self, frame):
        report=dict(element_id=self.design['element_id'],frame_id=self.design['frame_id'],
                    frame=int(frame),status='insufficient_evidence',
                    next_action='observe_element',last_seen_frame=self.last_seen,
                    observation_frames=self.observations,point_count=len(self.points),
                    design_pose=self.target.tolist(),estimated_pose=None,
                    translation_deviation_m=None,yaw_deviation_deg=None,
                    assumption='Single stationary known-size rectangle; pre-aligned design; local association')
        if self.observations<3 or len(self.points)<25:
            return report
        bound=np.array([.5,.5,np.deg2rad(25)])
        result=least_squares(lambda p: box_distance(self.points,p,self.design['size_m']),
                             self.target,bounds=(self.target-bound,self.target+bound),
                             loss='soft_l1',f_scale=.025)
        residual=box_distance(self.points,result.x,self.design['size_m'])
        singular=np.linalg.svd(result.jac,compute_uv=False)
        rms=float(np.sqrt(np.mean(residual**2)))
        report['fit_rmse_m']=rms
        # One visible flat face cannot constrain tangential displacement.
        if (not result.success or singular[-1]<.4 or rms>.06 or
                np.any(np.abs(result.x-self.target)>.98*bound)):
            return report
        delta=result.x-self.target
        translation=float(np.linalg.norm(delta[:2]))
        angle=float(np.rad2deg(delta[2]))
        deviated=(translation>self.design['translation_tolerance_m'] or
                  abs(angle)>self.design['yaw_tolerance_deg'])
        report.update(estimated_pose=result.x.tolist(),translation_deviation_m=translation,
                      yaw_deviation_deg=angle,status='deviation_detected' if deviated else 'within_tolerance',
                      next_action='request_placement_review' if deviated else 'continue_after_review')
        if self.last_seen is not None and frame-self.last_seen>30:
            report.update(status='stale_observation',next_action='reobserve_element')
        return report
