"""Small SE(2) scan-to-map front end; no ground truth enters this module."""
import numpy as np
from scipy.spatial import cKDTree


def transform(points, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def compose(a, b):
    xy = transform(np.asarray([b[:2]]), a)[0]
    return np.array([*xy, np.arctan2(np.sin(a[2]+b[2]), np.cos(a[2]+b[2]))])


def relative(a, b):
    c, s = np.cos(a[2]), np.sin(a[2])
    xy = np.array([[c,s],[-s,c]]) @ (b[:2]-a[:2])
    return np.array([*xy, np.arctan2(np.sin(b[2]-a[2]), np.cos(b[2]-a[2]))])


def icp(scan, reference, initial, gate=.45, iterations=25):
    pose = np.array(initial, dtype=float)
    tree = cKDTree(reference)
    for _ in range(iterations):
        p = transform(scan, pose)
        distances, ids = tree.query(p)
        valid = distances < gate
        if valid.sum() < 20:
            return np.array(initial), False
        # Trim worst correspondences to reduce influence of newly placed objects.
        valid &= distances <= np.quantile(distances[valid], .85)
        a, b = p[valid], reference[ids[valid]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a-ca).T @ (b-cb))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        shift = cb - rotation @ ca
        delta = np.array([*shift, np.arctan2(rotation[1,0], rotation[0,0])])
        pose = compose(delta, pose)
        if np.linalg.norm(delta) < 1e-5:
            break
    d, _ = tree.query(transform(scan, pose))
    return pose, bool((d < gate).mean() > .55)


class Mapper:
    def __init__(self):
        self.pose = np.zeros(3)
        self.points = np.empty((0, 2))
        self.accepted = True

    def update(self, scan, motion):
        prediction = compose(self.pose, motion)
        if len(self.points):
            self.pose, self.accepted = icp(scan, self.points, prediction)
        else:
            self.pose, self.accepted = prediction, True
        if self.accepted:
            merged = np.vstack([self.points, transform(scan, self.pose)])
            _, idx = np.unique(np.floor(merged/.06).astype(int), axis=0, return_index=True)
            self.points = merged[idx]
        return self.pose.copy()


class ChangeDetector:
    """Persistent novel occupied endpoints relative to a frozen observed baseline.

    Unobserved surfaces can be false positives; this is NOT semantic detection.
    """
    def __init__(self, baseline, threshold=.3, cell=.2, persistence=3):
        self.tree = cKDTree(baseline)
        self.threshold, self.cell, self.persistence = threshold, cell, persistence
        self.counts = {}

    def update(self, world_points):
        d, _ = self.tree.query(world_points)
        cells = set(map(tuple, np.floor(world_points[d > self.threshold]/self.cell).astype(int)))
        self.counts = {k: self.counts.get(k, 0)+1 for k in cells}
        return np.array([(np.array(k)+.5)*self.cell for k,v in self.counts.items()
                         if v >= self.persistence]).reshape(-1,2)
