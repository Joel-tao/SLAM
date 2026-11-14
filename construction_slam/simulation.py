"""Ray-cast planar lidar simulator. Truth is used only here and for scoring."""
import numpy as np


def rectangle(x0,y0,x1,y1):
    p = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    return [(p[i],p[(i+1)%4]) for i in range(4)]


STATIC = rectangle(-6,-5,6,5)+rectangle(-1,-.7,1,.7)+rectangle(-4,2,-3,3)+rectangle(3,-3,4,-2)
# Deliberate installation error: +12 cm x, -8 cm y, +8 degrees yaw.
# This truth is never passed to the inspector.
from .core import transform
ACTUAL_ELEMENT_POSE = np.array([2.62, 1.77, np.deg2rad(8)])
_corners = transform(np.array([[-.5,-.35],[.5,-.35],[.5,.35],[-.5,.35]]), ACTUAL_ELEMENT_POSE)
ADDITION = [(_corners[i], _corners[(i+1)%4]) for i in range(4)]


def cross(a,b):
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


def scan(pose, segments, rng, beams=360):
    angles = np.linspace(-np.pi,np.pi,beams,endpoint=False)
    rays = np.stack([np.cos(angles+pose[2]),np.sin(angles+pose[2])],axis=1)
    seg = np.array(segments)
    v = seg[:,1]-seg[:,0]
    offset = seg[:,0]-pose[:2]
    denominator = cross(rays[:,None,:],v[None,:,:])
    safe = np.where(abs(denominator)>1e-10,denominator,np.nan)
    t = cross(offset,v)[None,:]/safe
    u = cross(offset[None,:,:],rays[:,None,:])/safe
    hits = np.where((t>0)&(u>=0)&(u<=1),t,np.inf)
    ranges = hits.min(axis=1)
    valid = np.isfinite(ranges)&(ranges<15)
    ranges = ranges[valid]+rng.normal(0,.012,valid.sum())
    return np.stack([ranges*np.cos(angles[valid]),ranges*np.sin(angles[valid])],axis=1)


def trajectory(steps=240):
    t = np.linspace(0,4*np.pi,steps)
    return np.column_stack([4.6*np.cos(t),3.8*np.sin(t),
                            np.unwrap(np.arctan2(3.8*np.cos(t),-4.6*np.sin(t)))])
