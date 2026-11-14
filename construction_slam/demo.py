import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from .core import Mapper, ChangeDetector, relative, compose, transform
from .simulation import STATIC, ADDITION, scan, trajectory
from .inspection import ElementInspector

def outline(pose, size):
    x,y=np.asarray(size)/2
    return transform(np.array([[-x,-y],[x,-y],[x,y],[-x,y],[-x,-y]]),pose)


def run(seed=7, output='artifacts', gif=False):
    out = Path(output)
    out.mkdir(parents=True,exist_ok=True)
    rng = np.random.default_rng(seed)
    truth = trajectory()
    # All estimates use the first lidar frame as origin, never simulator world pose.
    gt = np.array([relative(truth[0],p) for p in truth])
    mapper = Mapper()
    odom = np.zeros(3)
    estimates, odometry, frames, changes = [], [], [], []
    detector = None
    inspector = None
    reports = []
    design = json.loads((Path(__file__).resolve().parent.parent/"config/design.json").read_text())
    insertion = len(truth)//2
    failures = 0
    for i,p in enumerate(truth):
        lidar = scan(p,STATIC+(ADDITION if i>=insertion else []),rng)
        motion = np.zeros(3) if i==0 else relative(truth[i-1],p)
        if i:
            motion[:2] *= 1.035
            motion += rng.normal(0,[.003,.003,.002]) + [0,0,.001]
        odom = compose(odom,motion)
        if i==insertion:
            detector = ChangeDetector(mapper.points.copy())
            inspector = ElementInspector(design,mapper.points.copy())
        estimate = mapper.update(lidar,motion)
        failures += int(not mapper.accepted)
        novelty = detector.update(transform(lidar,estimate)) if detector and mapper.accepted else np.empty((0,2))
        report = inspector.update(transform(lidar,estimate),i) if inspector and mapper.accepted else (inspector.report(i) if inspector else None)
        if report is not None:
            reports.append(report)
        estimates.append(estimate)
        odometry.append(odom.copy())
        changes.append(len(novelty))
        if i%4==0 or i==len(truth)-1:
            frames.append((i,mapper.points.copy(),novelty.copy(),report))
    estimates, odometry = np.array(estimates), np.array(odometry)
    errors = np.linalg.norm(estimates[:,:2]-gt[:,:2],axis=1)
    odom_errors = np.linalg.norm(odometry[:,:2]-gt[:,:2],axis=1)
    metrics = dict(seed=seed,frames=len(truth),insertion_frame=insertion,
                   scan_to_map_position_rmse_m=float(np.sqrt(np.mean(errors**2))),
                   odometry_position_rmse_m=float(np.sqrt(np.mean(odom_errors**2))),
                   rejected_scans=failures,peak_persistent_novel_cells=max(changes),
                   scoring='Position RMSE in shared initial frame; no post-hoc alignment')
    (out/'construction_state.json').write_text(json.dumps(reports[-1],indent=2)+'\n')
    (out/'inspection_history.json').write_text(json.dumps(reports,indent=2)+'\n')
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    np.savetxt(out/'trajectory.csv',np.column_stack([gt,odometry,estimates,errors]),delimiter=',',
               header='gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,estimate_x,estimate_y,estimate_yaw,position_error',comments='')
    np.save(out/'map_points.npy',mapper.points)
    fig, (ax,err) = plt.subplots(1,2,figsize=(13,6),gridspec_kw={'width_ratios':[1.4,1]})
    def draw(frame):
        i,points,novel,report = frame
        ax.clear(); err.clear()
        ax.scatter(points[:,0],points[:,1],s=2,c='#94a3b8',label='Observed map')
        ax.plot(gt[:i+1,0],gt[:i+1,1],c='#16a34a',label='Ground truth (evaluation only)')
        ax.plot(odometry[:i+1,0],odometry[:i+1,1],c='#f59e0b',label='Noisy odometry')
        ax.plot(estimates[:i+1,0],estimates[:i+1,1],c='#2563eb',label='Scan-to-map estimate')
        ax.scatter(*estimates[i,:2],s=70,c='#2563eb')
        if len(novel): ax.scatter(novel[:,0],novel[:,1],s=18,c='#e11d48',label='Persistent novel surfaces')
        planned=outline(design['design_pose'],design['size_m'])
        ax.plot(planned[:,0],planned[:,1],'--',c='#7c3aed',lw=2,label='Design footprint')
        if report and report['estimated_pose'] is not None:
            fitted=outline(report['estimated_pose'],design['size_m'])
            ax.plot(fitted[:,0],fitted[:,1],c='#e11d48',lw=2,label='Fitted element')
            ax.text(.02,.02,f"Placement offset: {report['translation_deviation_m']*100:.1f} cm / {report['yaw_deviation_deg']:.1f} deg",transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',alpha=.9))
        ax.set(xlim=(-6,6),ylim=(-2,12),xlabel='x [m]',ylabel='y [m]',aspect='equal',
               title='Construction added' if i>=insertion else 'Baseline mapping')
        ax.legend(fontsize=7,loc='upper right')
        err.plot(odom_errors[:i+1],c='#f59e0b',label='Odometry')
        err.plot(errors[:i+1],c='#2563eb',label='Scan-to-map')
        err.axvline(insertion,color='#e11d48',ls='--',label='Structure inserted')
        err.set(xlim=(0,len(truth)),ylim=(0,max(odom_errors.max(),errors.max())*1.1+.01),
                xlabel='Frame',ylabel='Position error [m]',title='Measured localization error')
        err.grid(alpha=.2); err.legend(fontsize=8)
        fig.suptitle('BUILD / MAP — 2D LiDAR mapping & construction change detection',fontsize=14)
        fig.tight_layout()
    draw(frames[-1]); fig.savefig(out/'overview.png',dpi=150)
    if gif:
        FuncAnimation(fig,draw,frames=frames,interval=100).save(out/'demo.gif',writer=PillowWriter(fps=10))
    plt.close(fig)
    print(json.dumps(metrics,indent=2))
    return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--output',default='artifacts')
    p.add_argument('--gif',action='store_true')
    a=p.parse_args(); run(a.seed,a.output,a.gif)
