#!/usr/bin/env python3
"""Per-second VIO-vs-mocap analysis of one replay (or the live estimate) of a flight bag (mocap glitches filtered).
   flight_analysis.py --bag B --traj T [--viz V] [--prior x y z (m)] [--camchain yaml] [--fit a b] [--yaw-from a b] [--events t1 t2 ...] [--rows]
 --fit a b   : bag-time window used for the 4-DoF (yaw+xyz) alignment and the constant mount rotation
               (default: first 10 s after init). Errors are reported with THIS alignment.
 --events    : bag times that delimit segments to summarise (e.g. takeoff, maneuver start/end, landing).
 --viz       : bag with /ov_msckf/loop_extrinsic (p_CinI) and /ov_msckf/points_msckf|points_slam recorded during the replay.
"""
import sys, math, argparse, numpy as np, rosbag
def R_of(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def ang(R): return math.degrees(math.acos(max(-1,min(1,(np.trace(R)-1)/2))))
def mean_rot(Rs):
    U,_,Vt=np.linalg.svd(sum(Rs)); R=U@Vt
    if np.linalg.det(R)<0: U[:,-1]*=-1; R=U@Vt
    return R
ap=argparse.ArgumentParser(); ap.add_argument('--bag',required=True); ap.add_argument('--traj',required=True); ap.add_argument('--viz')
ap.add_argument('--prior',type=float,nargs=3); ap.add_argument('--camchain'); ap.add_argument('--fit',type=float,nargs=2); ap.add_argument('--events',type=float,nargs='*',default=[])
ap.add_argument('--yaw-from',type=float,nargs=2,help='bag-time window for the yaw fit (needs horizontal motion); translation still from --fit')
ap.add_argument('--rows',action='store_true',help='print every 1 s row (default: every 5 s plus event neighbourhoods)'); ap.add_argument('--mocap',default='/natnet_ros/kingfisher/pose')
A=ap.parse_args()
bag=rosbag.Bag(A.bag); t0=bag.get_start_time(); Tm=[];Pm=[];Rm=[]
for _,m,_ in bag.read_messages(topics=[A.mocap]):
    p=m.pose.position; q=m.pose.orientation; Tm.append(m.header.stamp.to_sec()-t0); Pm.append([p.x,p.y,p.z]); Rm.append(R_of([q.x,q.y,q.z,q.w]))
bag.close(); Tm=np.array(Tm);Pm=np.array(Pm);Rm=np.array(Rm)
# mocap glitch filter: drop samples whose speed (vs the previous sample) exceeds 6 m/s, plus 0.1 s on each side
_sp=np.r_[0,np.linalg.norm(np.diff(Pm,axis=0),axis=1)/np.maximum(np.diff(Tm),1e-3)]
_bad=_sp>6.0
if _bad.any():
    _tb=Tm[_bad]; _mask=np.zeros(len(Tm),bool)
    for tb_ in _tb: _mask|=np.abs(Tm-tb_)<=0.1
    print("mocap glitches: %d samples with > 6 m/s jumps (max %.0f m/s) at t = %s ...; %d samples (%.1f s) removed"%(_bad.sum(),_sp.max(),", ".join("%.1f"%x for x in _tb[:8]),_mask.sum(),_mask.sum()/200.0))
    Tm,Pm,Rm=Tm[~_mask],Pm[~_mask],Rm[~_mask]
d=np.loadtxt(A.traj,comments='#'); T=d[:,0]-t0; P=d[:,1:4]; Rv=np.array([R_of(q) for q in d[:,4:8]])
ok=(T>=Tm[0])&(T<=Tm[-1]); T,P,Rv=T[ok],P[ok],Rv[ok]
Pmi=np.stack([np.interp(T,Tm,Pm[:,c]) for c in range(3)],1); j=np.clip(np.searchsorted(Tm,T),0,len(Tm)-1); Rmi=Rm[j]
fa,fb=(A.fit if A.fit else (T[0],T[0]+10)); s=(T>=fa)&(T<fb)
if s.sum()<20: print("fit window has %d samples; aborting"%s.sum()); sys.exit(1)
# 4-DoF alignment on the fit window: yaw by centred least squares, then translation
cv,cm=P[s].mean(0),Pmi[s].mean(0); a=P[s]-cv; b=Pmi[s]-cm
yaw=math.atan2((a[:,0]*b[:,1]-a[:,1]*b[:,0]).sum(),(a[:,0]*b[:,0]+a[:,1]*b[:,1]).sum())
if A.yaw_from:
    sy=(T>=A.yaw_from[0])&(T<A.yaw_from[1]); ay=P[sy]-P[sy].mean(0); by=Pmi[sy]-Pmi[sy].mean(0)
    yaw=math.atan2((ay[:,0]*by[:,1]-ay[:,1]*by[:,0]).sum(),(ay[:,0]*by[:,0]+ay[:,1]*by[:,1]).sum())
R=Rz(yaw); t=cm-R@cv
E=np.linalg.norm((P@R.T+t)-Pmi,axis=1)
# constant mount rotation body(mocap)->imu on the fit window, then the orientation residual
Rbi=mean_rot([Rmi[i].T@R@Rv[i] for i in np.where(s)[0]])
O=np.array([ang(R@Rv[i]@(Rmi[i]@Rbi).T) for i in range(len(T))])
# mocap kinematics for context
dt=np.gradient(Tm); Vm=np.gradient(Pm,axis=0)/dt[:,None]; spm=np.linalg.norm(Vm,axis=1)
roll=np.degrees(np.array([math.atan2(Rr[2,1],Rr[2,2]) for Rr in Rm])); z0=np.median(Pm[Tm<2,2]) if (Tm<2).sum() else Pm[0,2]
# VIO speed from the trajectory (numerical)
Vv=np.gradient(P,axis=0)/np.gradient(T)[:,None]; spv=np.linalg.norm(Vv,axis=1)
print("init at bag t = %.1f s; %d poses over %.1f s; alignment fitted on %.1f-%.1f s (%d samples)"%(T[0],len(T),T[-1]-T[0],fa,fb,s.sum()))
print("mount rotation body->imu from the fit window: %.1f deg; position RMS over the whole overlap with this alignment: %.2f m, max %.2f m"%(ang(Rbi),math.sqrt((E**2).mean()),E.max()))
# extrinsics / features
ext=None; feats={}
if A.viz:
    vb=rosbag.Bag(A.viz); ex=[]; odw=[]; rawf=[]
    for tp,m,tr in vb.read_messages(topics=['/ov_msckf/loop_extrinsic','/ov_msckf/points_msckf','/ov_msckf/points_slam','/ov_msckf/odomimu']):
        ts=m.header.stamp.to_sec()-t0
        if tp.endswith('loop_extrinsic'):
            p=m.pose.pose.position; q=m.pose.pose.orientation; ex.append([ts,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        elif tp.endswith('odomimu'): odw.append((tr.to_sec(),ts))
        else: rawf.append((tr.to_sec(),tp.split('/')[-1],m.width*m.height))   # point clouds are stamped with wall time: map via the odom record times
    vb.close(); ext=np.array(ex) if ex else None
    if odw and rawf:
        odw=np.array(odw); tw=np.array([r[0] for r in rawf]); tb=np.interp(tw,odw[:,0],odw[:,1])
        for (_,kind,n),t_ in zip(rawf,tb): feats.setdefault(kind,[]).append((t_,n))
    if ext is not None:
        pr=np.array(A.prior) if A.prior else ext[0,1:4]
        Rpr=None
        if A.camchain:
            import re
            rows=[l for l in open(A.camchain) if re.match(r'\s*- \[',l)]
            M=np.array([[float(x) for x in re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?',r)] for r in rows[:4]]); Rpr=M[:3,:3]  # R_CfromI
        print("online extrinsic p_CinI [mm] (prior %s):"%np.round(pr*1e3,1).tolist())
        for lab,tt in [("init",ext[0,0]),("+5 s",ext[0,0]+5),("+10 s",ext[0,0]+10),("+20 s",ext[0,0]+20),("+30 s",ext[0,0]+30),("+60 s",ext[0,0]+60),("end",ext[-1,0])]:
            k=np.argmin(np.abs(ext[:,0]-tt));
            if abs(ext[k,0]-tt)>2: continue
            pc=ext[k,1:4]; rot=""
            if Rpr is not None:
                x,y,z,w=ext[k,4:8]; Rq=R_of([x,y,z,w])   # JPL q_ItoC published xyzw == Hamilton R_CtoI? use chordal distance to the prior either way
                rot=" rot vs prior %.2f deg"%min(ang(Rq@Rpr.T),ang(Rq.T@Rpr.T))
            print("  %-6s t=%6.1f  [%6.1f %6.1f %6.1f]  |d prior| %5.1f mm%s"%(lab,ext[k,0],*(pc*1e3),np.linalg.norm(pc-pr)*1e3,rot))
        w=ext[:,0]>ext[-1,0]-20; print("  mean of the last 20 s: [%6.1f %6.1f %6.1f] mm, std [%4.1f %4.1f %4.1f] mm"%(*(ext[w,1:4].mean(0)*1e3),*(ext[w,1:4].std(0)*1e3)))
# segment summaries
ev=sorted(set([T[0]]+[e for e in A.events if T[0]<e<T[-1]]+[T[-1]]))
print("segments (alignment from the fit window): start-end | mean / max pos err [m] | mean orient err [deg] | mocap max speed, max |roll| | features msckf / slam per update")
for a_,b_ in zip(ev[:-1],ev[1:]):
    w=(T>=a_)&(T<b_); wm=(Tm>=a_)&(Tm<b_)
    fm=[n for (ts,n) in feats.get('points_msckf',[]) if a_<=ts<b_]; fs=[n for (ts,n) in feats.get('points_slam',[]) if a_<=ts<b_]
    print("  %5.1f-%5.1f | %6.2f / %6.2f | %5.1f | %4.1f m/s %4.0f deg | %s / %s"%(a_,b_,E[w].mean(),E[w].max(),O[w].mean(),np.percentile(spm[wm],99) if wm.any() else float('nan'),np.abs(roll[wm]).max() if wm.any() else float('nan'),("%.0f"%np.mean(fm)) if fm else "-",("%.0f"%np.mean(fs)) if fs else "-"))
# per-second rows
print("rows: t | mocap z, speed, |roll| | VIO speed | pos err | orient err | feats msckf/slam (mean per update) | zero-feature updates | p_CinI mm")
n0,n1=int(T[0]),int(T[-1])+1
for i in range(n0,n1):
    near=any(abs(i-e)<=6 for e in A.events)
    if not (A.rows or i%5==0 or near): continue
    w=(T>=i)&(T<i+1); wm=(Tm>=i)&(Tm<i+1)
    if not w.any(): continue
    fm=[n for (ts,n) in feats.get('points_msckf',[]) if i<=ts<i+1]; fs=[n for (ts,n) in feats.get('points_slam',[]) if i<=ts<i+1]
    z=[n for n in fm if n==0]
    ex_s=""
    if ext is not None:
        k=np.argmin(np.abs(ext[:,0]-(i+0.5)))
        if abs(ext[k,0]-(i+0.5))<1: ex_s="[%5.1f %5.1f %5.1f]"%tuple(ext[k,1:4]*1e3)
    print("  %4d | %5.2f %4.2f %4.0f | %5.2f | %7.2f | %5.1f | %s/%s | %s | %s"%(i,Pm[wm,2].mean()-z0 if wm.any() else float('nan'),spm[wm].max() if wm.any() else float('nan'),np.abs(roll[wm]).max() if wm.any() else float('nan'),spv[w].max(),E[w].mean(),O[w].mean(),("%3.0f"%np.mean(fm)) if fm else "  -",("%3.0f"%np.mean(fs)) if fs else "  -",("%d"%len(z)) if fm else "-",ex_s))
