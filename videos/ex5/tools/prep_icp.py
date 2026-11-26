"""ICP traces on real scans of dlg_feldtage_24:cut for the Exercise 5 videos.

Usage (Windows or Linux, numpy + scipy):  python prep_icp.py ../data/scans.json ../data/icp_demo.json

Same algorithm as icp_node.cpp (point-to-point, nearest neighbour < 1 m, SVD, 20 iterations,
stop below 1e-4 m and 1e-4 rad), written with numpy + scipy.
"""
import json
import math
import sys
import numpy as np
from scipy.spatial import cKDTree

data = json.load(open(sys.argv[1]))
out = sys.argv[2]
scans = [np.array(s["xy"]) for s in data["scans"]]
ts = np.array([s["t"] for s in data["scans"]])
odom = np.array(data["odom"])  # t, x, y, yaw, v, w


def T2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def xyth(T):
    return T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])


def apply(T, P):
    return P @ T[:2, :2].T + T[:2, 2]


def icp(src, tgt, guess_robot=(0, 0, 0), trace=False):
    T_total = np.linalg.inv(T2(*guess_robot))
    cur = apply(T_total, src)
    tree = cKDTree(tgt)
    steps = []
    for it in range(20):
        d, idx = tree.query(cur)
        keep = d ** 2 < 1.0
        if keep.sum() < 3:
            break
        s, t = cur[keep], tgt[idx[keep]]
        if trace:
            steps.append(dict(points=np.round(cur, 3).tolist(), pairs_src=np.round(s, 3).tolist(),
                              pairs_tgt=np.round(t, 3).tolist(), src_mean=s.mean(0).round(3).tolist(),
                              tgt_mean=t.mean(0).round(3).tolist()))
        sm, tm = s.mean(0), t.mean(0)
        H = (s - sm).T @ (t - tm)
        U, _, Vt = np.linalg.svd(H)
        V = Vt.T
        R = V @ U.T
        if np.linalg.det(R) < 0:
            V[:, 1] *= -1
            R = V @ U.T
        tr = tm - R @ sm
        cur = cur @ R.T + tr
        Ts = np.eye(3); Ts[:2, :2] = R; Ts[:2, 2] = tr
        T_total = Ts @ T_total
        if np.linalg.norm(tr) < 1e-4 and abs(math.atan2(R[1, 0], R[0, 0])) < 1e-4:
            break
    d, idx = tree.query(cur)
    keep = d ** 2 < 1.0
    rms = float(np.sqrt((d[keep] ** 2).mean())) if keep.any() else 0.0
    radius = float(np.linalg.norm(tgt[idx[keep]], axis=1).mean()) if keep.any() else 0.0
    res = dict(delta=xyth(np.linalg.inv(T_total)), inliers=int(keep.sum()), rms=rms, radius=radius, iterations=len(steps) if trace else None)
    if trace:
        steps.append(dict(points=np.round(cur, 3).tolist()))
        res["steps"] = steps
    return res


def odom_at(t):
    i = int(np.clip(np.searchsorted(odom[:, 0], t), 0, len(odom) - 1))
    return odom[i]


def true_delta(t0, t1):
    a, b = odom_at(t0), odom_at(t1)
    return xyth(np.linalg.inv(T2(*a[1:4])) @ T2(*b[1:4]))


result = dict(lidar_offset=data["lidar_offset"], lidar_yaw_deg=data["lidar_yaw_deg"], pairs={})

# pick a straight pair (fast, small omega) and a turning pair (big omega), 0.5 s apart
v, w = odom[:, 4], odom[:, 5]
k_gap = 10  # 10 scans at 20 Hz = 0.5 s
straight_t = odom[np.argmax((np.abs(w) < 0.02) & (v > 1.0) & (odom[:, 0] > 22))][0]
turn_t = odom[np.argmax(np.abs(w))][0] - 0.25
for name, t_pick in (("straight", straight_t), ("turn", turn_t)):
    k = int(np.argmin(np.abs(ts - t_pick)))
    src, tgt = scans[k], scans[k + k_gap]
    r = icp(src, tgt, trace=True)
    truth = true_delta(ts[k], ts[k + k_gap])
    raw = icp(src, tgt, trace=False)
    # the same pair in the lidar's own frame (no getTransform): undo the mount
    yaw = math.radians(data["lidar_yaw_deg"]); off = np.array(data["lidar_offset"][:2])
    Tbl = T2(off[0], off[1], yaw)
    no_tf = icp(apply(np.linalg.inv(Tbl), src), apply(np.linalg.inv(Tbl), tgt))
    result["pairs"][name] = dict(t=[ts[k], ts[k + k_gap]], src=np.round(src, 3).tolist(), tgt=np.round(tgt, 3).tolist(),
                                 steps=r["steps"], delta=r["delta"], truth=truth, inliers=r["inliers"], rms=r["rms"],
                                 radius=r["radius"], delta_no_tf=no_tf["delta"])
    print(f"{name}: t={ts[k]:.2f}-{ts[k+k_gap]:.2f}  ICP {np.round(r['delta'], 3)}  wheels {np.round(truth, 3)}  "
          f"iterations {len(r['steps']) - 1}  N={r['inliers']} rms={r['rms']:.3f} r={r['radius']:.1f}  without TF {np.round(no_tf['delta'], 3)}")
    # TASK 2: the same pair, started at the filter's guess (v*dt, 0, w*dt)
    o = odom_at(ts[k]); dt = ts[k + k_gap] - ts[k]
    guess = (o[4] * dt, 0.0, o[5] * dt)
    g = icp(src, tgt, guess_robot=guess, trace=True)
    result["pairs"][name].update(guess=guess, guess_steps=g["steps"], guess_delta=g["delta"], guess_inliers=g["inliers"],
                                 guess_rms=g["rms"])
    print(f"   with guess {np.round(guess, 3)}: ICP {np.round(g['delta'], 3)}  iterations {len(g['steps']) - 1}  N={g['inliers']} rms={g['rms']:.3f}")

# pure-ICP path over the whole bag (zero guess, like icp_only), and the wheels' path
pose = np.eye(3)
path = [(ts[0], 0.0, 0.0, 0.0)]
N, rms = [], []
for k in range(1, len(scans)):
    r = icp(scans[k - 1], scans[k])
    pose = pose @ T2(*r["delta"])
    path.append((ts[k], *xyth(pose)))
    N.append(r["inliers"]); rms.append(r["rms"])
o0 = T2(*odom[0, 1:4])
wheels = [(o[0], *xyth(np.linalg.inv(o0) @ T2(*o[1:4]))) for o in odom[::5]]
dist_icp = float(np.sum(np.hypot(*np.diff(np.array(path)[:, 1:3], axis=0).T)))
dist_wheels = float(np.sum(np.hypot(*np.diff(odom[:, 1:3], axis=0).T)))
print(f"pure ICP end {np.round(path[-1][1:], 2)} path {dist_icp:.1f} m | wheels end {np.round(wheels[-1][1:], 2)} path {dist_wheels:.1f} m")
print(f"median N {np.median(N):.0f}, median rms {np.median(rms):.3f}")
result["path_icp"] = np.round(np.array(path), 3).tolist()
result["path_wheels"] = np.round(np.array(wheels), 3).tolist()
result["dist"] = dict(icp=dist_icp, wheels=dist_wheels)
json.dump(result, open(out, "w"))
