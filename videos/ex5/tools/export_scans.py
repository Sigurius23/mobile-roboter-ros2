"""Usage (Ubuntu, ROS sourced):  python3 export_scans.py <recorded bag dir> <out.json>

Export recorded /ouster/scan (in base_footprint), /odom poses and the lidar mount to JSON."""
import json
import math
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

uri = sys.argv[1]
out = sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
scans, odom, static = [], [], {}
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == "/ouster/scan":
        scans.append(deserialize_message(data, LaserScan))
    elif topic == "/odom":
        odom.append(deserialize_message(data, Odometry))
    elif topic == "/tf_static":
        for t in deserialize_message(data, TFMessage).transforms:
            static[t.child_frame_id] = t
print(f"scans {len(scans)}, odom {len(odom)}, static frames {len(static)}")


def T(st):
    return st.sec + st.nanosec * 1e-9


def qmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_base(frame):
    R, p, f = np.eye(3), np.zeros(3), frame
    while f in static and f != "base_footprint":
        tr = static[f].transform
        Rf = qmat(tr.rotation)
        tf = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        R, p = Rf @ R, Rf @ p + tf
        f = static[f].header.frame_id
    assert f == "base_footprint", f
    return R, p


lidar_frame = scans[0].header.frame_id
R, p = to_base(lidar_frame)
yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
print(f"lidar frame {lidar_frame}: offset {np.round(p, 3)}, yaw {yaw:.1f} deg")

t0 = T(odom[0].header.stamp)
res = dict(lidar_frame=lidar_frame, lidar_offset=p.round(4).tolist(), lidar_yaw_deg=round(yaw, 2), scans=[], odom=[])
for s in scans:
    a = s.angle_min + s.angle_increment * np.arange(len(s.ranges))
    rr = np.array(s.ranges, dtype=float)
    ok = np.isfinite(rr) & (rr >= s.range_min) & (rr <= s.range_max)
    pl = np.stack([rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok]), np.zeros(ok.sum())], 1)
    pb = pl @ R.T + p
    res["scans"].append(dict(t=round(T(s.header.stamp) - t0, 4), xy=np.round(pb[:, :2], 3).tolist()))
for o in odom:
    q = o.pose.pose.orientation
    res["odom"].append([round(T(o.header.stamp) - t0, 4), round(o.pose.pose.position.x, 4), round(o.pose.pose.position.y, 4),
                        round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)), 5),
                        round(o.twist.twist.linear.x, 4), round(o.twist.twist.angular.z, 4)])
n = [len(s["xy"]) for s in res["scans"]]
print(f"points per scan: min {min(n)}, median {int(np.median(n))}, max {max(n)}; scan times {res['scans'][0]['t']:.2f} .. {res['scans'][-1]['t']:.2f}")
json.dump(res, open(out, "w"))
