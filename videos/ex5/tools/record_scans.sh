#!/usr/bin/env bash
# Record /ouster/scan (made by pointcloud_to_laserscan with the launch file's parameters)
# plus /odom and /tf_static while dlg_feldtage_24:cut plays once, then export them to ../data/scans.json.
# Run inside Ubuntu (WSL) from anywhere; needs ROS 2 Jazzy, marina and a workspace with
# pointcloud_to_laserscan built (WS, default ~/ex5_ws).
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=${WS:-$HOME/ex5_ws}
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=46
OUT=$HOME/ex5_scans; rm -rf $OUT
BAG=$(marina resolve dlg_feldtage_24:cut)
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=pcl_2_lsc \
  -r cloud_in:=/ouster/points -r scan_in:=/ouster/points -r scan:=/ouster/scan \
  -p target_frame:=os_lidar -p transform_tolerance:=0.01 -p min_height:=2.0 -p max_height:=6.0 \
  -p angle_min:=-3.141592653589793 -p angle_max:=3.141592653589793 -p angle_increment:=0.0087 \
  -p scan_time:=0.3333 -p range_min:=0.45 -p range_max:=50.0 -p use_inf:=true -p inf_epsilon:=1.0 \
  > /tmp/p2l_rec.log 2>&1 &
sleep 2
timeout --signal=INT 52 ros2 bag record -o $OUT -s mcap /ouster/scan /odom /tf_static > $HOME/rec.log 2>&1 &
REC=$!
sleep 3
ros2 bag play "$BAG" --remap /tf:=/tf_old > /tmp/play.log 2>&1
sleep 2
wait $REC; sleep 1
pkill -f pointcloud_to_laserscan_node
ros2 bag info $OUT | grep -E "Duration|Topic"
python3 "$HERE/export_scans.py" "$OUT" "$HERE/../data/scans.json" 2>&1 | grep -v "^\[INFO"
ls -la "$HERE/../data"
