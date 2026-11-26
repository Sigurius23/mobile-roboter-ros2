#!/usr/bin/env bash
# Play dlg_feldtage_24:cut through the ICP node (filter + ICP) and show it in RViz, for the screen
# recording in part 5 (capture_rviz.py grabs the window from Windows). Run inside Ubuntu (WSL).
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=${WS:-$HOME/ex5_ws}
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=46
BAG=$(marina resolve dlg_feldtage_24:cut)
LOG=/tmp/rviz_run; mkdir -p $LOG
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=pcl_2_lsc \
  -r cloud_in:=/ouster/points -r scan_in:=/ouster/points -r scan:=/ouster/scan \
  -p target_frame:=os_lidar -p transform_tolerance:=0.01 -p min_height:=2.0 -p max_height:=6.0 \
  -p angle_min:=-3.141592653589793 -p angle_max:=3.141592653589793 -p angle_increment:=0.0087 \
  -p scan_time:=0.3333 -p range_min:=0.45 -p range_max:=50.0 -p use_inf:=true -p inf_epsilon:=1.0 \
  > $LOG/p2l.log 2>&1 &
ros2 run icp_odom icp_node --ros-args -p use_icp:=true -p icp_only:=false -p icp_gain:=2.0 > $LOG/icp.log 2>&1 &
python3 "$HERE/path_pub.py" --ros-args -p use_sim_time:=true > $LOG/path.log 2>&1 &
rviz2 -d "$HERE/ex5.rviz" --ros-args -p use_sim_time:=true > $LOG/rviz.log 2>&1 &
RVIZ=$!
sleep ${WAIT:-12}
echo "PLAY $(date +%s.%N)" > $LOG/marks
ros2 bag play "$BAG" --clock --remap /tf:=/tf_old > $LOG/play.log 2>&1
echo "END $(date +%s.%N)" >> $LOG/marks
sleep ${HOLD:-4}
kill $RVIZ
pkill -f icp_node; pkill -f pointcloud_to_laserscan_node; pkill -f path_pub.py
sleep 1
tail -3 $LOG/icp.log; tail -3 $LOG/rviz.log
