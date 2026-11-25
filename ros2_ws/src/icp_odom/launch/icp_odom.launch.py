#!/usr/bin/env python3
from better_launch import BetterLaunch, launch_this
import math

@launch_this(ui=True)
def pointcloud_2d():
    bl = BetterLaunch()

    # --- User parameters ---
    use_icp: bool = True  # HIER ÄNDERN
    icp_only: bool = False  # HIER ÄNDERN
    icp_gain: float = 2.0
    # ---        

    with bl.group("icp_odom"):
        bl.node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pcl_2_lsc",
            params={
                'target_frame': 'os_lidar',
                'transform_tolerance': 0.01,
                'min_height': 2.0,
                'max_height': 6.0,
                'angle_min': -math.pi,
                'angle_max': math.pi,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            },
            remaps={
                "scan_in": "/ouster/points",
                "scan": "/ouster/scan",
            }
        )
        bl.node(
            package="icp_odom",
            executable="icp_node",
            name="icp_kalman_node",
            params={
                'use_icp': use_icp,
                'icp_only': icp_only,
                'icp_gain': icp_gain,
            },
        )
        
