"""Publish the node's trajectory (TF odom -> base_footprint) as nav_msgs/Path on /icp_path, for RViz."""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class PathPub(Node):
    def __init__(self):
        super().__init__("icp_path_pub")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(Path, "/icp_path", 10)
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.last_stamp = None
        self.create_timer(0.1, self.tick)

    def tick(self):
        try:
            t = self.buf.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception:
            return
        stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        if self.last_stamp is not None and stamp < self.last_stamp:  # bag looped: start a new path
            self.path.poses.clear()
        self.last_stamp = stamp
        p = PoseStamped()
        p.header = t.header
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.orientation = t.transform.rotation
        self.path.poses.append(p)
        self.path.header.stamp = t.header.stamp
        self.pub.publish(self.path)


def main():
    rclpy.init()
    rclpy.spin(PathPub())


if __name__ == "__main__":
    main()
