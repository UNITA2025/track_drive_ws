#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from interfaces_pkg.msg import ErpStatusMsg1, ConeInfoArray
from scipy.interpolate import CubicSpline
import numpy as np

class WaypointExtractor(Node):
    def __init__(self):
        super().__init__('waypoint_extractor')
        self.centroid_threshold = 5.0
        self.frame_id = "velodyne"
        self.left_color = "yellow"
        self.right_color = "blue"
        self.steering_angle_deg = 0.0
        self.left_cones = []
        self.right_cones = []

        self.left_centroid_sub = self.create_subscription(ConeInfoArray, '/cones/cone_info_left', self.left_centroid_callback, 10)
        self.right_centroid_sub = self.create_subscription(ConeInfoArray, '/cones/cone_info_right', self.right_centroid_callback, 10)
        self.steering_angle_sub = self.create_subscription(ErpStatusMsg1, '/erp42_status', self.steering_angle_callback, 10)

        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

    def steering_angle_callback(self, msg: ErpStatusMsg1):
        self.steering_angle_deg = (msg.steer / 2000.0) * 30.0

    def left_centroid_callback(self, msg: ConeInfoArray):
        self.left_cones = [Point(x=c.x, y=c.y, z=0.0) for c in msg.cones
                          if math.sqrt(c.x**2 + c.y**2) <= self.centroid_threshold]
        self.compute_and_publish_path()

    def right_centroid_callback(self, msg: ConeInfoArray):
        self.right_cones = [Point(x=c.x, y=c.y, z=0.0) for c in msg.cones
                           if math.sqrt(c.x**2 + c.y**2) <= self.centroid_threshold]
        self.compute_and_publish_path()

    def compute_and_publish_path(self):
        left = sorted(self.left_cones, key=lambda p: p.x)
        right = sorted(self.right_cones, key=lambda p: p.x)

        if not left or not right:
            self.get_logger().warn("Left or right cones missing, cannot generate path.")
            return

        matched_points = []
        i, j = 0, 0
        while i < len(left) and j < len(right):
            left_point = left[i]
            right_point = right[j]

            if abs(left_point.x - right_point.x) > 0.5:
                if left_point.x < right_point.x:
                    i += 1
                else:
                    j += 1
                continue

            cx = (left_point.x + right_point.x) / 2.0 + 0.4  # X 전방 offset
            cy = (left_point.y + right_point.y) / 2.0
            matched_points.append((cx, cy))
            i += 1
            j += 1

        if len(matched_points) < 3:
            self.get_logger().warn("Not enough matched points for spline interpolation.")
            return

        xs, ys = zip(*matched_points)
        cs = CubicSpline(xs, ys)

        x_new = np.linspace(xs[0], xs[-1], 100)
        y_new = cs(x_new)

        waypoints = [Point(x=float(x), y=float(y), z=0.0) for x, y in zip(x_new, y_new)]

        self.publish_markers(waypoints)
        self.publish_path(waypoints)

    def publish_path(self, points):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for p in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = p
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(f"Published Path with {len(points)} points")

    def publish_markers(self, waypoints):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, point in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = point
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.3 * 1e9)
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()