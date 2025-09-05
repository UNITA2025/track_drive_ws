#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from interfaces_pkg.msg import ErpStatusMsg1
from interfaces_pkg.msg import ConeInfoArray  # 메시지 타입

class WaypointExtractor(Node):
    def __init__(self):
        super().__init__('waypoint_extractor')

        # params
        self.centroid_threshold = 5.0
        self.default_offset = 1.5
        self.frame_id = "velodyne"
        self.steering_angle_deg = 0.0

        # subs
        self.centroid_sub = self.create_subscription(
            ConeInfoArray, '/cones/cone_info_down', self.centroid_callback, 10
        )
        self.steering_angle_sub = self.create_subscription(
            ErpStatusMsg1, '/erp42_status', self.steering_angle_callback, 10
        )

        # pubs
        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

    def steering_angle_callback(self, msg: ErpStatusMsg1):
        # ERP42 스티어링 ±2000 카운트 → ±30도
        self.steering_angle_deg = (msg.steer / 2000.0) * 30.0

    def centroid_callback(self, msg: ConeInfoArray):
        left, right = [], []

        # 좌/우 구분: y 좌표 기준
        for c in msg.cones:
            dist = math.sqrt(c.x**2 + c.y**2)
            if dist > self.centroid_threshold:
                continue

            if c.y >= 0:
                left.append(Point(x=c.x, y=c.y, z=0.0))
            else:
                right.append(Point(x=c.x, y=c.y, z=0.0))

        # x 좌표 기준 정렬
        left.sort(key=lambda p: p.x)
        right.sort(key=lambda p: p.x)

        # 웨이포인트 생성
        if left and right:
            waypoints = [Point(x=(l.x+r.x)/2.0, y=(l.y+r.y)/2.0, z=0.0)
                         for l, r in zip(left, right)]
        elif left:
            waypoints = [Point(x=p.x, y=p.y - self.default_offset, z=0.0) for p in left[1:]]
            self.get_logger().warn("Only left cones detected; offset path used.")
        elif right:
            waypoints = [Point(x=p.x, y=p.y + self.default_offset, z=0.0) for p in right[1:]]
            self.get_logger().warn("Only right cones detected; offset path used.")
        else:
            self.get_logger().warn("No cones after filtering.")
            return

        # 퍼블리시
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
        self.get_logger().info(f"Published Path: {len(points)} points")

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
