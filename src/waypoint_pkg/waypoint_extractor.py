#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from interfaces_control_pkg.msg import ErpStatusMsg
from interfaces_pkg.msg import ConeInfoArray


class WaypointExtractor(Node):
    def __init__(self):
        super().__init__('waypoint_extractor')

        # ===== params =====
        self.centroid_threshold = 3.0
        self.default_offset = 1.5
        self.frame_id = "velodyne"
        self.left_color = "blue"
        self.right_color = "yellow"
        self.steering_angle_deg = 0.0

        # 주기 / 동기화
        self.process_hz = 10.0
        self.sync_timeout_sec = 0.2   # 좌/우 최신 데이터 간 허용 지연
        self._last_pub_time = None

        # ===== state (caches) =====
        self._left_pts = []
        self._right_pts = []
        self._t_left = None
        self._t_right = None

        # ===== subs =====
        # 양쪽 토픽을 같은 콜백으로 받되, "발행"은 하지 않고 캐시만 갱신
        self.create_subscription(ConeInfoArray, '/cones/cone_info_left',  self._cones_callback, 10)
        self.create_subscription(ConeInfoArray, '/cones/cone_info_right', self._cones_callback, 10)
        self.create_subscription(ErpStatusMsg, '/erp42_status', self.steering_angle_callback, 10)

        # ===== pubs =====
        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # 타이머: 한 번만 계산/발행
        self.create_timer(1.0 / self.process_hz, self._on_timer)

    # ---------------- Callbacks ----------------
    def steering_angle_callback(self, msg: ErpStatusMsg):
        self.steering_angle_deg = (msg.steer / 2000.0) * 30.0

    def _cones_callback(self, msg: ConeInfoArray):
        """좌/우 토픽 모두 이 콜백으로 들어옴. 여기서는 캐시만 갱신하고 발행은 하지 않음."""
        now = self.get_clock().now()

        # 메시지 안에서 양쪽 색을 모두 뽑아 캐시 갱신 (YOLO 색 뒤바뀜 대비)
        left = [
            Point(x=c.x, y=c.y, z=0.0)
            for c in msg.cones
            if c.cone_color == self.left_color and math.hypot(c.x, c.y) <= self.centroid_threshold
        ]
        right = [
            Point(x=c.x, y=c.y, z=0.0)
            for c in msg.cones
            if c.cone_color == self.right_color and math.hypot(c.x, c.y) <= self.centroid_threshold
        ]

        if left:
            left.sort(key=lambda p: p.x)
            self._left_pts = left
            self._t_left = now
        if right:
            right.sort(key=lambda p: p.x)
            self._right_pts = right
            self._t_right = now

    # ---------------- Timer = single publish per tick ----------------
    def _on_timer(self):
        now = self.get_clock().now()

        # 최근 좌/우 데이터 신선도 확인
        def fresh(t):
            if t is None:
                return False
            return (now - t).nanoseconds * 1e-9 <= self.sync_timeout_sec

        left_fresh = fresh(self._t_left)
        right_fresh = fresh(self._t_right)

        # 아무 데이터도 없으면 패스
        if not left_fresh and not right_fresh:
            return

        # 좌/우 데이터 선택
        left = self._left_pts if left_fresh else []
        right = self._right_pts if right_fresh else []

        waypoints = self._build_waypoints(left, right)

        # 중복 제거(좌표를 2cm 해상도로 라운딩)
        uniq = []
        seen = set()
        for p in waypoints:
            key = (round(p.x, 2), round(p.y, 2))
            if key not in seen:
                uniq.append(p)
                seen.add(key)

        # 발행 (이 타이머에서만!)
        self.publish_markers(uniq)
        self.publish_path(uniq)
        self._last_pub_time = now

    # ---------------- Core logic ----------------
    def _build_waypoints(self, left, right):
        waypoints = []

        if left and right:
            # 둘 다 있을 때: 왼쪽 기준 가까운 오른쪽을 찾아 중점
            for base in left:
                candidates = [
                    p for p in right
                    if p.y > base.y and math.hypot(p.x - base.x, p.y - base.y) <= 4.0
                ]
                if candidates:
                    closest = min(candidates, key=lambda p: math.hypot(p.x - base.x, p.y - base.y))
                    mx = (base.x + closest.x) / 2.0
                    my = (base.y + closest.y) / 2.0
                    waypoints.append(Point(x=mx, y=my, z=0.0))
                else:
                    # 오른쪽이 비었으면 왼쪽에서 +오프셋
                    waypoints.append(Point(x=base.x, y=base.y + self.default_offset, z=0.0))
        elif left and not right:
            # 왼쪽만 있을 때: +오프셋
            for base in left:
                waypoints.append(Point(x=base.x, y=base.y + self.default_offset, z=0.0))
        elif right and not left:
            # 오른쪽만 있을 때: -오프셋 (좌표계에 따라 부호 필요시 반대로)
            for base in right:
                waypoints.append(Point(x=base.x, y=base.y - self.default_offset, z=0.0))
        else:
            pass

        return waypoints

    # ---------------- Publishers ----------------
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
