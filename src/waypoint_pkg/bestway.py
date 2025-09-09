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
        self.default_offset = 1.2  # Y축 offset(좌우), X축 offset은 코드 내 고정 0.4 적용
        self.frame_id = "velodyne"
        self.left_color = "yellow"
        self.right_color = "blue"
        self.steering_angle_deg = 0.0

        # 좌측, 우측 콘 리스트
        self.left_cones = []
        self.right_cones = []
        # 하향 센서 좌우 콘 리스트
        self.left_cones_down = []
        self.right_cones_down = []

        # 센서별 토픽 구독
        self.left_centroid_sub = self.create_subscription(
            ConeInfoArray, '/cones/cone_info_left', self.left_centroid_callback, 10)
        self.right_centroid_sub = self.create_subscription(
            ConeInfoArray, '/cones/cone_info_right', self.right_centroid_callback, 10)
        self.down_centroid_sub = self.create_subscription(
            ConeInfoArray, '/cones/cone_info_down', self.down_centroid_callback, 10)
        self.steering_angle_sub = self.create_subscription(
            ErpStatusMsg1
        , '/erp42_status', self.steering_angle_callback, 10)

        # 출판자
        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

    def steering_angle_callback(self, msg: ErpStatusMsg1
):
        self.steering_angle_deg = (msg.steer / 2000.0) * 30.0

    def left_centroid_callback(self, msg: ConeInfoArray):
        # 거리 필터링해 좌측 콘 리스트 갱신
        self.left_cones = [Point(x=c.x, y=c.y, z=0.0)
                           for c in msg.cones
                           if math.sqrt(c.x**2 + c.y**2) <= self.centroid_threshold]
        self.compute_and_publish_path()

    def right_centroid_callback(self, msg: ConeInfoArray):
        # 거리 필터링해 우측 콘 리스트 갱신
        self.right_cones = [Point(x=c.x, y=c.y, z=0.0)
                            for c in msg.cones
                            if math.sqrt(c.x**2 + c.y**2) <= self.centroid_threshold]
        self.compute_and_publish_path()

    def down_centroid_callback(self, msg: ConeInfoArray):
        # 거리 필터링 및 색상 분할로 좌우 콘 리스트 갱신
        left = []
        right = []
        for c in msg.cones:
            dist = math.sqrt(c.x**2 + c.y**2)
            if dist > self.centroid_threshold:
                continue
            if c.cone_color == self.left_color:
                left.append(Point(x=c.x, y=c.y, z=0.0))
            elif c.cone_color == self.right_color:
                right.append(Point(x=c.x, y=c.y, z=0.0))
        self.left_cones_down = left
        self.right_cones_down = right
        self.compute_and_publish_path_down()

    def compute_and_publish_path(self):
        # 좌우 콘 리스트 정렬
        left = sorted(self.left_cones, key=lambda p: p.x)
        right = sorted(self.right_cones, key=lambda p: p.x)

        # 좌우 콘 둘 다 없는 경우
        if not left and not right:
            self.get_logger().warn("No cones received on either side.")
            return

        waypoints = []
        i, j = 0, 0

        # 좌우 번갈아 기준점 결정하며 경로 생성
        while i < len(left) or j < len(right):
            if i < len(left) and j < len(right):
                if left[i].x <= right[j].x:
                    base_side = 'left'
                    base_point = left[i]
                    i += 1
                else:
                    base_side = 'right'
                    base_point = right[j]
                    j += 1
            elif i < len(left):
                base_side = 'left'
                base_point = left[i]
                i += 1
            elif j < len(right):
                base_side = 'right'
                base_point = right[j]
                j += 1
            else:
                break

            opp_list = right if base_side == 'left' else left
            min_x_diff = float('inf')
            min_idx = -1

            # 반대편 콘 중 가장 x좌표 차이 작은 인덱스 찾기
            for idx, pt in enumerate(opp_list):
                x_diff = abs(base_point.x - pt.x)
                if x_diff < min_x_diff:
                    min_x_diff = x_diff
                    min_idx = idx

            if min_x_diff >= 0.4 or min_idx == -1:
                # 한쪽만 보일 때 offset 적용
                if base_side == 'left':
                    new_x = base_point.x - 0.4         # X offset 뒤쪽으로
                    new_y = base_point.y - self.default_offset  # Y offset 오른쪽 (음수)
                else:
                    new_x = base_point.x + 0.4         # X offset 앞쪽으로
                    new_y = base_point.y + self.default_offset  # Y offset 왼쪽 (양수)
                waypoints.append(Point(x=new_x, y=new_y, z=0.0))
            else:
                # 좌우 쌍 중점 계산 후 전방 offset 더하기
                opp_point = opp_list[min_idx]
                cx = (base_point.x + opp_point.x) / 2.0 + 0.4
                cy = (base_point.y + opp_point.y) / 2.0
                waypoints.append(Point(x=cx, y=cy, z=0.0))
                del opp_list[min_idx]

        if not waypoints:
            self.get_logger().warn("No waypoints generated after processing.")
            return

        # 보간법 적용 위한 x, y 분리
        xs = [p.x for p in waypoints]
        ys = [p.y for p in waypoints]
        # 스플라인 보간 (CubicSpline) 수행, 점 수 100 개
        if len(xs) < 3:
            # 점이 3개 미만이면 보간 안 하고 그냥 publish
            self.publish_markers(waypoints)
            self.publish_path(waypoints)
            return

        cs = CubicSpline(xs, ys)
        x_new = np.linspace(xs[0], xs[-1], 100)
        y_new = cs(x_new)
        smooth_waypoints = [Point(x=float(x), y=float(y), z=0.0) for x, y in zip(x_new, y_new)]

        self.publish_markers(smooth_waypoints)
        self.publish_path(smooth_waypoints)

    def compute_and_publish_path_down(self):
        # 하향 센서 좌우 콘 정렬
        left = sorted(self.left_cones_down, key=lambda p: p.x)
        right = sorted(self.right_cones_down, key=lambda p: p.x)

        # 좌우 콘 둘 다 없는 경우
        if not left and not right:
            self.get_logger().warn("No cones received from down sensor.")
            return

        waypoints = []
        i, j = 0, 0

        # 좌우 번갈아 기준점 결정하며 경로 생성
        while i < len(left) or j < len(right):
            if i < len(left) and j < len(right):
                if left[i].x <= right[j].x:
                    base_side = 'left'
                    base_point = left[i]
                    i += 1
                else:
                    base_side = 'right'
                    base_point = right[j]
                    j += 1
            elif i < len(left):
                base_side = 'left'
                base_point = left[i]
                i += 1
            elif j < len(right):
                base_side = 'right'
                base_point = right[j]
                j += 1
            else:
                break

            opp_list = right if base_side == 'left' else left
            min_x_diff = float('inf')
            min_idx = -1

            for idx, pt in enumerate(opp_list):
                x_diff = abs(base_point.x - pt.x)
                if x_diff < min_x_diff:
                    min_x_diff = x_diff
                    min_idx = idx

            if min_x_diff >= 0.4 or min_idx == -1:
                if base_side == 'left':
                    new_x = base_point.x - 0.4
                    new_y = base_point.y - self.default_offset
                else:
                    new_x = base_point.x + 0.4
                    new_y = base_point.y + self.default_offset
                waypoints.append(Point(x=new_x, y=new_y, z=0.0))
            else:
                opp_point = opp_list[min_idx]
                cx = (base_point.x + opp_point.x) / 2.0 + 0.4
                cy = (base_point.y + opp_point.y) / 2.0
                waypoints.append(Point(x=cx, y=cy, z=0.0))
                del opp_list[min_idx]

        if not waypoints:
            self.get_logger().warn("No waypoints generated after processing from down sensor.")
            return

        # 보간처리 (선택사항, 필요 시 적용)
        xs = [p.x for p in waypoints]
        ys = [p.y for p in waypoints]
        if len(xs) < 3:
            self.publish_markers(waypoints)
            self.publish_path(waypoints)
            return

        cs = CubicSpline(xs, ys)
        x_new = np.linspace(xs[0], xs[-1], 100)
        y_new = cs(x_new)
        smooth_waypoints = [Point(x=float(x), y=float(y), z=0.0) for x, y in zip(x_new, y_new)]

        self.publish_markers(smooth_waypoints)
        self.publish_path(smooth_waypoints)

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
