#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry   # ✅ Odometry 추가
from interfaces_pkg.msg import ErpCmdMsg1, ErpStatusMsg1  # ERP42 메시지

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.ctrl_cmd_msg = ErpCmdMsg1()

        # 구독
        self.path_sub = self.create_subscription(Path, '/waypoint_path', self.path_callback, 10)
        self.status_sub = self.create_subscription(ErpStatusMsg1, '/erp42_status', self.status_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/local_enu', self.odom_callback, 10)  # ✅ 수정

        # 퍼블리셔
        self.erp_cmd_pub = self.create_publisher(ErpCmdMsg1, '/erp42_ctrl_cmd', 10)

        # 상태 변수
        self.current_position = [0.0, 0.0]
        self.current_yaw = 0.0
        self.current_speed = 0
        self.current_steering_angle = 0
        self.previous_steering_angle = 0

        self.lat_err = deque(maxlen=100)
        self.PID_steer = self.PID(self)

        self.waypoints = []
        self.waypoint_index = 0

        # 10Hz 제어 루프
        self.control_timer = self.create_timer(0.1, self.follow_waypoints)

    # ------------------ Odometry 콜백 ------------------
    def odom_callback(self, msg: Odometry):
        """
        Odometry 메시지에서 위치(x, y), orientation(quaternion) → yaw 계산, 속도 갱신
        """
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.current_speed = msg.twist.twist.linear.x  # ✅ 실제 속도 (m/s)

        self.get_logger().info(
            f"[ODOM] Pos: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}), "
            f"Yaw: {math.degrees(self.current_yaw):.2f} deg, "
            f"Speed: {self.current_speed:.2f} m/s"
        )

    # ------------------ ERP42 상태 콜백 ------------------
    def status_callback(self, msg: ErpStatusMsg1):
        self.current_speed = msg.speed
        # ±2000 카운트 → ±30도
        self.current_steering_angle = (msg.steer / 2000.0) * 30.0

    # ------------------ Path 콜백 ------------------
    def path_callback(self, msg: Path):
        self.waypoints = [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]
        self.waypoint_index = 0
        self.get_logger().info(f"Received path with {len(self.waypoints)} waypoints.")

    # ------------------ Waypoint 추종 ------------------
    def follow_waypoints(self):
        if not self.waypoints or self.waypoint_index >= len(self.waypoints):
            return

        wx, wy = self.waypoints[self.waypoint_index]
        dx = wx - self.current_position[0]
        dy = wy - self.current_position[1]
        dist = math.hypot(dx, dy)

        # 웨이포인트 도달 판정
        if dist < 0.5:
            self.waypoint_index += 1
            return

        # 헤딩 에러 (yaw 보정 포함)
        angle_to_waypoint = math.atan2(dy, dx)
        angle_error = (angle_to_waypoint - self.current_yaw + math.pi) % (2 * math.pi) - math.pi

        # Pure-pursuit 조향
        wheel_base = 1.04
        delta_deg = math.degrees(math.atan2(2.0 * wheel_base * math.sin(angle_error), dist))
        steering_angle_deg = max(min(delta_deg, 30.0), -30.0)

        # PID 보정
        delta_err = steering_angle_deg - self.current_steering_angle
        steer_pid = self.PID_steer.control(delta_err)
        steer_pid = max(min(steer_pid, 30.0), -30.0)

        # 최종 조향 명령: ±30도 → ±2000 카운트
        steering_angle_cmd = int((steer_pid / 30.0) * 2000)
        steering_angle_cmd = max(min(steering_angle_cmd, 2000), -2000)

        # 곡률 기반 속도
        k = abs(2.0 * math.sin(angle_error) / max(dist, 1e-3))
        if k < 0.1:
            speed = 40
        elif k < 0.2:
            speed = 20
        else:
            speed = 15

        # 횡오차 추정
        self.lat_err.append(abs(dy))
        avg_lat_err = sum(self.lat_err) / len(self.lat_err)

        # 명령 퍼블리시
        self.ctrl_cmd_msg.steer = steering_angle_cmd
        self.ctrl_cmd_msg.speed = speed
        self.ctrl_cmd_msg.gear = 0
        self.ctrl_cmd_msg.e_stop = False
        self.ctrl_cmd_msg.brake = 0
        self.erp_cmd_pub.publish(self.ctrl_cmd_msg)

        self.get_logger().info(
            f"[ERP42 CMD] Speed: {speed}, Steer: {steering_angle_cmd}, "
            f"Curvature: {k:.4f}, LatErr(avg): {avg_lat_err:.2f}"
        )

        self.previous_steering_angle = self.current_steering_angle

    # ------------------ 간단 PID 클래스 ------------------
    class PID:
        def __init__(self, outer):
            self.node = outer
            self.kp = 1.0
            self.ki = 0.001
            self.kd = 0.001
            self.Pterm = 0.0
            self.Iterm = 0.0
            self.Dterm = 0.0
            self.prev_error = 0.0
            self.dt = 0.1

        def control(self, error):
            self.Pterm = self.kp * error
            self.Dterm = self.kd * (error - self.prev_error) / self.dt
            if abs(error) < 15.0:
                self.Iterm += error * self.dt
            self.prev_error = error
            output = self.Pterm + self.ki * self.Iterm + self.Dterm
            self.node.get_logger().info(
                f"P: {self.Pterm:.2f}, I: {self.ki * self.Iterm:.2f}, D: {self.Dterm:.2f}"
            )
            return output

# ------------------ main ------------------
def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
