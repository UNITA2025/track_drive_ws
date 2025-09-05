import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from interfaces_pkg.msg import ErpCmdMsg1, ErpStatusMsg1

class SmoothController(Node):
    def __init__(self):
        super().__init__('smooth_controller')
        self.current_position = None
        self.current_yaw = None
        self.erp_status = None
        self.waypoints = []
        self.waypoint_index = 0

        self.erp_cmd_pub = self.create_publisher(ErpCmdMsg1, '/erp42_ctrl_cmd', 10)

        # Subscribers
        self.path_sub = self.create_subscription(Path, '/waypoint_path', self.path_callback, 10)
        self.status_sub = self.create_subscription(ErpStatusMsg1, '/erp42_status', self.status_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/local_enu2', self.odom_callback, 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.cmd_msg = ErpCmdMsg1()

    def path_callback(self, msg: Path):
        self.waypoints = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        self.waypoint_index = 0
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    def status_callback(self, msg: ErpStatusMsg1):
        self.erp_status = msg
        if msg.e_stop:
            self.get_logger().warn("E-STOP received! Emergency stop activated.")
            # 즉시 정지 명령 발행
            self.cmd_msg.speed = 0
            self.cmd_msg.brake = 100
            self.cmd_msg.steer = 0
            self.cmd_msg.gear = 1  # 중립
            self.cmd_msg.e_stop = True
            self.erp_cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg: Odometry):
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # e_stop 상태 시 제어 중단
        if self.erp_status and self.erp_status.e_stop:
            return

        if self.current_position is None or self.current_yaw is None or not self.waypoints:
            return

        # 웨이포인트 도달 체크 및 인덱스 갱신
        while self.waypoint_index < len(self.waypoints):
            wp = self.waypoints[self.waypoint_index]
            dist = math.hypot(wp[0] - self.current_position[0], wp[1] - self.current_position[1])
            if dist < 0.5:
                self.waypoint_index += 1
            else:
                break

        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().info("Reached final waypoint. Stopping vehicle.")
            self.cmd_msg.speed = 0
            self.cmd_msg.brake = 100
            self.cmd_msg.steer = 0
            self.cmd_msg.gear = 1  # 중립
            self.cmd_msg.e_stop = False
            self.erp_cmd_pub.publish(self.cmd_msg)
            return

        target_wp = self.waypoints[self.waypoint_index]

        # Pure Pursuit 조향 계산
        dx = target_wp[0] - self.current_position[0]
        dy = target_wp[1] - self.current_position[1]
        angle_to_wp = math.atan2(dy, dx)
        angle_error = angle_to_wp - self.current_yaw
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        wheel_base = 1.04
        lookahead_dist = math.hypot(dx, dy)
        steering_rad = math.atan2(2 * wheel_base * math.sin(angle_error), lookahead_dist)
        steering_deg = math.degrees(steering_rad)
        steering_deg = max(min(steering_deg, 30), -30)

        # 곡률 계산 후 속도, 브레이크 조절
        curvature = abs(steering_rad / lookahead_dist) if lookahead_dist > 0 else 0

        if curvature < 0.01:  # 직선
            speed = 5 * 1000 / 3600  # 5km/h -> m/s
            brake = 0
        else:  # 곡선
            speed = 3 * 1000 / 3600  # 3km/h -> m/s
            brake = 20

        # 기어 상태 결정: 항상 전진 (0)으로 설정 가능하며 수정 가능
        gear_cmd = 0  # 0: 직진, 1: 중립, 2: 후진 필요시 조건 추가

        # 제어 명령 세팅
        self.cmd_msg.steer = int((steering_deg / 30) * 2000)
        self.cmd_msg.speed = int(speed * 100)
        self.cmd_msg.brake = brake
        self.cmd_msg.gear = gear_cmd
        self.cmd_msg.e_stop = False

        self.erp_cmd_pub.publish(self.cmd_msg)
        self.get_logger().info(f"Steer: {steering_deg:.1f} deg, Speed: {speed*3.6:.1f} km/h, Brake: {brake}, Gear: {gear_cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = SmoothController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
