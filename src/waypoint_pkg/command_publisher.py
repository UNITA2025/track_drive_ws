#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from interfaces_control_pkg.msg import ErpCmdMsg, ErpStatusMsg
from sensor_msgs.msg import Imu

# ==================== 튜닝 파라미터 ====================
# 값이 작으면 더 정확히 waypoint를 통과해야 다음으로 넘어감
# 값이 크면 waypoint 근처만 가도 통과 처리됨
WAYPOINT_RADIUS = 0.4  # 웨이포인트 도달 판정 반경 (m)

WHEEL_BASE = 1.04
MAX_STEER_DEG = 30.0
MIN_LOOKAHEAD = 0.5

# 속도 제한 (m/s)
V_MAX = 4.0 * 1000.0 / 3600.0  # 4 km/h
V_MIN = 1.5 * 1000.0 / 3600.0   # 1.5 km/h

# 곡률 감속 계수
CURVATURE_GAIN = 5.0

# PID 파라미터 (steering 제어)
#KP (비례 이득): 차량이 waypoint를 향해 빠르게 반응하는 정도, 너무 크면 차량이 심하게 흔들림 (overshoot), 너무 작으면 반응이 느리고 곡선 추종이 안 됨
#KI (적분 이득): 오차가 누적될 때 보정, 보통 Steering에서는 거의 0 또는 매우 작게 둠 (드리프트 보정용)
#KD (미분 이득): 급격한 오차 변화 억제 (방향 전환 안정화), 너무 크면 떨림 발생
STEER_KP, STEER_KI, STEER_KD = 1.0, 0.00, 0.4
STEER_OUTPUT_LIMIT = (-30.0, 30.0)  # deg
STEER_INTEGRAL_LIMIT = 20.0

# PID 파라미터 (speed 제어, m/s 단위)
#KP: 속도 오차에 대한 직접 반응 (차가 목표 속도에 빨리 도달)
#KI: 언덕이나 마찰 같은 지속적 편차 보정 (과속/저속 방지)
#KD: 속도 변동 억제, 부드럽게 반응
SPEED_KP, SPEED_KI, SPEED_KD = 1.5, 0.00, 0.05
SPEED_OUTPUT_LIMIT = (-3.0, 3.0)
SPEED_INTEGRAL_LIMIT = 2.0
#튜닝 순서:
#1. KI=0, KD=0으로 두고 KP부터 올림 → 목표 속도 근처까지 반응 확인
#2.목표 근처에서 오차가 계속 남으면 KI 조금 증가
#3.응답이 출렁이면 KD 조금 추가

CONTROL_DT = 0.1  # 제어 주기 (s)

# 엔코더 해상도 (실차 보정 필요) → 1 tick = 1mm 로 가정
ENCODER_RESOLUTION = 0.001  # m/tick
# ======================================================


class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limits = output_limits
        self.integral_limit = integral_limit

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.output_limits is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))

        self.prev_error = error
        return output


class SmoothController(Node):
    def __init__(self):
        super().__init__('control_node')
        self.current_yaw = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_encoder = None
        self.erp_status = None
        self.current_steer_deg = 0.0
        self.current_speed_mps = 0.0
        self.waypoints = []
        self.waypoint_index = 0

        self.encoder_resolution = ENCODER_RESOLUTION

        # ROS publishers & subscribers
        self.erp_cmd_pub = self.create_publisher(ErpCmdMsg, '/erp42_ctrl_cmd', 10)
        self.path_sub = self.create_subscription(Path, '/waypoint_path', self.path_callback, 10)
        self.status_sub = self.create_subscription(ErpStatusMsg, '/erp42_status', self.status_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.create_timer(CONTROL_DT, self.control_loop)
        self.cmd_msg = ErpCmdMsg()

        # PID 초기화
        self.pid_steer = PIDController(STEER_KP, STEER_KI, STEER_KD, CONTROL_DT,
                                       output_limits=STEER_OUTPUT_LIMIT, integral_limit=STEER_INTEGRAL_LIMIT)
        self.pid_speed = PIDController(SPEED_KP, SPEED_KI, SPEED_KD, CONTROL_DT,
                                       output_limits=SPEED_OUTPUT_LIMIT, integral_limit=SPEED_INTEGRAL_LIMIT)

    # ===== Callbacks =====
    def path_callback(self, msg: Path):
        self.waypoints = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        self.waypoint_index = 0   # 무조건 첫 waypoint부터 시작
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints, starting at index 0")
        self.pid_steer.reset()
        self.pid_speed.reset()

    def status_callback(self, msg: ErpStatusMsg):
        self.erp_status = msg
        self.current_steer_deg = (msg.steer / 2000.0) * MAX_STEER_DEG if msg.steer is not None else 0.0
        self.current_speed_mps = (msg.speed / 10.0) / 3.6 if getattr(msg, 'speed', None) is not None else 0.0

        # Dead reckoning with encoder
        if self.last_encoder is not None and self.current_yaw is not None:
            delta_encoder = msg.encoder - self.last_encoder
            delta_s = delta_encoder * self.encoder_resolution
            self.current_x += delta_s * math.cos(self.current_yaw)
            self.current_y += delta_s * math.sin(self.current_yaw)

        self.last_encoder = msg.encoder

        if getattr(msg, 'e_stop', False):
            self.get_logger().warn("E-STOP received! Emergency stop activated.")
            self.stop_vehicle()

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ===== Vehicle Control =====
    def stop_vehicle(self):
        self.cmd_msg.steer = 0
        self.cmd_msg.speed = 0
        self.cmd_msg.brake = 33   # 최대 제동
        self.cmd_msg.gear = 1     # Neutral
        self.cmd_msg.e_stop = True
        self.erp_cmd_pub.publish(self.cmd_msg)
        self.get_logger().warn("Vehicle STOP command published!")

    def control_loop(self):
        if self.erp_status and getattr(self.erp_status, 'e_stop', False):
            self.stop_vehicle()
            return
        if self.current_yaw is None or not self.waypoints:
            return

        # 현재 목표 waypoint
        target_wp = self.waypoints[self.waypoint_index]
        dx = target_wp[0] - self.current_x
        dy = target_wp[1] - self.current_y
        dist_to_wp = math.hypot(dx, dy)

        # waypoint 도달 → 다음 waypoint로
        if dist_to_wp < WAYPOINT_RADIUS and self.waypoint_index < len(self.waypoints) - 1:
            self.waypoint_index += 1
            target_wp = self.waypoints[self.waypoint_index]
            dx = target_wp[0] - self.current_x
            dy = target_wp[1] - self.current_y
            dist_to_wp = math.hypot(dx, dy)

        # steering 제어
        angle_to_wp = math.atan2(dy, dx)
        angle_error = (angle_to_wp - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
        lookahead_dist = max(dist_to_wp, MIN_LOOKAHEAD)
        steering_rad_pp = math.atan2(2.0 * WHEEL_BASE * math.sin(angle_error), lookahead_dist)
        steering_deg_pp = max(min(math.degrees(steering_rad_pp), MAX_STEER_DEG), -MAX_STEER_DEG)

        error_deg = steering_deg_pp - self.current_steer_deg
        pid_correction_deg = self.pid_steer.update(error_deg)
        commanded_steer_deg = max(min(self.current_steer_deg + pid_correction_deg, MAX_STEER_DEG), -MAX_STEER_DEG)
        steer_cmd_int = int((commanded_steer_deg / MAX_STEER_DEG) * 2000)

        # curvature 기반 목표 속도
        curvature = abs(2.0 * math.sin(angle_error) / lookahead_dist) if lookahead_dist > 0 else 0.0
        target_speed_mps = V_MAX / (1.0 + CURVATURE_GAIN * curvature)
        target_speed_mps = max(min(target_speed_mps, V_MAX), V_MIN)

        # Speed PID 제어
        speed_error = target_speed_mps - self.current_speed_mps
        pid_out = self.pid_speed.update(speed_error)
        desired_speed_mps = self.current_speed_mps + pid_out
        desired_speed_mps = max(0.0, min(desired_speed_mps, V_MAX))

        speed_cmd = int(desired_speed_mps * 3.6 * 10)  # ERP42 단위 변환 (0.1 km/h)

        # ERP42 Command
        self.cmd_msg.steer = steer_cmd_int
        self.cmd_msg.speed = speed_cmd
        if speed_cmd > 0:
            self.cmd_msg.brake = 1
            self.cmd_msg.gear = 0   # Drive
        else:
            self.cmd_msg.brake = 33
            self.cmd_msg.gear = 1   # Neutral
        self.cmd_msg.e_stop = False
        self.erp_cmd_pub.publish(self.cmd_msg)

        # Debug
        self.get_logger().info(
            f"WP#{self.waypoint_index} | dist:{dist_to_wp:.2f}m | "
            f"pos:({self.current_x:.2f},{self.current_y:.2f}) | "
            f"steer:{commanded_steer_deg:.2f}deg | tgt_spd:{target_speed_mps*3.6:.2f}km/h | "
            f"cur_spd:{self.current_speed_mps*3.6:.2f}km/h"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SmoothController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, stopping vehicle...")
        node.stop_vehicle()
    finally:
        node.stop_vehicle()  # 종료 시 항상 STOP
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
