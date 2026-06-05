#!/usr/bin/env python3

"""
nav_explorer.py

A small, parameterized ROS 2 navigation node based on the stable simple_explorer.
It reads LaserScan + OccupancyGrid + TF and publishes Twist commands.

Intended to be run by nav_stack.py, but can also be run directly:

python3 nav_explorer.py --ros-args \
  -p scan_topic:=/lidar2 \
  -p map_topic:=/map \
  -p cmd_vel_topic:=/model/vehicle_blue/cmd_vel \
  -p map_frame:=map \
  -p base_frame:=vehicle_blue/chassis \
  -p use_sim_time:=true
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.executors import ExternalShutdownException

import tf2_ros


class NavExplorer(Node):
    def __init__(self):
        super().__init__('nav_explorer')

        # Topic/frame params. These are the main portability knobs.
        self.declare_parameter('scan_topic', '/lidar2')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('found_topic', '/detections/found')
        self.declare_parameter('stop_on_found', True)

        # Movement params.
        self.declare_parameter('forward_speed', 0.30)
        self.declare_parameter('escape_forward_speed', 0.30)
        self.declare_parameter('backup_speed', -0.25)
        self.declare_parameter('turn_speed', 0.65)

        # Obstacle/recovery params.
        self.declare_parameter('front_stop_distance', 1.2)
        self.declare_parameter('corner_stop_distance', 0.5)
        self.declare_parameter('side_danger_distance', 0.4)
        self.declare_parameter('recovery_front_clear_distance', 1.0)
        self.declare_parameter('recovery_corner_clear_distance', 0.4)
        self.declare_parameter('recovery_side_clear_distance', 0.2)
        self.declare_parameter('emergency_front_distance', 0.6)
        self.declare_parameter('rear_clear_distance', 0.8)
        self.declare_parameter('rear_corner_clear_distance', 0.6)
        self.declare_parameter('emergency_backup_duration_sec', 0.8)
        self.declare_parameter('min_recovery_spin_sec', 0.8)
        self.declare_parameter('max_recovery_spin_sec', 4.0)
        self.declare_parameter('escape_forward_duration_sec', 1.0)

        # Exploration/completion params.
        self.declare_parameter('unknown_search_radius_m', 8.0)
        self.declare_parameter('unknown_max_front_m', 8.0)
        self.declare_parameter('exploration_max_bias', 0.40)
        self.declare_parameter('exploration_gain', 0.85)
        self.declare_parameter('exploration_deadband', 0.08)
        self.declare_parameter('min_unknown_for_bias', 15.0)
        self.declare_parameter('completion_check_interval_sec', 3.0)
        self.declare_parameter('completion_low_unknown_threshold', 8.0)
        self.declare_parameter('completion_required_low_checks', 5)
        self.declare_parameter('disable_completion_stop', False)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.found_topic = self.get_parameter('found_topic').value
        self.stop_on_found = bool(self.get_parameter('stop_on_found').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.escape_forward_speed = float(self.get_parameter('escape_forward_speed').value)
        self.backup_speed = float(self.get_parameter('backup_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)

        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.corner_stop_distance = float(self.get_parameter('corner_stop_distance').value)
        self.side_danger_distance = float(self.get_parameter('side_danger_distance').value)
        self.recovery_front_clear_distance = float(self.get_parameter('recovery_front_clear_distance').value)
        self.recovery_corner_clear_distance = float(self.get_parameter('recovery_corner_clear_distance').value)
        self.recovery_side_clear_distance = float(self.get_parameter('recovery_side_clear_distance').value)
        self.emergency_front_distance = float(self.get_parameter('emergency_front_distance').value)
        self.rear_clear_distance = float(self.get_parameter('rear_clear_distance').value)
        self.rear_corner_clear_distance = float(self.get_parameter('rear_corner_clear_distance').value)
        self.emergency_backup_duration_sec = float(self.get_parameter('emergency_backup_duration_sec').value)
        self.min_recovery_spin_sec = float(self.get_parameter('min_recovery_spin_sec').value)
        self.max_recovery_spin_sec = float(self.get_parameter('max_recovery_spin_sec').value)
        self.escape_forward_duration_sec = float(self.get_parameter('escape_forward_duration_sec').value)

        self.unknown_search_radius_m = float(self.get_parameter('unknown_search_radius_m').value)
        self.unknown_max_front_m = float(self.get_parameter('unknown_max_front_m').value)
        self.exploration_max_bias = float(self.get_parameter('exploration_max_bias').value)
        self.exploration_gain = float(self.get_parameter('exploration_gain').value)
        self.exploration_deadband = float(self.get_parameter('exploration_deadband').value)
        self.min_unknown_for_bias = float(self.get_parameter('min_unknown_for_bias').value)
        self.completion_check_interval_sec = float(self.get_parameter('completion_check_interval_sec').value)
        self.completion_low_unknown_threshold = float(self.get_parameter('completion_low_unknown_threshold').value)
        self.completion_required_low_checks = int(self.get_parameter('completion_required_low_checks').value)
        self.disable_completion_stop = bool(self.get_parameter('disable_completion_stop').value)

        self.scan = None
        self.map = None
        self.state = 'FORWARD'

        self.turn_direction = 1.0
        self.recovery_spin_until = None
        self.recovery_spin_started = None
        self.escape_forward_until = None
        self.emergency_backup_until = None
        self.failed_recovery_spins = 0
        self.shutdown_requested = False

        self.last_completion_check_ns = 0
        self.low_unknown_check_count = 0

        self.front_width = math.radians(40)
        self.corner_width = math.radians(30)
        self.side_width = math.radians(25)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        if self.stop_on_found:
            self.create_subscription(Bool, self.found_topic, self.found_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            'Nav explorer started with '
            f'scan_topic={self.scan_topic}, map_topic={self.map_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, map_frame={self.map_frame}, '
            f'base_frame={self.base_frame}, found_topic={self.found_topic}, '
            f'stop_on_found={self.stop_on_found}'
        )

    def scan_callback(self, msg):
        self.scan = msg

    def map_callback(self, msg):
        self.map = msg

    def found_callback(self, msg):
        if not self.stop_on_found or not msg.data or self.shutdown_requested:
            return

        self.get_logger().warn(
            f'Detection found on {self.found_topic}. Stopping navigation and exiting.'
        )
        self.shutdown_requested = True
        self.state = 'STOPPED'

        # Publish multiple zero commands so the bridge/sim receives at least one.
        for _ in range(5):
            self.stop_robot()

        # Let nav_stack.py observe this process exit and shut down the stack.
        rclpy.shutdown()

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def angle_diff(self, a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def sector_distance(self, target_angle, width):
        if self.scan is None:
            return float('inf')

        ranges = np.array(self.scan.ranges, dtype=float)
        if len(ranges) == 0:
            return float('inf')

        vals = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r):
                continue
            if r <= self.scan.range_min or r >= self.scan.range_max:
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(self.angle_diff(angle, target_angle)) <= width:
                vals.append(r)

        if not vals:
            return float('inf')
        return float(min(vals))

    def scan_clearances(self):
        front = self.sector_distance(math.radians(0), self.front_width)
        front_left = self.sector_distance(math.radians(45), self.corner_width)
        front_right = self.sector_distance(math.radians(-45), self.corner_width)
        left = self.sector_distance(math.radians(90), self.side_width)
        right = self.sector_distance(math.radians(-90), self.side_width)
        return front, front_left, front_right, left, right

    def rear_clearances(self):
        rear = self.sector_distance(math.radians(180), self.front_width)
        rear_left = self.sector_distance(math.radians(135), self.corner_width)
        rear_right = self.sector_distance(math.radians(-135), self.corner_width)
        return rear, rear_left, rear_right

    def rear_is_clear(self):
        rear, rear_left, rear_right = self.rear_clearances()
        return (
            rear > self.rear_clear_distance
            and rear_left > self.rear_corner_clear_distance
            and rear_right > self.rear_corner_clear_distance
        )

    def path_blocked(self):
        front, front_left, front_right, left, right = self.scan_clearances()
        self.get_logger().info(
            f'Clearance: front={front:.2f}, FL={front_left:.2f}, '
            f'FR={front_right:.2f}, L={left:.2f}, R={right:.2f}'
        )
        return (
            front < self.front_stop_distance
            or front_left < self.corner_stop_distance
            or front_right < self.corner_stop_distance
        )

    def emergency_too_close(self):
        front, front_left, front_right, _, _ = self.scan_clearances()
        return (
            front < self.emergency_front_distance
            or front_left < self.emergency_front_distance
            or front_right < self.emergency_front_distance
        )

    def recovery_corridor_clear(self):
        front, front_left, front_right, left, right = self.scan_clearances()
        self.get_logger().info(
            f'Recovery clear check: front={front:.2f}, FL={front_left:.2f}, '
            f'FR={front_right:.2f}, L={left:.2f}, R={right:.2f}'
        )
        return (
            front > self.recovery_front_clear_distance
            and front_left > self.recovery_corner_clear_distance
            and front_right > self.recovery_corner_clear_distance
            and left > self.recovery_side_clear_distance
            and right > self.recovery_side_clear_distance
        )

    def get_robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'No {self.map_frame} -> {self.base_frame} TF yet: {e}')
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return x, y, yaw

    def count_unknown_left_right(self):
        if self.map is None:
            return None, None

        pose = self.get_robot_pose_in_map()
        if pose is None:
            return None, None

        robot_x, robot_y, yaw = pose
        info = self.map.info
        if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
            return None, None

        data = np.array(self.map.data, dtype=np.int16).reshape((info.height, info.width))
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        robot_cx = int((robot_x - origin_x) / resolution)
        robot_cy = int((robot_y - origin_y) / resolution)
        cells = int(self.unknown_search_radius_m / resolution)

        left_unknown = 0.0
        right_unknown = 0.0
        front_unknown = 0.0

        y_start = max(0, robot_cy - cells)
        y_end = min(info.height, robot_cy + cells)
        x_start = max(0, robot_cx - cells)
        x_end = min(info.width, robot_cx + cells)

        for gy in range(y_start, y_end):
            for gx in range(x_start, x_end):
                if data[gy, gx] != -1:
                    continue

                wx = origin_x + (gx + 0.5) * resolution
                wy = origin_y + (gy + 0.5) * resolution
                dx = wx - robot_x
                dy = wy - robot_y

                local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
                local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

                if local_x < 0.2 or local_x > self.unknown_max_front_m:
                    continue

                distance = math.hypot(local_x, local_y)
                weight = 1.0 / max(distance, 1.0)

                if abs(local_y) < 0.25:
                    front_unknown += weight
                elif local_y > 0.0:
                    left_unknown += weight
                else:
                    right_unknown += weight

        left_unknown += front_unknown * 0.5
        right_unknown += front_unknown * 0.5
        return left_unknown, right_unknown

    def exploration_bias(self):
        left_unknown, right_unknown = self.count_unknown_left_right()
        if left_unknown is None or right_unknown is None:
            return 0.0

        total = left_unknown + right_unknown
        if total < self.min_unknown_for_bias:
            return 0.0

        score = (left_unknown - right_unknown) / max(total, 1.0)
        if abs(score) < self.exploration_deadband:
            return 0.0

        bias = max(-self.exploration_max_bias, min(self.exploration_max_bias, score * self.exploration_gain))
        self.get_logger().info(
            f'Explore bias: left_unknown={left_unknown:.1f}, '
            f'right_unknown={right_unknown:.1f}, angular_bias={bias:.2f}'
        )
        return bias

    def check_exploration_complete(self, now_ns):
        if self.disable_completion_stop:
            return False

        elapsed_sec = (now_ns - self.last_completion_check_ns) / 1e9
        if elapsed_sec < self.completion_check_interval_sec:
            return False
        self.last_completion_check_ns = now_ns

        left_unknown, right_unknown = self.count_unknown_left_right()
        if left_unknown is None or right_unknown is None:
            self.get_logger().warn('Completion check skipped: map/TF unavailable.')
            self.low_unknown_check_count = 0
            return False

        total_unknown = left_unknown + right_unknown
        self.get_logger().info(
            f'Completion check: total_unknown={total_unknown:.1f}, '
            f'low_count={self.low_unknown_check_count}/{self.completion_required_low_checks}'
        )

        if total_unknown < self.completion_low_unknown_threshold:
            self.low_unknown_check_count += 1
        else:
            self.low_unknown_check_count = 0

        if self.low_unknown_check_count >= self.completion_required_low_checks:
            self.get_logger().warn('Exploration complete: nearby unknown space stayed low.')
            return True
        return False

    def choose_turn_direction(self):
        front, front_left, front_right, left, right = self.scan_clearances()
        left_unknown, right_unknown = self.count_unknown_left_right()

        if front_left < front_right - 0.15:
            self.turn_direction = -1.0
            self.get_logger().info('Obstacle closer on front-left. Turning RIGHT.')
            return

        if front_right < front_left - 0.15:
            self.turn_direction = 1.0
            self.get_logger().info('Obstacle closer on front-right. Turning LEFT.')
            return

        if left_unknown is not None and right_unknown is not None:
            self.get_logger().info(f'Unknown: left={left_unknown:.1f}, right={right_unknown:.1f}')
            self.turn_direction = -1.0 if right_unknown > left_unknown else 1.0
            self.get_logger().info('Turning toward larger unknown side.')
            return

        self.turn_direction = -1.0 if right > left else 1.0
        self.get_logger().info('Fallback: turning toward larger lidar clearance.')

    def forward_with_clearance_steering(self, speed=None):
        front, front_left, front_right, left, right = self.scan_clearances()
        if speed is None:
            speed = self.forward_speed

        angular = 0.0

        # Strong local safety steering. Exploration only adds after this.
        if front_left < self.corner_stop_distance * 1.7:
            angular -= 0.35
        if front_right < self.corner_stop_distance * 1.7:
            angular += 0.35
        if left < self.side_danger_distance:
            angular -= 0.30
        if right < self.side_danger_distance:
            angular += 0.30

        angular += self.exploration_bias()
        angular = max(-0.70, min(0.70, angular))
        self.publish_cmd(speed, angular)

    def enter_recovery_spin(self, now_ns):
        self.choose_turn_direction()
        self.recovery_spin_started = now_ns
        self.recovery_spin_until = now_ns + int(self.max_recovery_spin_sec * 1e9)
        self.state = 'RECOVERY_SPIN'
        self.get_logger().warn(f'Entering RECOVERY_SPIN direction={self.turn_direction}')

    def loop(self):
        if self.shutdown_requested:
            self.stop_robot()
            return

        if self.scan is None or self.map is None:
            self.stop_robot()
            return

        now_ns = self.get_clock().now().nanoseconds

        if self.state == 'FORWARD':
            if self.check_exploration_complete(now_ns):
                self.get_logger().warn('Area appears fully mapped. Stopping robot.')
                self.stop_robot()
                self.state = 'STOPPED'
                return

            if self.emergency_too_close() and self.rear_is_clear():
                self.get_logger().warn('Emergency too close. Brief backup.')
                self.emergency_backup_until = now_ns + int(self.emergency_backup_duration_sec * 1e9)
                self.state = 'EMERGENCY_BACKUP'
                return

            if self.path_blocked():
                self.stop_robot()
                self.enter_recovery_spin(now_ns)
                return

            self.forward_with_clearance_steering()

        elif self.state == 'RECOVERY_SPIN':
            elapsed_spin_sec = (now_ns - self.recovery_spin_started) / 1e9

            if elapsed_spin_sec >= self.min_recovery_spin_sec and self.recovery_corridor_clear():
                self.get_logger().warn('Recovery corridor clear. Committing forward.')
                self.stop_robot()
                self.escape_forward_until = now_ns + int(self.escape_forward_duration_sec * 1e9)
                self.state = 'ESCAPE_FORWARD'
                return

            if now_ns > self.recovery_spin_until:
                self.failed_recovery_spins += 1
                self.turn_direction *= -1.0
                self.get_logger().warn(
                    f'Recovery spin failed. Flipping direction. '
                    f'failed_recovery_spins={self.failed_recovery_spins}'
                )
                if self.failed_recovery_spins >= 2:
                    self.get_logger().warn('Too many failed spins. Forcing backup.')
                    self.failed_recovery_spins = 0
                    self.emergency_backup_until = now_ns + int(2.0 * 1e9)
                    self.state = 'EMERGENCY_BACKUP'
                    return
                self.turn_direction *= -1.0
                self.recovery_spin_started = now_ns
                self.recovery_spin_until = now_ns + int(self.max_recovery_spin_sec * 1e9)
                return

            self.publish_cmd(0.0, self.turn_direction * self.turn_speed)

        elif self.state == 'ESCAPE_FORWARD':
            if self.emergency_too_close():
                self.get_logger().warn('Escape forward got too close. Spinning again.')
                self.stop_robot()
                self.enter_recovery_spin(now_ns)
                return

            if now_ns < self.escape_forward_until:
                self.forward_with_clearance_steering(speed=self.escape_forward_speed)
            else:
                self.get_logger().info('Escape forward complete. Returning to FORWARD.')
                self.state = 'FORWARD'

        elif self.state == 'EMERGENCY_BACKUP':
            if not self.rear_is_clear():
                self.get_logger().warn('Rear blocked during emergency backup. Spinning.')
                self.stop_robot()
                self.enter_recovery_spin(now_ns)
                return

            if now_ns < self.emergency_backup_until:
                self.publish_cmd(self.backup_speed, 0.0)
            else:
                self.stop_robot()
                self.enter_recovery_spin(now_ns)

        elif self.state == 'STOPPED':
            self.stop_robot()


def main():
    rclpy.init()
    node = NavExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
