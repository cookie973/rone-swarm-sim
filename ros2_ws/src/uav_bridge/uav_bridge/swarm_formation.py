#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swarm_formation.py — 6-UAV 蜂群编队控制节点

功能:
    - 订阅每架无人机的 ENU 位置与速度 (/{ns}/uav/odom)
    - 支持 anchored 与 leader_follower 两种编队模式
    - 使用五次多项式轨迹生成平滑编队切换
    - 使用闭环速度控制连续发布速度指令
    - 在机间距离过近时触发硬暂停，避免继续逼近
"""

import itertools
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, TypeAlias

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


Vector3: TypeAlias = Tuple[float, float, float]


def _vec_add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_scale(a: Vector3, scale: float) -> Vector3:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _vec_norm(a: Vector3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _vec_norm_xy(a: Vector3) -> float:
    return math.hypot(a[0], a[1])


def _distance_sq(a: Vector3, b: Vector3) -> float:
    delta = _vec_sub(a, b)
    return delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]


def _minimum_circle_radius(n: int, spacing: float) -> float:
    if n <= 1:
        return 0.0
    if n == 2:
        return spacing / 2.0
    return spacing / (2.0 * math.sin(math.pi / n))


def _polar_point(radius: float, angle: float, altitude: float) -> Vector3:
    return (radius * math.cos(angle), radius * math.sin(angle), altitude)


def _bearing(point: Vector3, fallback: float) -> float:
    if _vec_norm_xy(point) < 1e-3:
        return fallback
    return math.atan2(point[1], point[0])


def formation_circle(
    n: int,
    radius: float = 5.0,
    altitude: float = 5.0,
    spacing: float = 4.0,
) -> List[Vector3]:
    if n <= 0:
        return []
    if n == 1:
        return [(0.0, 0.0, altitude)]

    radius = max(radius, _minimum_circle_radius(n, spacing) * 1.15)
    points: List[Vector3] = []
    for i in range(n):
        angle = math.pi / 2.0 + 2.0 * math.pi * i / n
        points.append((radius * math.cos(angle), radius * math.sin(angle), altitude))
    return points


def formation_line(n: int, spacing: float = 3.0, altitude: float = 5.0) -> List[Vector3]:
    offset = -(n - 1) * spacing / 2.0
    return [(offset + i * spacing, 0.0, altitude) for i in range(n)]


def formation_v_shape(n: int, spacing: float = 3.0, altitude: float = 5.0) -> List[Vector3]:
    points: List[Vector3] = [(0.0, 0.0, altitude)]
    for i in range(1, n):
        side = 1 if i % 2 == 1 else -1
        rank = (i + 1) // 2
        points.append((-rank * spacing, side * rank * spacing, altitude))
    return points


def formation_grid(n: int, spacing: float = 3.0, altitude: float = 5.0) -> List[Vector3]:
    if n == 6:
        half_x = spacing
        half_y = spacing / 2.0
        return [
            (-half_x, -half_y, altitude),
            (0.0, -half_y, altitude),
            (half_x, -half_y, altitude),
            (-half_x, half_y, altitude),
            (0.0, half_y, altitude),
            (half_x, half_y, altitude),
        ]

    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    origin_x = -(cols - 1) * spacing / 2.0
    origin_y = -(rows - 1) * spacing / 2.0
    points: List[Vector3] = []
    for i in range(n):
        row, col = divmod(i, cols)
        points.append((origin_x + col * spacing, origin_y + row * spacing, altitude))
    return points


FORMATIONS = {
    "circle": formation_circle,
    "line": formation_line,
    "v_shape": formation_v_shape,
    "grid": formation_grid,
}

CONTROL_MODES = {"anchored", "leader_follower"}


def _best_assignment(current_positions: List[Vector3], desired_targets: List[Vector3]) -> List[int]:
    indices = list(range(len(desired_targets)))
    best_perm = indices
    best_cost = math.inf

    for perm in itertools.permutations(indices):
        cost = 0.0
        for i, target_idx in enumerate(perm):
            cost += _distance_sq(current_positions[i], desired_targets[target_idx])
        if cost < best_cost:
            best_cost = cost
            best_perm = list(perm)

    return list(best_perm)


def _cyclic_assignment(current_points: List[Vector3], desired_points: List[Vector3]) -> List[int]:
    count = len(desired_points)
    if count <= 1:
        return list(range(count))

    angle_step = 2.0 * math.pi / count
    current_order = sorted(
        range(count),
        key=lambda idx: (_bearing(current_points[idx], idx * angle_step), _vec_norm_xy(current_points[idx])),
    )
    desired_order = sorted(
        range(count),
        key=lambda idx: (_bearing(desired_points[idx], idx * angle_step), _vec_norm_xy(desired_points[idx])),
    )

    best_shift = 0
    best_cost = math.inf
    for shift in range(count):
        cost = 0.0
        for rank, current_idx in enumerate(current_order):
            desired_idx = desired_order[(rank + shift) % count]
            cost += _distance_sq(current_points[current_idx], desired_points[desired_idx])
        if cost < best_cost:
            best_cost = cost
            best_shift = shift

    assignment = [0] * count
    for rank, current_idx in enumerate(current_order):
        assignment[current_idx] = desired_order[(rank + best_shift) % count]
    return assignment


def _centroid(points: List[Vector3], altitude: float) -> Vector3:
    if not points:
        return (0.0, 0.0, altitude)
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    return (sum_x / len(points), sum_y / len(points), altitude)


def build_anchored_slots(name: str, n: int, radius: float, spacing: float, altitude: float) -> List[Vector3]:
    if name == "circle":
        return formation_circle(n, radius, altitude, spacing)
    return FORMATIONS.get(name, formation_circle)(n, spacing, altitude)


def leader_follower_offsets(name: str, n: int, radius: float, spacing: float) -> List[Vector3]:
    offsets: List[Vector3] = [(0.0, 0.0, 0.0)]
    follower_count = max(0, n - 1)

    if follower_count == 0:
        return offsets

    if name == "circle":
        # Spread the circular follower arc out more aggressively so mode/formation
        # transitions do not squeeze followers back toward the leader.
        arc_radius = max(radius * 1.3, _minimum_circle_radius(follower_count + 1, spacing) * 1.3)
        start = math.pi * 0.15
        end = math.pi * 0.85
        for i in range(follower_count):
            angle = start + (end - start) * i / max(1, follower_count - 1)
            offsets.append((-arc_radius * math.cos(angle), arc_radius * math.sin(angle), 0.0))
        return offsets

    if name == "line":
        for rank in range(1, n):
            offsets.append((-rank * spacing, 0.0, 0.0))
        return offsets

    if name == "v_shape":
        for i in range(1, n):
            side = 1 if i % 2 == 1 else -1
            rank = (i + 1) // 2
            offsets.append((-rank * spacing, side * rank * spacing, 0.0))
        return offsets

    if name == "grid":
        cols = max(2, math.ceil(math.sqrt(follower_count)))
        lateral_spacing = spacing * 1.25
        row_spacing = spacing * 1.5
        for i in range(follower_count):
            row, col = divmod(i, cols)
            y = (col - (cols - 1) / 2.0) * lateral_spacing
            x = -((row + 1.5) * row_spacing)
            offsets.append((x, y, 0.0))
        return offsets

    cols = max(2, math.ceil(math.sqrt(follower_count)))
    for i in range(follower_count):
        row, col = divmod(i, cols)
        y = (col - (cols - 1) / 2.0) * spacing
        x = -(row + 1) * spacing
        offsets.append((x, y, 0.0))
    return offsets


@dataclass
class QuinticProfile1D:
    p0: float
    v0: float
    pf: float
    start_time: float
    duration: float

    def sample(self, now: float) -> Tuple[float, float]:
        if self.duration <= 1e-6:
            return (self.pf, 0.0)

        elapsed = min(max(0.0, now - self.start_time), self.duration)
        if elapsed >= self.duration:
            return (self.pf, 0.0)

        t = elapsed
        T = self.duration
        delta = self.pf - self.p0

        a0 = self.p0
        a1 = self.v0
        a2 = 0.0
        a3 = (10.0 * delta - 6.0 * self.v0 * T) / (T ** 3)
        a4 = (-15.0 * delta + 8.0 * self.v0 * T) / (T ** 4)
        a5 = (6.0 * delta - 3.0 * self.v0 * T) / (T ** 5)

        position = a0 + a1 * t + a2 * t * t + a3 * (t ** 3) + a4 * (t ** 4) + a5 * (t ** 5)
        velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * (t ** 2) + 4.0 * a4 * (t ** 3) + 5.0 * a5 * (t ** 4)
        return (position, velocity)


@dataclass
class QuinticProfile3D:
    x: QuinticProfile1D
    y: QuinticProfile1D
    z: QuinticProfile1D

    def sample(self, now: float) -> Tuple[Vector3, Vector3]:
        px, vx = self.x.sample(now)
        py, vy = self.y.sample(now)
        pz, vz = self.z.sample(now)
        return ((px, py, pz), (vx, vy, vz))


@dataclass
class SegmentedProfile3D:
    segments: List[QuinticProfile3D]

    def sample(self, now: float) -> Tuple[Vector3, Vector3]:
        if not self.segments:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        for segment in self.segments:
            end_time = segment.x.start_time + segment.x.duration
            if now <= end_time:
                return segment.sample(now)

        final_segment = self.segments[-1]
        return final_segment.sample(final_segment.x.start_time + final_segment.x.duration)


class SwarmFormation(Node):

    def __init__(self):
        super().__init__("swarm_formation")

        self.declare_parameter("num_uav", 6)
        self.declare_parameter("ns_prefix", "uav")
        self.declare_parameter("default_formation", "circle")
        self.declare_parameter("control_mode", "leader_follower")
        self.declare_parameter("control_reference", False)
        self.declare_parameter("formation_radius", 5.0)
        self.declare_parameter("formation_spacing", 4.0)
        self.declare_parameter("formation_altitude", 5.0)
        self.declare_parameter("position_gain", 0.48)
        self.declare_parameter("velocity_gain", 0.42)
        self.declare_parameter("max_speed", 0.65)
        self.declare_parameter("max_accel", 0.12)
        self.declare_parameter("control_rate", 15.0)
        self.declare_parameter("min_separation", 3.0)
        self.declare_parameter("repulsion_gain", 4.5)
        self.declare_parameter("safety_pause_distance", 3.1)
        self.declare_parameter("safety_resume_distance", 3.8)
        self.declare_parameter("safety_recovery_speed", 0.75)
        self.declare_parameter("transition_duration", 12.0)

        self.num_uav = int(self.get_parameter("num_uav").value)
        self.ns_prefix = str(self.get_parameter("ns_prefix").value)
        self.formation_name = str(self.get_parameter("default_formation").value)
        self.control_mode = str(self.get_parameter("control_mode").value)
        self.control_reference = bool(self.get_parameter("control_reference").value)
        self.radius = float(self.get_parameter("formation_radius").value)
        self.spacing = float(self.get_parameter("formation_spacing").value)
        self.altitude = float(self.get_parameter("formation_altitude").value)
        self.kp_pos = float(self.get_parameter("position_gain").value)
        self.kp_vel = float(self.get_parameter("velocity_gain").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_accel = float(self.get_parameter("max_accel").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.min_separation = float(self.get_parameter("min_separation").value)
        self.repulsion_gain = float(self.get_parameter("repulsion_gain").value)
        self.safety_pause_distance = float(self.get_parameter("safety_pause_distance").value)
        self.safety_resume_distance = max(
            self.safety_pause_distance + 0.1,
            float(self.get_parameter("safety_resume_distance").value),
        )
        self.safety_recovery_speed = float(self.get_parameter("safety_recovery_speed").value)
        self.transition_duration = max(0.1, float(self.get_parameter("transition_duration").value))
        self.reference_index = 0
        self.control_period = 1.0 / max(1.0, self.control_rate)
        self.safety_trigger_cycles = 4
        self.safety_resume_cycles = 8

        if self.formation_name not in FORMATIONS:
            self.get_logger().warn(f"未知默认队形 {self.formation_name}，回退到 circle")
            self.formation_name = "circle"
        if self.control_mode not in CONTROL_MODES:
            self.get_logger().warn(f"未知默认模式 {self.control_mode}，回退到 leader_follower")
            self.control_mode = "leader_follower"
        if self.spacing < self.min_separation:
            self.spacing = self.min_separation + 0.5
            self.get_logger().warn(
                f"formation_spacing 已提升到 {self.spacing:.2f}m，以保证安全编队间距"
            )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odoms: List[Optional[Odometry]] = [None] * self.num_uav
        self.vel_pubs = []
        self.anchor_profiles: List[Optional[SegmentedProfile3D]] = [None] * self.num_uav
        self.leader_offset_profiles: List[Optional[SegmentedProfile3D]] = [None] * self.num_uav
        self.anchor_goals: List[Vector3] = [(0.0, 0.0, self.altitude)] * self.num_uav
        self.leader_offset_goals: List[Vector3] = [(0.0, 0.0, 0.0)] * self.num_uav
        self.last_commands: List[Vector3] = [(0.0, 0.0, 0.0)] * self.num_uav
        self.anchor_layout_dirty = True
        self.leader_layout_dirty = True
        self.safety_pause_active = False
        self.safety_close_cycles = 0
        self.safety_clear_cycles = 0

        for i in range(self.num_uav):
            ns = f"/{self.ns_prefix}{i + 1}"
            self.create_subscription(
                Odometry,
                f"{ns}/uav/odom",
                lambda msg, idx=i: self._odom_cb(idx, msg),
                sensor_qos,
            )
            self.vel_pubs.append(
                self.create_publisher(TwistStamped, f"{ns}/uav/cmd/velocity", cmd_qos)
            )

        self.create_subscription(String, "/swarm/formation", self._formation_cb, 10)
        self.create_subscription(String, "/swarm/mode", self._mode_cb, 10)
        self.timer = self.create_timer(self.control_period, self._control_loop)

        self.get_logger().info(
            "SwarmFormation 启动: "
            f"{self.num_uav} UAVs, 默认队形={self.formation_name}, 模式={self.control_mode}, "
            f"参考机=uav{self.reference_index + 1}, vmax={self.max_speed:.2f}m/s, amax={self.max_accel:.2f}m/s^2"
        )

    def _odom_cb(self, idx: int, msg: Odometry):
        self.odoms[idx] = msg

    def _formation_cb(self, msg: String):
        name = msg.data.strip().lower()
        if name not in FORMATIONS:
            self.get_logger().warn(f"未知队形 '{name}', 可选: {sorted(FORMATIONS)}")
            return
        self.formation_name = name
        self.anchor_layout_dirty = True
        self.leader_layout_dirty = True
        self.get_logger().info(f"队形切换 -> {name}")

    def _mode_cb(self, msg: String):
        mode = msg.data.strip().lower()
        if mode not in CONTROL_MODES:
            self.get_logger().warn(f"未知控制模式 '{mode}', 可选: {sorted(CONTROL_MODES)}")
            return
        self.control_mode = mode
        self.anchor_layout_dirty = True
        self.leader_layout_dirty = True
        self.get_logger().info(f"控制模式切换 -> {mode}")

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _positions_by_uav(self) -> Optional[List[Vector3]]:
        positions: List[Vector3] = []
        for odom in self.odoms:
            if odom is None:
                return None
            positions.append(
                (
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                )
            )
        return positions

    def _velocities_by_uav(self) -> Optional[List[Vector3]]:
        velocities: List[Vector3] = []
        for odom in self.odoms:
            if odom is None:
                return None
            velocities.append(
                (
                    odom.twist.twist.linear.x,
                    odom.twist.twist.linear.y,
                    odom.twist.twist.linear.z,
                )
            )
        return velocities

    def _plan_duration(self, starts: List[Vector3], goals: List[Vector3], start_vels: List[Vector3]) -> float:
        duration = self.transition_duration
        for start, goal, start_vel in zip(starts, goals, start_vels):
            distance = _vec_norm(_vec_sub(goal, start))
            if distance < 1e-6:
                continue
            velocity_bound = 1.875 * distance / max(0.1, self.max_speed)
            accel_bound = math.sqrt(5.7735 * distance / max(0.05, self.max_accel))
            moving_bound = _vec_norm(start_vel) / max(0.05, self.max_accel)
            duration = max(duration, velocity_bound, accel_bound, moving_bound)
        return max(0.5, duration)

    def _build_profile(
        self,
        start: Vector3,
        start_vel: Vector3,
        goal: Vector3,
        start_time: float,
        duration: float,
    ) -> QuinticProfile3D:
        return QuinticProfile3D(
            x=QuinticProfile1D(start[0], start_vel[0], goal[0], start_time, duration),
            y=QuinticProfile1D(start[1], start_vel[1], goal[1], start_time, duration),
            z=QuinticProfile1D(start[2], start_vel[2], goal[2], start_time, duration),
        )

    def _build_segmented_profiles(
        self,
        starts: List[Vector3],
        start_vels: List[Vector3],
        waypoint_groups: List[List[Vector3]],
    ) -> List[SegmentedProfile3D]:
        segments: List[List[QuinticProfile3D]] = [[] for _ in waypoint_groups]
        phase_starts = list(starts)
        phase_vels = list(start_vels)
        phase_start_time = self._now_seconds()
        phase_count = max((len(points) for points in waypoint_groups), default=0)

        for phase_idx in range(phase_count):
            phase_goals = [points[min(phase_idx, len(points) - 1)] for points in waypoint_groups]
            duration = self._plan_duration(phase_starts, phase_goals, phase_vels)
            for idx, goal in enumerate(phase_goals):
                segments[idx].append(
                    self._build_profile(phase_starts[idx], phase_vels[idx], goal, phase_start_time, duration)
                )
            phase_starts = phase_goals
            phase_vels = [(0.0, 0.0, 0.0)] * len(waypoint_groups)
            phase_start_time += duration

        return [SegmentedProfile3D(profile_segments) for profile_segments in segments]

    def _build_transition_waypoints(
        self,
        starts: List[Vector3],
        goals: List[Vector3],
        moving_count: int,
    ) -> List[List[Vector3]]:
        if moving_count <= 1:
            return [[goal] for goal in goals]

        base_radius = _minimum_circle_radius(moving_count, self.safety_resume_distance)
        goal_radius = max((_vec_norm_xy(goal) for goal in goals), default=0.0)
        transition_radius = max(base_radius, goal_radius + self.spacing)

        angle_step = 2.0 * math.pi / moving_count
        waypoints: List[List[Vector3]] = []
        for idx, (start, goal) in enumerate(zip(starts, goals)):
            current_angle = _bearing(start, idx * angle_step)
            goal_angle = _bearing(goal, current_angle)
            current_radius = _vec_norm_xy(start)
            breakout_radius = max(current_radius, transition_radius)
            breakout = _polar_point(breakout_radius, current_angle, goal[2])
            transfer = _polar_point(transition_radius, goal_angle, goal[2])
            waypoints.append([breakout, transfer, goal])
        return waypoints

    def _refresh_anchor_profiles(self):
        positions = self._positions_by_uav()
        velocities = self._velocities_by_uav()
        if positions is None or velocities is None:
            return

        center = _centroid(positions, self.altitude)
        slots = build_anchored_slots(
            self.formation_name,
            self.num_uav,
            self.radius,
            self.spacing,
            self.altitude,
        )
        desired_targets = [
            (center[0] + slot[0], center[1] + slot[1], slot[2])
            for slot in slots
        ]
        relative_positions = [_vec_sub(position, center) for position in positions]
        relative_targets = [_vec_sub(target, center) for target in desired_targets]
        assignment = _cyclic_assignment(relative_positions, relative_targets)
        self.anchor_goals = [desired_targets[target_idx] for target_idx in assignment]

        relative_anchor_goals = [_vec_sub(goal, center) for goal in self.anchor_goals]
        waypoint_groups = self._build_transition_waypoints(relative_positions, relative_anchor_goals, self.num_uav)
        absolute_waypoint_groups = [
            [
                (center[0] + point[0], center[1] + point[1], point[2])
                for point in points
            ]
            for points in waypoint_groups
        ]
        self.anchor_profiles = self._build_segmented_profiles(positions, velocities, absolute_waypoint_groups)
        self.anchor_layout_dirty = False
        self.get_logger().info("anchored 轨迹已重规划，采用三段式过渡")

    def _refresh_leader_profiles(self):
        positions = self._positions_by_uav()
        velocities = self._velocities_by_uav()
        if positions is None or velocities is None:
            return

        leader_pos = positions[self.reference_index]
        leader_vel = velocities[self.reference_index]
        current_offsets: List[Vector3] = [(0.0, 0.0, 0.0)] * self.num_uav
        current_relative_vels: List[Vector3] = [(0.0, 0.0, 0.0)] * self.num_uav
        follower_indices: List[int] = []
        follower_offsets: List[Vector3] = []

        for i in range(self.num_uav):
            if i == self.reference_index:
                continue
            follower_indices.append(i)
            offset = _vec_sub(positions[i], leader_pos)
            relative_vel = _vec_sub(velocities[i], leader_vel)
            current_offsets[i] = offset
            current_relative_vels[i] = relative_vel
            follower_offsets.append(offset)

        desired_offsets = leader_follower_offsets(
            self.formation_name,
            self.num_uav,
            self.radius,
            self.spacing,
        )
        desired_follower_offsets = [
            desired_offsets[i] for i in range(self.num_uav) if i != self.reference_index
        ]
        assignment = _cyclic_assignment(follower_offsets, desired_follower_offsets)

        self.leader_offset_goals = [(0.0, 0.0, 0.0)] * self.num_uav
        for follower_order, uav_idx in enumerate(follower_indices):
            self.leader_offset_goals[uav_idx] = desired_follower_offsets[assignment[follower_order]]

        start_offsets = [current_offsets[i] for i in range(self.num_uav)]
        start_rel_vels = [current_relative_vels[i] for i in range(self.num_uav)]
        follower_start_offsets = [start_offsets[i] for i in follower_indices]
        follower_goal_offsets = [self.leader_offset_goals[i] for i in follower_indices]
        follower_waypoints = self._build_transition_waypoints(
            follower_start_offsets,
            follower_goal_offsets,
            len(follower_indices),
        )
        waypoint_groups: List[List[Vector3]] = [[(0.0, 0.0, 0.0)] for _ in range(self.num_uav)]
        for follower_order, uav_idx in enumerate(follower_indices):
            waypoint_groups[uav_idx] = follower_waypoints[follower_order]
        self.leader_offset_profiles = self._build_segmented_profiles(start_offsets, start_rel_vels, waypoint_groups)
        self.leader_layout_dirty = False
        self.get_logger().info(
            f"leader_follower 轨迹已重规划，参考机=uav{self.reference_index + 1}，采用三段式过渡"
        )

    def _avoidance_velocity(self, idx: int, positions: List[Vector3]) -> Vector3:
        repel_x = 0.0
        repel_y = 0.0
        repel_z = 0.0
        current = positions[idx]

        for other_idx, other in enumerate(positions):
            if other_idx == idx:
                continue
            dx = current[0] - other[0]
            dy = current[1] - other[1]
            dz = current[2] - other[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < 1e-6 or distance >= self.min_separation:
                continue

            strength = self.repulsion_gain * (self.min_separation - distance) / self.min_separation
            repel_x += strength * dx / distance
            repel_y += strength * dy / distance
            repel_z += 0.2 * strength * dz / distance

        return (repel_x, repel_y, repel_z)

    def _apply_accel_limit(self, idx: int, command: Vector3) -> Vector3:
        previous = self.last_commands[idx]
        delta = _vec_sub(command, previous)
        max_delta = self.max_accel * self.control_period
        delta_norm = _vec_norm(delta)
        if delta_norm > max_delta > 0.0:
            delta = _vec_scale(delta, max_delta / delta_norm)
        limited = _vec_add(previous, delta)

        speed = _vec_norm(limited)
        if speed > self.max_speed > 0.0:
            limited = _vec_scale(limited, self.max_speed / speed)

        self.last_commands[idx] = limited
        return limited

    def _tracking_command(
        self,
        idx: int,
        desired_position: Vector3,
        desired_velocity: Vector3,
        positions: List[Vector3],
        velocities: List[Vector3],
    ) -> Vector3:
        current_position = positions[idx]
        current_velocity = velocities[idx]
        pos_err = _vec_sub(desired_position, current_position)
        vel_err = _vec_sub(desired_velocity, current_velocity)

        feedback = (
            self.kp_pos * pos_err[0] + self.kp_vel * vel_err[0],
            self.kp_pos * pos_err[1] + self.kp_vel * vel_err[1],
            self.kp_pos * pos_err[2] + self.kp_vel * vel_err[2],
        )
        command = _vec_add(desired_velocity, feedback)
        command = _vec_add(command, self._avoidance_velocity(idx, positions))
        return self._apply_accel_limit(idx, command)

    def _evaluate_safety_pause(self, positions: List[Vector3]) -> bool:
        closest_distance = math.inf
        closest_pair: Optional[Tuple[int, int]] = None

        for i in range(self.num_uav):
            for j in range(i + 1, self.num_uav):
                distance = math.sqrt(_distance_sq(positions[i], positions[j]))
                if distance < closest_distance:
                    closest_distance = distance
                    closest_pair = (i, j)

        if closest_pair is None:
            return False

        if closest_distance < self.safety_pause_distance:
            self.safety_close_cycles += 1
            self.safety_clear_cycles = 0
            if not self.safety_pause_active and self.safety_close_cycles >= self.safety_trigger_cycles:
                self.safety_pause_active = True
                self.get_logger().warn(
                    f"安全暂停触发: uav{closest_pair[0] + 1} 与 uav{closest_pair[1] + 1} 距离 {closest_distance:.2f}m"
                )
            return True

        self.safety_close_cycles = 0

        if self.safety_pause_active and closest_distance > self.safety_resume_distance:
            self.safety_clear_cycles += 1
        else:
            self.safety_clear_cycles = 0

        if self.safety_pause_active and self.safety_clear_cycles >= self.safety_resume_cycles:
            self.safety_pause_active = False
            self.safety_clear_cycles = 0
            self.anchor_layout_dirty = True
            self.leader_layout_dirty = True
            self.get_logger().info(
                f"安全暂停解除: 最近距离恢复到 {closest_distance:.2f}m，重新规划轨迹"
            )

        return self.safety_pause_active

    def _publish_velocity(self, idx: int, velocity: Vector3):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = velocity[0]
        cmd.twist.linear.y = velocity[1]
        cmd.twist.linear.z = velocity[2]
        self.vel_pubs[idx].publish(cmd)

    def _publish_zero_all(self):
        for i in range(self.num_uav):
            self.last_commands[i] = (0.0, 0.0, 0.0)
            self._publish_velocity(i, (0.0, 0.0, 0.0))

    def _publish_safety_recovery(self, positions: List[Vector3]):
        recovery_commands: List[Vector3] = [(0.0, 0.0, 0.0)] * self.num_uav

        for i in range(self.num_uav):
            repel = [0.0, 0.0, 0.0]
            for j in range(self.num_uav):
                if i == j:
                    continue
                delta = _vec_sub(positions[i], positions[j])
                distance = _vec_norm(delta)
                if distance < 1e-6 or distance > self.safety_resume_distance:
                    continue
                strength = (self.safety_resume_distance - distance) / self.safety_resume_distance
                repel[0] += strength * delta[0] / distance
                repel[1] += strength * delta[1] / distance
            repel_vec = (repel[0], repel[1], 0.0)
            repel_norm = _vec_norm(repel_vec)
            if repel_norm > 1e-6:
                recovery_commands[i] = _vec_scale(
                    repel_vec,
                    min(self.safety_recovery_speed, repel_norm) / repel_norm,
                )

        for i, command in enumerate(recovery_commands):
            limited = command
            speed = _vec_norm(limited)
            if speed > self.safety_recovery_speed > 0.0:
                limited = _vec_scale(limited, self.safety_recovery_speed / speed)
            self.last_commands[i] = limited
            self._publish_velocity(i, limited)

    def _control_loop(self):
        positions = self._positions_by_uav()
        velocities = self._velocities_by_uav()
        if positions is None or velocities is None:
            return

        if self._evaluate_safety_pause(positions):
            self._publish_safety_recovery(positions)
            return

        now = self._now_seconds()
        if self.control_mode == "leader_follower":
            self._leader_follower_control(now, positions, velocities)
            return
        self._anchored_control(now, positions, velocities)

    def _anchored_control(self, now: float, positions: List[Vector3], velocities: List[Vector3]):
        if self.anchor_layout_dirty or any(profile is None for profile in self.anchor_profiles):
            self._refresh_anchor_profiles()

        for i in range(self.num_uav):
            profile = self.anchor_profiles[i]
            if profile is None:
                continue
            desired_position, desired_velocity = profile.sample(now)
            command = self._tracking_command(i, desired_position, desired_velocity, positions, velocities)
            self._publish_velocity(i, command)

    def _leader_follower_control(self, now: float, positions: List[Vector3], velocities: List[Vector3]):
        if self.leader_layout_dirty or any(profile is None for profile in self.leader_offset_profiles):
            self._refresh_leader_profiles()

        leader_position = positions[self.reference_index]
        leader_velocity = velocities[self.reference_index]

        for i in range(self.num_uav):
            if i == self.reference_index and not self.control_reference:
                self.last_commands[i] = (0.0, 0.0, 0.0)
                continue
            profile = self.leader_offset_profiles[i]
            if profile is None:
                continue
            desired_offset, desired_offset_velocity = profile.sample(now)
            desired_position = _vec_add(leader_position, desired_offset)
            desired_velocity = _vec_add(leader_velocity, desired_offset_velocity)
            command = self._tracking_command(i, desired_position, desired_velocity, positions, velocities)
            self._publish_velocity(i, command)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmFormation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
