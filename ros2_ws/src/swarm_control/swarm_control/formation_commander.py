"""formation_commander.py

Centralized formation switching with layered collision avoidance:
1) Lift: hold XY, spread drones to different altitude layers.
2) XY move: keep layer altitude, move each drone to assigned XY target.
3) Merge: keep XY, merge all drones to final common altitude.
"""

import itertools
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String


# ─── GPS → ENU 坐标转换 ──────────────────────────────────────

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = 2.0 * _WGS84_F - _WGS84_F * _WGS84_F


def _llh_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * slat * slat)
    return (
        (N + alt_m) * clat * clon,
        (N + alt_m) * clat * slon,
        (N * (1.0 - _WGS84_E2) + alt_m) * slat,
    )


def _ecef_to_enu(x, y, z, ref_lat_deg, ref_lon_deg, rx, ry, rz):
    dx, dy, dz = x - rx, y - ry, z - rz
    slat = math.sin(math.radians(ref_lat_deg))
    clat = math.cos(math.radians(ref_lat_deg))
    slon = math.sin(math.radians(ref_lon_deg))
    clon = math.cos(math.radians(ref_lon_deg))
    e = -slon * dx + clon * dy
    n = -slat * clon * dx - slat * slon * dy + clat * dz
    u = clat * clon * dx + clat * slon * dy + slat * dz
    return e, n, u


def _gps_to_enu(lat, lon, alt, ref):
    """ref = (lat0, lon0, x0, y0, z0)"""
    x, y, z = _llh_to_ecef(lat, lon, alt)
    return _ecef_to_enu(x, y, z, ref[0], ref[1], ref[2], ref[3], ref[4])


def _enu_to_ecef(e, n, u, ref_lat_deg, ref_lon_deg, rx, ry, rz):
    slat = math.sin(math.radians(ref_lat_deg))
    clat = math.cos(math.radians(ref_lat_deg))
    slon = math.sin(math.radians(ref_lon_deg))
    clon = math.cos(math.radians(ref_lon_deg))

    dx = -slon * e - slat * clon * n + clat * clon * u
    dy = clon * e - slat * slon * n + clat * slon * u
    dz = clat * n + slat * u
    return rx + dx, ry + dy, rz + dz


def _ecef_to_llh(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _WGS84_E2))

    for _ in range(8):
        slat = math.sin(lat)
        N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * slat * slat)
        alt = p / max(math.cos(lat), 1e-9) - N
        lat = math.atan2(z, p * (1.0 - _WGS84_E2 * N / (N + alt)))

    slat = math.sin(lat)
    N = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * slat * slat)
    alt = p / max(math.cos(lat), 1e-9) - N
    return math.degrees(lat), math.degrees(lon), alt


def _enu_to_llh(e, n, u, ref):
    x, y, z = _enu_to_ecef(e, n, u, ref[0], ref[1], ref[2], ref[3], ref[4])
    return _ecef_to_llh(x, y, z)


# ─── 主节点 ──────────────────────────────────────────────────

class FormationCommander(Node):

    def __init__(self):
        super().__init__('formation_commander')

        # ---- 参数 ----
        self.declare_parameter('num_drones', 6)
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('control_rate', 6.0)
        self.declare_parameter('default_formation', 'line_x')
        self.declare_parameter('spacing', 6.0)
        self.declare_parameter('auto_start_default', False)
        self.declare_parameter('min_takeoff_z', 2.0)
        self.declare_parameter('layer_spacing', 0.8)
        self.declare_parameter('layer_base_offset', 0.5)
        self.declare_parameter('xy_tolerance', 0.4)
        self.declare_parameter('z_tolerance', 0.25)
        self.declare_parameter('xy_tolerance_reconfig', 1.2)
        self.declare_parameter('z_tolerance_reconfig', 0.45)
        self.declare_parameter('hold_cycles', 1)
        self.declare_parameter('phase_timeout_lift', 40.0)
        self.declare_parameter('phase_timeout_xy', 90.0)
        self.declare_parameter('phase_timeout_merge', 60.0)
        self.declare_parameter('max_step_xy', 2.5)
        self.declare_parameter('max_step_z', 1.0)
        self.declare_parameter('max_step_xy_reconfig', 6.0)
        self.declare_parameter('max_step_z_reconfig', 2.0)
        self.declare_parameter('path_conflict_distance', 4.0)
        self.declare_parameter('path_conflict_penalty', 1000.0)
        self.declare_parameter('planning_snap_tolerance', 2.0)
        self.declare_parameter('command_topic', 'waypoint')
        self.declare_parameter('phase_timeout_land', 90.0)
        self.declare_parameter('mission_step_xy', 0.8)
        self.declare_parameter('mission_tolerance', 0.8)
        self.declare_parameter('mission_step_xy_reconfig_scale', 0.25)
        self.declare_parameter('mission_step_xy_xy_scale', 0.15)
        self.declare_parameter('mission_step_xy_queue_scale', 0.25)
        self.declare_parameter('search_group_spacing', 5.0)
        self.declare_parameter('search_snap_tolerance', 3.0)
        self.declare_parameter('orbit_radius_default', 12.0)
        self.declare_parameter('orbit_angular_speed_default', 0.12)

        self.num_drones = self.get_parameter('num_drones').value
        self.target_alt = self.get_parameter('target_altitude').value
        self.control_rate = self.get_parameter('control_rate').value
        self.spacing = self.get_parameter('spacing').value
        self.auto_start_default = self.get_parameter('auto_start_default').value
        self.min_takeoff_z = self.get_parameter('min_takeoff_z').value
        self.layer_spacing = self.get_parameter('layer_spacing').value
        self.layer_base_offset = self.get_parameter('layer_base_offset').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value
        self.z_tolerance = self.get_parameter('z_tolerance').value
        self.xy_tolerance_reconfig = self.get_parameter('xy_tolerance_reconfig').value
        self.z_tolerance_reconfig = self.get_parameter('z_tolerance_reconfig').value
        self.hold_cycles = int(self.get_parameter('hold_cycles').value)
        self.phase_timeout_lift = self.get_parameter('phase_timeout_lift').value
        self.phase_timeout_xy = self.get_parameter('phase_timeout_xy').value
        self.phase_timeout_merge = self.get_parameter('phase_timeout_merge').value
        self.max_step_xy = self.get_parameter('max_step_xy').value
        self.max_step_z = self.get_parameter('max_step_z').value
        self.max_step_xy_reconfig = self.get_parameter('max_step_xy_reconfig').value
        self.max_step_z_reconfig = self.get_parameter('max_step_z_reconfig').value
        self.path_conflict_distance = self.get_parameter('path_conflict_distance').value
        self.path_conflict_penalty = self.get_parameter('path_conflict_penalty').value
        self.planning_snap_tolerance = self.get_parameter('planning_snap_tolerance').value
        self.command_topic = self.get_parameter('command_topic').value
        self.phase_timeout_land = self.get_parameter('phase_timeout_land').value
        self.mission_step_xy = self.get_parameter('mission_step_xy').value
        self.mission_tolerance = self.get_parameter('mission_tolerance').value
        self.mission_step_xy_reconfig_scale = self.get_parameter('mission_step_xy_reconfig_scale').value
        self.mission_step_xy_xy_scale = self.get_parameter('mission_step_xy_xy_scale').value
        self.mission_step_xy_queue_scale = self.get_parameter('mission_step_xy_queue_scale').value
        self.search_group_spacing = self.get_parameter('search_group_spacing').value
        self.search_snap_tolerance = self.get_parameter('search_snap_tolerance').value
        self.orbit_radius_default = self.get_parameter('orbit_radius_default').value
        self.orbit_angular_speed_default = self.get_parameter('orbit_angular_speed_default').value

        # ---- 队形定义 ----
        self.formations = self._build_formations()
        self.current_formation = self.get_parameter('default_formation').value

        # ---- 状态 ----
        self.local_pos = {}        # {id: [x,y,z]}  各机本地 ENU（odom）
        self.gps_pos = {}          # {id: (lat,lon,alt)}  最新 GPS
        self.home_offset = {}      # {id: [e,n]}  各机本地原点在公共 ENU 中的位移（仅 XY）
        self.active_global_xy = {}  # {id: [e,n]} assigned final global XY
        self.target_offset_xy = {}   # {id: [dx,dy]} target offset around formation center
        self.phase_start_offset_xy = {}  # {id: [dx,dy]} offset held during lift phase
        self.layer_z = {}           # {id: z_layer}
        self.lift_xy_local = {}     # compatibility cache for current phase entry
        self.phase_hold = {}        # {id: consecutive reached cycles}
        self.phase = 'idle'         # idle/lift/xy/merge/land
        self.phase_start_time = 0.0
        self.land_after_merge = False
        self.land_sent = set()
        self.pending_request = None
        self.active_offset_xy = {}
        self.steady_global_xy = {}
        self.last_plan_center = None
        self.mission_center_goal = None
        self.mission_center_current = None
        self.mission_active = False
        self.mission_speed_mode = 'cruise'
        self.reconfig_has_layers = False
        self.target_global_xy = {}
        self.phase_start_global_xy = {}
        self.search_active = False
        self.search_targets = []
        self.search_assignments = {}
        self.search_target_counts = {}
        self.search_groups = {}
        self.orbit_active = False
        self.orbit_pending_spec = None
        self.orbit_center = None
        self.orbit_radius = self.orbit_radius_default
        self.orbit_angular_speed = self.orbit_angular_speed_default
        self.orbit_phase_map = {}
        self.orbit_start_time = 0.0
        self._ref = None           # (lat0, lon0, x0, y0, z0) 公共 ENU 参考点
        self._calibrated = False
        self._default_started = False

        # ---- ROS 接口 ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.waypoint_pubs = {}
        self.land_pubs = {}
        self.status_pub = self.create_publisher(String, '/swarm/status', 10)
        for i in range(1, self.num_drones + 1):
            self.create_subscription(
                Odometry,
                f'/uav{i}/uav/odom',
                lambda msg, d=i: self._on_odom(d, msg),
                sensor_qos,
            )
            self.create_subscription(
                NavSatFix,
                f'/uav{i}/uav/navsatfix',
                lambda msg, d=i: self._on_gps(d, msg),
                10,
            )
            self.waypoint_pubs[i] = self.create_publisher(
                NavSatFix,
                f'/uav{i}/uav/cmd/waypoint',
                10,
            )
            self.land_pubs[i] = self.create_publisher(
                Empty,
                f'/uav{i}/uav/cmd/land',
                10,
            )

        self.create_subscription(
            String, '/swarm/formation', self._on_formation, 10,
        )
        self.create_subscription(
            Point, '/swarm/mission_center', self._on_mission_center, 10,
        )
        self.create_subscription(
            String, '/swarm/search_mission', self._on_search_mission, 10,
        )
        self.create_subscription(
            String, '/swarm/search_orbit', self._on_search_orbit, 10,
        )
        self.create_subscription(
            Empty, '/swarm/clear_task', self._on_clear_task, 10,
        )

        # ---- 控制定时器 ----
        dt = 1.0 / max(0.5, self.control_rate)
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'FormationCommander 启动: {self.num_drones} 架, '
            f'间距={self.spacing}m, 高度={self.target_alt}m, '
            f'队形={self.current_formation}, '
            f'可用: {list(self.formations.keys())}'
        )

    # ================================================================
    #  队形定义
    # ================================================================

    def _build_formations(self):
        n = self.num_drones
        s = self.spacing
        fmts = {}

        def center_offsets(offsets):
            cx = sum(p[0] for p in offsets) / len(offsets)
            cy = sum(p[1] for p in offsets) / len(offsets)
            return [[p[0] - cx, p[1] - cy] for p in offsets]

        # ---- 横队：X(East) 方向一字排开 ----
        half = (n - 1) * s / 2.0
        fmts['line_x'] = center_offsets([[i * s - half, 0.0] for i in range(n)])

        # ---- 纵队：Y(North) 方向一字排开 ----
        fmts['line_y'] = center_offsets([[0.0, i * s - half] for i in range(n)])

        # ---- T 型编队：横杠 4 架 + 竖杆 2 架 ----
        bar_n = min(4, n)
        bar_half = (bar_n - 1) * s / 2.0
        t_offsets = [[i * s - bar_half, s] for i in range(bar_n)]
        for j in range(n - bar_n):
            t_offsets.append([0.0, -j * s])
        fmts['t_shape'] = center_offsets(t_offsets)

        # ---- 梯队：沿东北方向等距排列 ----
        fmts['echelon'] = center_offsets([[i * s * 0.8 - half * 0.8, i * s * 0.8 - half * 0.8] for i in range(n)])

        # ---- 矩形：3x2 布局 ----
        rect = []
        rect_cols = 3
        rect_rows = max(1, math.ceil(n / rect_cols))
        x_half = (rect_cols - 1) * s / 2.0
        y_half = (rect_rows - 1) * s / 2.0
        for row in range(rect_rows):
            for col in range(rect_cols):
                if len(rect) >= n:
                    break
                rect.append([col * s - x_half, row * s - y_half])
        fmts['rectangle'] = center_offsets(rect)

        # ---- V 型：前尖后展 ----
        v_shape = [[0.0, 2.0 * s]]
        arm_levels = [1, 1, 2, 2, 3]
        arm_sides = [-1, 1, -1, 1, 0]
        for level, side in zip(arm_levels, arm_sides):
            if len(v_shape) >= n:
                break
            if side == 0:
                v_shape.append([0.0, 2.0 * s - level * s])
            else:
                v_shape.append([side * level * 0.9 * s, 2.0 * s - level * s])
        fmts['v_shape'] = center_offsets(v_shape[:n])

        # ---- 原始 2x3 队形 ----
        origin = []
        for row in range(2):
            for col in range(3):
                if len(origin) >= n:
                    break
                origin.append([col * s, row * s])
        fmts['origin_grid'] = center_offsets(origin)

        return fmts

    # ================================================================
    #  回调
    # ================================================================

    def _on_odom(self, drone_id, msg):
        p = msg.pose.pose.position
        self.local_pos[drone_id] = [p.x, p.y, p.z]
        self._try_calibrate(drone_id)

    def _on_gps(self, drone_id, msg):
        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
        if abs(lat) < 1e-3 and abs(lon) < 1e-3:
            return  # 无效 GPS
        self.gps_pos[drone_id] = (lat, lon, alt)

        # 首次收到有效 GPS 时锁定公共 ENU 参考点
        if self._ref is None:
            x0, y0, z0 = _llh_to_ecef(lat, lon, alt)
            self._ref = (lat, lon, x0, y0, z0)
            self.get_logger().info(
                f'公共 ENU 参考 (uav{drone_id}): '
                f'lat={lat:.7f}, lon={lon:.7f}')

        self._try_calibrate(drone_id)

    def _try_calibrate(self, drone_id):
        """当 GPS + odom 都就绪时，计算该机的 home_offset。"""
        if drone_id in self.home_offset:
            return
        if (self._ref is None
                or drone_id not in self.gps_pos
                or drone_id not in self.local_pos):
            return

        lat, lon, alt = self.gps_pos[drone_id]
        ge, gn, _ = _gps_to_enu(lat, lon, alt, self._ref)
        lo = self.local_pos[drone_id]

        # home_offset = 该机在公共 ENU 中的位置 – 该机 odom 位置
        self.home_offset[drone_id] = [ge - lo[0], gn - lo[1]]
        self.get_logger().info(
            f'uav{drone_id} 校准完成: home_offset=('
            f'{self.home_offset[drone_id][0]:+.2f}, '
            f'{self.home_offset[drone_id][1]:+.2f})')

        if len(self.home_offset) == self.num_drones and not self._calibrated:
            self._calibrated = True
            self.get_logger().info(
                f'✓ 全部 {self.num_drones} 架校准完成，可执行队形切换')

    def _on_formation(self, msg):
        name = msg.data.strip().lower()
        if name in ('origin', 'home', 'original', 'origin_grid'):
            self.current_formation = 'origin_grid'
            self._handle_request(('origin', False))
            self.get_logger().info(f'>>> 回原始编队请求 -> {name}')
            return
        if name in ('origin_land', 'home_land', 'return_land', 'original_land'):
            self.current_formation = 'origin_grid'
            self._handle_request(('origin', True))
            self.get_logger().info(f'>>> 回原始编队并降落请求 -> {name}')
            return
        if name not in self.formations:
            self.get_logger().warn(
                f'未知队形 "{name}", 可选: {list(self.formations.keys())} + [origin, origin_land]')
            return
        self.current_formation = name
        self._handle_request(('formation', name))
        self.get_logger().info(f'>>> 队形切换请求 -> {name}')

    def _on_mission_center(self, msg):
        if not self._calibrated:
            self.get_logger().warn('尚未完成校准，忽略 mission_center')
            return
        if self.search_active or self.orbit_active:
            self._bootstrap_offsets_from_current()
        self._clear_search_state()
        self._clear_orbit_state()
        self.mission_center_goal = [float(msg.x), float(msg.y)]
        if self.mission_center_current is None:
            c = self._compute_centroid()
            if c is not None:
                self.mission_center_current = [c[0], c[1]]
        self.mission_active = True
        self.get_logger().info(
            f'>>> 编队前往目标中心 -> ({self.mission_center_goal[0]:.1f}, {self.mission_center_goal[1]:.1f})'
        )

    def _on_search_mission(self, msg):
        if not self._calibrated:
            self.get_logger().warn('尚未完成校准，忽略 search_mission')
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('search_mission JSON 解析失败')
            return

        targets = payload.get('targets', [])
        request_id = payload.get('request_id')
        normalized_targets = []
        for idx, target in enumerate(targets):
            try:
                x = float(target['x'])
                y = float(target['y'])
                weight = max(0.01, float(target.get('weight', 1.0)))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f'search_mission 目标 {idx} 非法，已忽略')
                continue
            normalized_targets.append({
                'id': str(target.get('id', f'target_{idx + 1}')),
                'x': x,
                'y': y,
                'weight': weight,
            })

        if not normalized_targets:
            self.get_logger().warn('search_mission 未提供有效目标点')
            return
        if len(normalized_targets) >= self.num_drones:
            self.get_logger().warn(f'search_mission 目标点数量需小于无人机数量({self.num_drones})')
            return

        self._handle_request(('search', {
            'request_id': request_id,
            'targets': normalized_targets,
        }))
        self.get_logger().info(
            f'>>> 多目标搜索请求 -> {len(normalized_targets)} 个目标点'
        )

    def _on_search_orbit(self, msg):
        if not self._calibrated:
            self.get_logger().warn('尚未完成校准，忽略 search_orbit')
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('search_orbit JSON 解析失败')
            return

        try:
            x = float(payload['x'])
            y = float(payload['y'])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn('search_orbit 目标点非法')
            return

        radius = max(6.0, float(payload.get('radius', self.orbit_radius_default)))
        angular_speed = float(payload.get('angular_speed', self.orbit_angular_speed_default))
        self._handle_request(('orbit', {
            'x': x,
            'y': y,
            'radius': radius,
            'angular_speed': angular_speed,
        }))
        self.get_logger().info(
            f'>>> 盘旋搜索请求 -> center=({x:.1f}, {y:.1f}) radius={radius:.1f} speed={angular_speed:.2f}'
        )

    def _on_clear_task(self, _msg):
        self.get_logger().info('>>> 收到 clear_task，清除当前任务与稳态引导')
        self.phase = 'idle'
        self.pending_request = None
        self.land_after_merge = False
        self.land_sent = set()
        self.mission_active = False
        self.mission_center_goal = None
        self.mission_center_current = None
        self.reconfig_has_layers = False
        self._set_mission_speed_mode('cruise')
        self._clear_search_state()
        self._clear_orbit_state()
        self._clear_motion_targets()
        self._reset_phase_hold()

    # ================================================================
    #  目标计算（公共 ENU 坐标系）
    # ================================================================

    def _global_xy(self, did):
        if did not in self.local_pos or did not in self.home_offset:
            return None
        lo = self.local_pos[did]
        ho = self.home_offset[did]
        return [lo[0] + ho[0], lo[1] + ho[1]]

    def _local_xy_from_global(self, did, gx, gy):
        ho = self.home_offset[did]
        return [gx - ho[0], gy - ho[1]]

    def _all_airborne(self):
        for did in range(1, self.num_drones + 1):
            if did not in self.local_pos:
                return False
            if self.local_pos[did][2] < self.min_takeoff_z:
                return False
        return True

    def _compute_centroid(self):
        xs, ys = [], []
        for did in range(1, self.num_drones + 1):
            g = self._global_xy(did)
            if g is None:
                return None
            xs.append(g[0])
            ys.append(g[1])
        return sum(xs) / len(xs), sum(ys) / len(ys)

    def _assign_targets(self, drone_ids, cur_xy, target_xy):
        n = len(drone_ids)
        best = None
        best_cost = float('inf')

        if n <= 8:
            for perm in itertools.permutations(range(n)):
                cost = 0.0
                assigned_targets = []
                for i in range(n):
                    dx = cur_xy[i][0] - target_xy[perm[i]][0]
                    dy = cur_xy[i][1] - target_xy[perm[i]][1]
                    cost += dx * dx + dy * dy
                    assigned_targets.append(target_xy[perm[i]])
                conflicts = self._count_path_conflicts(cur_xy, assigned_targets)
                cost += conflicts * self.path_conflict_penalty
                if cost < best_cost:
                    best_cost = cost
                    best = perm
        else:
            # Greedy fallback for larger swarms.
            rest = set(range(n))
            out = []
            for i in range(n):
                pick = min(
                    rest,
                    key=lambda j: (cur_xy[i][0] - target_xy[j][0]) ** 2
                    + (cur_xy[i][1] - target_xy[j][1]) ** 2,
                )
                rest.remove(pick)
                out.append(pick)
            best = tuple(out)

        mapping = {}
        for i, did in enumerate(drone_ids):
            mapping[did] = target_xy[best[i]]
        return mapping

    def _point_to_segment_distance_sq(self, px, py, ax, ay, bx, by):
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 1e-9:
            dx = px - ax
            dy = py - ay
            return dx * dx + dy * dy
        t = ((px - ax) * abx + (py - ay) * aby) / ab2
        t = max(0.0, min(1.0, t))
        qx = ax + t * abx
        qy = ay + t * aby
        dx = px - qx
        dy = py - qy
        return dx * dx + dy * dy

    def _segments_intersect(self, a0, a1, b0, b1):
        def orient(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

        def on_segment(p, q, r):
            return (
                min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
                and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9
            )

        o1 = orient(a0, a1, b0)
        o2 = orient(a0, a1, b1)
        o3 = orient(b0, b1, a0)
        o4 = orient(b0, b1, a1)

        if ((o1 > 0 and o2 < 0) or (o1 < 0 and o2 > 0)) and ((o3 > 0 and o4 < 0) or (o3 < 0 and o4 > 0)):
            return True

        if abs(o1) <= 1e-9 and on_segment(a0, b0, a1):
            return True
        if abs(o2) <= 1e-9 and on_segment(a0, b1, a1):
            return True
        if abs(o3) <= 1e-9 and on_segment(b0, a0, b1):
            return True
        if abs(o4) <= 1e-9 and on_segment(b0, a1, b1):
            return True
        return False

    def _path_conflict(self, a_start, a_target, b_start, b_target):
        clearance_sq = self.path_conflict_distance * self.path_conflict_distance
        if self._segments_intersect(a_start, a_target, b_start, b_target):
            return True
        dists = [
            self._point_to_segment_distance_sq(a_start[0], a_start[1], b_start[0], b_start[1], b_target[0], b_target[1]),
            self._point_to_segment_distance_sq(a_target[0], a_target[1], b_start[0], b_start[1], b_target[0], b_target[1]),
            self._point_to_segment_distance_sq(b_start[0], b_start[1], a_start[0], a_start[1], a_target[0], a_target[1]),
            self._point_to_segment_distance_sq(b_target[0], b_target[1], a_start[0], a_start[1], a_target[0], a_target[1]),
        ]
        return min(dists) < clearance_sq

    def _count_path_conflicts(self, cur_xy, assigned_targets):
        conflicts = 0
        for i in range(len(cur_xy)):
            for j in range(i + 1, len(cur_xy)):
                if self._path_conflict(cur_xy[i], assigned_targets[i], cur_xy[j], assigned_targets[j]):
                    conflicts += 1
        return conflicts

    def _group_offsets_for_count(self, count, spacing):
        if count <= 0:
            return []
        if count == 1:
            return [[0.0, 0.0]]
        if count == 2:
            half = spacing / 2.0
            return [[-half, 0.0], [half, 0.0]]
        if count == 3:
            return [
                [0.0, spacing * 0.6],
                [-spacing * 0.55, -spacing * 0.35],
                [spacing * 0.55, -spacing * 0.35],
            ]
        if count == 4:
            half = spacing / 2.0
            return [
                [-half, half],
                [half, half],
                [-half, -half],
                [half, -half],
            ]
        if count == 5:
            return [
                [0.0, 0.0],
                [-spacing, 0.0],
                [spacing, 0.0],
                [0.0, spacing],
                [0.0, -spacing],
            ]
        radius = max(spacing, spacing * 0.95)
        offsets = []
        for idx in range(count):
            angle = 2.0 * math.pi * idx / count
            offsets.append([radius * math.cos(angle), radius * math.sin(angle)])
        return offsets

    def _weighted_target_counts(self, targets):
        if not targets:
            return {}
        counts = {target['id']: 1 for target in targets}
        remaining = self.num_drones - len(targets)
        if remaining <= 0:
            return counts

        total_weight = sum(target['weight'] for target in targets)
        ideals = {}
        for target in targets:
            share = remaining * target['weight'] / max(total_weight, 1e-6)
            ideals[target['id']] = share

        for target in targets:
            extra = int(math.floor(ideals[target['id']]))
            counts[target['id']] += extra
            remaining -= extra

        if remaining > 0:
            ranked = sorted(
                targets,
                key=lambda target: (ideals[target['id']] - math.floor(ideals[target['id']]), target['weight']),
                reverse=True,
            )
            for idx in range(remaining):
                counts[ranked[idx % len(ranked)]['id']] += 1
        return counts

    def _build_search_slots(self, targets, counts):
        slots = []
        groups = {}
        for target in targets:
            target_id = target['id']
            count = counts[target_id]
            offsets = self._group_offsets_for_count(count, self.search_group_spacing)
            groups[target_id] = {
                'target': [target['x'], target['y']],
                'weight': target['weight'],
                'count': count,
                'slots': [],
            }
            for offset in offsets:
                slot_xy = [target['x'] + offset[0], target['y'] + offset[1]]
                groups[target_id]['slots'].append(slot_xy)
                slots.append((target_id, slot_xy))
        return slots, groups

    def _build_orbit_slots(self, center_x, center_y, radius):
        slots = []
        for idx in range(self.num_drones):
            angle = math.pi / 2.0 + 2.0 * math.pi * idx / self.num_drones
            slots.append((angle, [
                center_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle),
            ]))
        return slots

    def _current_search_start_xy_map(self, drone_ids):
        start_xy_map = {}
        snap_count = 0
        for did in drone_ids:
            current_xy = self._global_xy(did)
            if current_xy is None:
                continue
            snapped_xy = current_xy
            planned_xy, tolerance = self._current_mode_anchor_xy(did, self._get_reference_center())
            if planned_xy is not None:
                if math.hypot(current_xy[0] - planned_xy[0], current_xy[1] - planned_xy[1]) <= tolerance:
                    snapped_xy = [planned_xy[0], planned_xy[1]]
                    snap_count += 1
            start_xy_map[did] = snapped_xy
        return start_xy_map, snap_count

    def _current_mode_anchor_xy(self, did, center=None):
        if did in self.steady_global_xy:
            return [self.steady_global_xy[did][0], self.steady_global_xy[did][1]], self.search_snap_tolerance
        if did in self.target_global_xy:
            return [self.target_global_xy[did][0], self.target_global_xy[did][1]], self.search_snap_tolerance
        if center is None:
            center = self._get_reference_center()
        if center is not None and did in self.active_offset_xy:
            return [center[0] + self.active_offset_xy[did][0], center[1] + self.active_offset_xy[did][1]], self.planning_snap_tolerance
        if did in self.active_global_xy:
            return [self.active_global_xy[did][0], self.active_global_xy[did][1]], self.planning_snap_tolerance
        return None, self.planning_snap_tolerance

    def _clear_search_state(self):
        self.search_active = False
        self.search_targets = []
        self.search_assignments = {}
        self.search_target_counts = {}
        self.search_groups = {}
        self.steady_global_xy = {}

    def _clear_orbit_state(self):
        self.orbit_active = False
        self.orbit_pending_spec = None
        self.orbit_center = None
        self.orbit_radius = self.orbit_radius_default
        self.orbit_angular_speed = self.orbit_angular_speed_default
        self.orbit_phase_map = {}
        self.orbit_start_time = 0.0

    def _clear_motion_targets(self):
        self.active_global_xy = {}
        self.target_offset_xy = {}
        self.target_global_xy = {}
        self.phase_start_offset_xy = {}
        self.phase_start_global_xy = {}
        self.layer_z = {}
        self.lift_xy_local = {}

    def _target_global_xy_for_phase(self, did, use_start=False):
        if use_start:
            if did in self.phase_start_global_xy:
                return self.phase_start_global_xy[did]
            return self._target_global_xy(did, self.phase_start_offset_xy)
        if did in self.target_global_xy:
            return self.target_global_xy[did]
        return self._target_global_xy(did, self.target_offset_xy)

    def _planning_start_xy_map(self, drone_ids, center):
        start_xy_map = {}
        snap_count = 0
        if center is None:
            return start_xy_map, snap_count
        for did in drone_ids:
            current_xy = self._global_xy(did)
            if current_xy is None:
                continue
            snapped_xy = current_xy
            planned_xy, tolerance = self._current_mode_anchor_xy(did, center)
            if planned_xy is not None:
                if math.hypot(current_xy[0] - planned_xy[0], current_xy[1] - planned_xy[1]) <= tolerance:
                    snapped_xy = planned_xy
                    snap_count += 1
            start_xy_map[did] = snapped_xy
        return start_xy_map, snap_count

    def _build_conflict_graph(self, drone_ids, start_xy_map, target_xy_map):
        graph = {did: set() for did in drone_ids}
        for idx, did_a in enumerate(drone_ids):
            start_a = start_xy_map.get(did_a)
            target_a = target_xy_map[did_a]
            for did_b in drone_ids[idx + 1:]:
                start_b = start_xy_map.get(did_b)
                target_b = target_xy_map[did_b]
                if start_a is None or start_b is None:
                    continue
                if self._path_conflict(start_a, target_a, start_b, target_b):
                    graph[did_a].add(did_b)
                    graph[did_b].add(did_a)
        return graph

    def _build_layer_plan(self, drone_ids, start_xy_map, target_xy_map):
        graph = self._build_conflict_graph(drone_ids, start_xy_map, target_xy_map)
        coloring = {}
        order = sorted(
            drone_ids,
            key=lambda did: (len(graph[did]), (target_xy_map[did][0] - start_xy_map[did][0]) ** 2 + (target_xy_map[did][1] - start_xy_map[did][1]) ** 2),
            reverse=True,
        )
        for did in order:
            used = {coloring[nbr] for nbr in graph[did] if nbr in coloring}
            color = 0
            while color in used:
                color += 1
            coloring[did] = color

        layer_z = {}
        has_layers = False
        for did in drone_ids:
            color = coloring.get(did, 0)
            if color == 0:
                desired = self.target_alt
            else:
                desired = self.target_alt + self.layer_base_offset + (color - 1) * self.layer_spacing
                has_layers = True
            layer_z[did] = max(self.local_pos[did][2], desired)
        return layer_z, has_layers

    def _handle_request(self, request):
        request_type = request[0]
        if not self._calibrated:
            self.get_logger().warn('尚未完成 GPS/odom 校准，无法执行请求')
            return
        if not self._all_airborne():
            self.get_logger().warn('存在未起飞无人机，暂不执行请求')
            return

        if request_type == 'formation' and self.mission_active:
            self._set_mission_speed_mode('reconfig')
        if request_type == 'search':
            self._set_mission_speed_mode('cruise')
        if request_type == 'orbit':
            self._set_mission_speed_mode('cruise')

        # If we are already merging back to nominal altitude, queue the next request and execute it immediately after merge.
        if self.phase in ('merge', 'land'):
            self.pending_request = request
            self.get_logger().info(f'已排队新请求，等待当前阶段 {self.phase} 完成后执行')
            return

        if request_type == 'origin':
            self._start_return_to_origin(do_land=request[1])
        elif request_type == 'search':
            self.land_after_merge = False
            self._start_search_mission(request[1])
        elif request_type == 'orbit':
            self.land_after_merge = False
            self._start_search_orbit(request[1])
        else:
            self.land_after_merge = False
            self._start_reconfiguration(request[1])

    def _bootstrap_offsets_from_current(self):
        c = self._compute_centroid()
        if c is None:
            return False
        self.active_offset_xy = {}
        for did in range(1, self.num_drones + 1):
            g = self._global_xy(did)
            self.active_offset_xy[did] = [g[0] - c[0], g[1] - c[1]]
        self.last_plan_center = [c[0], c[1]]
        if self.mission_center_current is None:
            self.mission_center_current = [c[0], c[1]]
        return True

    def _get_reference_center(self):
        if self.mission_active and self.mission_center_current is not None:
            return [self.mission_center_current[0], self.mission_center_current[1]]
        if self.last_plan_center is not None:
            return [self.last_plan_center[0], self.last_plan_center[1]]
        c = self._compute_centroid()
        if c is None:
            return None
        return [c[0], c[1]]

    def _target_global_xy(self, did, offset_map):
        center = self._get_reference_center()
        if center is None or did not in offset_map:
            return None
        return [center[0] + offset_map[did][0], center[1] + offset_map[did][1]]

    def _refresh_active_global_xy(self):
        center = self._get_reference_center()
        if center is None:
            return
        self.last_plan_center = [center[0], center[1]]
        self.active_global_xy = {
            did: [center[0] + self.target_offset_xy[did][0], center[1] + self.target_offset_xy[did][1]]
            for did in self.target_offset_xy
        }

    def _reset_phase_hold(self):
        self.phase_hold = {did: 0 for did in range(1, self.num_drones + 1)}

    def _set_mission_speed_mode(self, mode):
        if mode not in ('cruise', 'reconfig'):
            return
        if self.mission_speed_mode == mode:
            return
        self.mission_speed_mode = mode
        if self.mission_active:
            if mode == 'reconfig':
                self.get_logger().info('>>> mission 进入变阵优先慢速')
            else:
                self.get_logger().info('>>> 队形完成，mission 恢复巡航速度')

    def _mission_step_limit(self):
        step_limit = self.mission_step_xy
        if not self.mission_active:
            return step_limit
        if self.mission_speed_mode == 'reconfig':
            scale = self.mission_step_xy_reconfig_scale
            if self.pending_request is not None and self.phase in ('merge', 'land'):
                scale = min(scale, self.mission_step_xy_queue_scale)
            if self.phase == 'xy':
                scale = min(scale, self.mission_step_xy_xy_scale)
            return step_limit * max(0.05, min(1.0, scale))
        return step_limit

    def _enter_phase(self, name):
        self.phase = name
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9
        self._reset_phase_hold()
        self.get_logger().info(f'>>> 进入阶段: {name}')

    def _start_reconfiguration(self, formation_name):
        if not self._calibrated:
            self.get_logger().warn('尚未完成 GPS/odom 校准，无法切换队形')
            return
        if not self._all_airborne():
            self.get_logger().warn('存在未起飞无人机，暂不执行队形切换')
            return

        c = self._get_reference_center()
        if c is None:
            self.get_logger().warn('无法计算质心，等待更多状态')
            return

        offsets = self.formations[formation_name]
        drone_ids = list(range(1, self.num_drones + 1))
        actual_cur_xy = [self._global_xy(did) for did in drone_ids]
        start_xy_map, snap_count = self._planning_start_xy_map(drone_ids, c)
        cur_xy = [start_xy_map.get(did, actual_cur_xy[idx]) for idx, did in enumerate(drone_ids)]
        target_xy = [[c[0] + o[0], c[1] + o[1]] for o in offsets[: self.num_drones]]
        self._clear_search_state()
        self._clear_orbit_state()
        self.active_global_xy = self._assign_targets(drone_ids, cur_xy, target_xy)
        self.last_plan_center = [c[0], c[1]]
        self.target_global_xy = {}
        self.phase_start_global_xy = {}
        self.target_offset_xy = {
            did: [self.active_global_xy[did][0] - c[0], self.active_global_xy[did][1] - c[1]]
            for did in drone_ids
        }
        self.phase_start_offset_xy = {
            did: [actual_cur_xy[idx][0] - c[0], actual_cur_xy[idx][1] - c[1]]
            for idx, did in enumerate(drone_ids)
        }

        self.layer_z, self.reconfig_has_layers = self._build_layer_plan(drone_ids, start_xy_map, self.active_global_xy)

        self.lift_xy_local = {
            did: [self.local_pos[did][0], self.local_pos[did][1]] for did in drone_ids
        }
        self.pending_request = None

        self.get_logger().info(
            f'规划完成: centroid=({c[0]:.2f},{c[1]:.2f}) formation={formation_name}'
        )
        if snap_count > 0:
            self.get_logger().info(f'  规划起点吸附: {snap_count}/{len(drone_ids)} 架在容差内按当前编队槽位规划')
        for did in drone_ids:
            g = self.active_global_xy[did]
            lxy = self._local_xy_from_global(did, g[0], g[1])
            self.get_logger().info(
                f'  uav{did}: target_global=({g[0]:+.1f},{g[1]:+.1f}) '
                f'target_local=({lxy[0]:+.1f},{lxy[1]:+.1f}) layer_z={self.layer_z[did]:.1f}'
            )

        if self.reconfig_has_layers:
            self._enter_phase('lift')
        else:
            self.get_logger().info('规划结果无路径冲突，跳过 lift，直接进入 xy')
            self._enter_phase('xy')

    def _start_search_mission(self, mission_spec):
        if not self._calibrated:
            self.get_logger().warn('尚未完成 GPS/odom 校准，无法执行多目标搜索')
            return
        if not self._all_airborne():
            self.get_logger().warn('存在未起飞无人机，暂不执行多目标搜索')
            return

        targets = mission_spec['targets']
        drone_ids = list(range(1, self.num_drones + 1))
        counts = self._weighted_target_counts(targets)
        slots, groups = self._build_search_slots(targets, counts)
        if len(slots) != self.num_drones:
            self.get_logger().warn('多目标搜索槽位数量与无人机数量不一致，取消本次任务')
            return

        self.mission_active = False
        self.mission_center_goal = None
        self.mission_center_current = None
        self.current_formation = 'search'

        start_xy_map, snap_count = self._current_search_start_xy_map(drone_ids)
        cur_xy = [start_xy_map.get(did, self._global_xy(did)) for did in drone_ids]
        target_xy = [slot_xy for _, slot_xy in slots]
        assigned = self._assign_targets(drone_ids, cur_xy, target_xy)
        assigned_target_ids = {}
        for did in drone_ids:
            assigned_xy = assigned[did]
            matched_target_id = None
            for target_id, slot_xy in slots:
                if math.hypot(assigned_xy[0] - slot_xy[0], assigned_xy[1] - slot_xy[1]) <= 1e-6:
                    matched_target_id = target_id
                    break
            assigned_target_ids[did] = matched_target_id

        self.target_global_xy = {
            did: [assigned[did][0], assigned[did][1]]
            for did in drone_ids
        }
        self.phase_start_global_xy = {
            did: [start_xy_map.get(did, self._global_xy(did))[0], start_xy_map.get(did, self._global_xy(did))[1]]
            for did in drone_ids
        }
        self.target_offset_xy = {}
        self.phase_start_offset_xy = {}
        self.last_plan_center = self._compute_centroid()
        self.layer_z, self.reconfig_has_layers = self._build_layer_plan(drone_ids, start_xy_map, self.target_global_xy)
        self.pending_request = None
        self._clear_orbit_state()
        self.search_active = True
        self.search_targets = targets
        self.search_assignments = assigned_target_ids
        self.search_target_counts = counts
        self.search_groups = groups

        self.get_logger().info('规划完成: 多目标搜索')
        if snap_count > 0:
            self.get_logger().info(f'  搜索规划起点吸附: {snap_count}/{len(drone_ids)} 架使用稳态搜索槽位')
        for target in targets:
            target_id = target['id']
            members = [did for did in drone_ids if assigned_target_ids[did] == target_id]
            self.get_logger().info(
                f'  {target_id}: goal=({target["x"]:+.1f},{target["y"]:+.1f}) weight={target["weight"]:.2f} count={counts[target_id]} members={members}'
            )
        for did in drone_ids:
            g = self.target_global_xy[did]
            lxy = self._local_xy_from_global(did, g[0], g[1])
            self.get_logger().info(
                f'  uav{did}: search_target=({g[0]:+.1f},{g[1]:+.1f}) '
                f'target_local=({lxy[0]:+.1f},{lxy[1]:+.1f}) layer_z={self.layer_z[did]:.1f} target_id={assigned_target_ids[did]}'
            )

        if self.reconfig_has_layers:
            self._enter_phase('lift')
        else:
            self.get_logger().info('多目标搜索规划无路径冲突，跳过 lift，直接进入 xy')
            self._enter_phase('xy')

    def _start_search_orbit(self, orbit_spec):
        if not self._calibrated:
            self.get_logger().warn('尚未完成 GPS/odom 校准，无法执行盘旋搜索')
            return
        if not self._all_airborne():
            self.get_logger().warn('存在未起飞无人机，暂不执行盘旋搜索')
            return
        if not self.active_offset_xy and not self._bootstrap_offsets_from_current():
            self.get_logger().warn('无法获取当前编队状态，暂不执行盘旋搜索')
            return

        current_center = self._compute_centroid()
        if current_center is None:
            self.get_logger().warn('无法计算当前编队中心，暂不执行盘旋搜索')
            return

        self._clear_search_state()
        self._clear_orbit_state()
        self.target_global_xy = {}
        self.phase_start_global_xy = {}
        self.target_offset_xy = {
            did: [self.active_offset_xy[did][0], self.active_offset_xy[did][1]]
            for did in self.active_offset_xy
        }
        self.phase_start_offset_xy = {}
        for did in range(1, self.num_drones + 1):
            gxy = self._global_xy(did)
            if gxy is None:
                continue
            self.phase_start_offset_xy[did] = [
                gxy[0] - current_center[0],
                gxy[1] - current_center[1],
            ]
        self.last_plan_center = [current_center[0], current_center[1]]
        self.orbit_pending_spec = {
            'x': float(orbit_spec['x']),
            'y': float(orbit_spec['y']),
            'radius': max(6.0, float(orbit_spec.get('radius', self.orbit_radius_default))),
            'angular_speed': float(orbit_spec.get('angular_speed', self.orbit_angular_speed_default)),
        }
        self.mission_center_goal = [self.orbit_pending_spec['x'], self.orbit_pending_spec['y']]
        if self.mission_center_current is None:
            c = self._compute_centroid()
            if c is not None:
                self.mission_center_current = [c[0], c[1]]
        self.mission_active = True
        self.get_logger().info(
            f'>>> 编队前往盘旋搜索点 -> ({self.orbit_pending_spec["x"]:.1f}, {self.orbit_pending_spec["y"]:.1f})'
        )

    def _activate_orbit_pattern(self, orbit_spec):
        drone_ids = list(range(1, self.num_drones + 1))
        start_xy_map, snap_count = self._current_search_start_xy_map(drone_ids)
        cur_xy = [start_xy_map.get(did, self._global_xy(did)) for did in drone_ids]
        orbit_slots = self._build_orbit_slots(orbit_spec['x'], orbit_spec['y'], orbit_spec['radius'])
        target_xy = [slot_xy for _, slot_xy in orbit_slots]
        assigned = self._assign_targets(drone_ids, cur_xy, target_xy)

        phase_map = {}
        for did in drone_ids:
            assigned_xy = assigned[did]
            matched_angle = orbit_slots[0][0]
            for angle, slot_xy in orbit_slots:
                if math.hypot(assigned_xy[0] - slot_xy[0], assigned_xy[1] - slot_xy[1]) <= 1e-6:
                    matched_angle = angle
                    break
            phase_map[did] = matched_angle

        self._clear_search_state()
        self._clear_orbit_state()
        self.current_formation = 'orbit'
        self.target_global_xy = {
            did: [assigned[did][0], assigned[did][1]]
            for did in drone_ids
        }
        self.phase_start_global_xy = {
            did: [start_xy_map.get(did, self._global_xy(did))[0], start_xy_map.get(did, self._global_xy(did))[1]]
            for did in drone_ids
        }
        self.target_offset_xy = {}
        self.phase_start_offset_xy = {}
        self.last_plan_center = [orbit_spec['x'], orbit_spec['y']]
        self.layer_z, self.reconfig_has_layers = self._build_layer_plan(drone_ids, start_xy_map, self.target_global_xy)
        self.pending_request = None
        self.orbit_active = True
        self.orbit_center = [orbit_spec['x'], orbit_spec['y']]
        self.orbit_radius = orbit_spec['radius']
        self.orbit_angular_speed = orbit_spec['angular_speed']
        self.orbit_phase_map = phase_map
        self.orbit_start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(
            f'规划完成: 盘旋搜索 center=({orbit_spec["x"]:.2f},{orbit_spec["y"]:.2f}) radius={orbit_spec["radius"]:.1f}'
        )
        if snap_count > 0:
            self.get_logger().info(f'  盘旋规划起点吸附: {snap_count}/{len(drone_ids)} 架复用当前状态')
        for did in drone_ids:
            g = self.target_global_xy[did]
            lxy = self._local_xy_from_global(did, g[0], g[1])
            self.get_logger().info(
                f'  uav{did}: orbit_target=({g[0]:+.1f},{g[1]:+.1f}) '
                f'target_local=({lxy[0]:+.1f},{lxy[1]:+.1f}) layer_z={self.layer_z[did]:.1f}'
            )

        if self.reconfig_has_layers:
            self._enter_phase('lift')
        else:
            self.get_logger().info('盘旋搜索规划无路径冲突，跳过 lift，直接进入 xy')
            self._enter_phase('xy')

    def _within(self, did, target_local_xy, target_z, check_xy=True):
        lo = self.local_pos[did]
        if check_xy:
            dxy = math.hypot(target_local_xy[0] - lo[0], target_local_xy[1] - lo[1])
            if dxy > self.xy_tolerance:
                return False
        return abs(target_z - lo[2]) <= self.z_tolerance

    def _within_relative_shape(self, did, target_offset_xy, target_z):
        center = self._compute_centroid()
        current_global = self._global_xy(did)
        if center is None or current_global is None:
            return False
        current_offset = [current_global[0] - center[0], current_global[1] - center[1]]
        dxy = math.hypot(current_offset[0] - target_offset_xy[0], current_offset[1] - target_offset_xy[1])
        if dxy > self.xy_tolerance_reconfig:
            return False
        return abs(target_z - self.local_pos[did][2]) <= self.z_tolerance_reconfig

    def _update_hold(self, did, reached):
        if reached:
            self.phase_hold[did] = min(self.phase_hold.get(did, 0) + 1, self.hold_cycles)
        else:
            self.phase_hold[did] = 0

    def _phase_all_reached(self):
        for did in range(1, self.num_drones + 1):
            if self.phase_hold.get(did, 0) < self.hold_cycles:
                return False
        return True

    def _phase_timeout(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.phase_start_time
        if self.phase == 'lift':
            return elapsed > self.phase_timeout_lift
        if self.phase == 'xy':
            return elapsed > self.phase_timeout_xy
        if self.phase == 'merge':
            return elapsed > self.phase_timeout_merge
        if self.phase == 'land':
            return elapsed > self.phase_timeout_land
        return False

    def _start_return_to_origin(self, do_land):
        if not self._calibrated:
            self.get_logger().warn('尚未完成 GPS/odom 校准，无法回原始编队')
            return
        if not self._all_airborne():
            self.get_logger().warn('存在未起飞无人机，暂不执行回原始编队')
            return
        self.mission_active = False
        self.mission_center_goal = None
        self.target_global_xy = {}
        self.phase_start_global_xy = {}

        self.active_global_xy = {
            did: [self.home_offset[did][0], self.home_offset[did][1]]
            for did in range(1, self.num_drones + 1)
        }
        self.last_plan_center = [
            sum(self.active_global_xy[did][0] for did in range(1, self.num_drones + 1)) / self.num_drones,
            sum(self.active_global_xy[did][1] for did in range(1, self.num_drones + 1)) / self.num_drones,
        ]
        start_xy_map, _ = self._planning_start_xy_map(
            list(range(1, self.num_drones + 1)),
            self.last_plan_center,
        )
        self._clear_search_state()
        self.layer_z, self.reconfig_has_layers = self._build_layer_plan(
            list(range(1, self.num_drones + 1)),
            start_xy_map,
            self.active_global_xy,
        )
        self.target_offset_xy = {
            did: [
                self.active_global_xy[did][0] - self.last_plan_center[0],
                self.active_global_xy[did][1] - self.last_plan_center[1],
            ]
            for did in range(1, self.num_drones + 1)
        }
        self.phase_start_offset_xy = {
            did: [
                self._global_xy(did)[0] - self.last_plan_center[0],
                self._global_xy(did)[1] - self.last_plan_center[1],
            ]
            for did in range(1, self.num_drones + 1)
        }
        self.lift_xy_local = {
            did: [self.local_pos[did][0], self.local_pos[did][1]]
            for did in range(1, self.num_drones + 1)
        }
        self.land_after_merge = do_land
        self.land_sent = set()
        self.pending_request = None
        self.get_logger().info('规划完成: 返回原始编队')
        for did in range(1, self.num_drones + 1):
            g = self.active_global_xy[did]
            lxy = self._local_xy_from_global(did, g[0], g[1])
            self.get_logger().info(
                f'  uav{did}: home_global=({g[0]:+.1f},{g[1]:+.1f}) '
                f'home_local=({lxy[0]:+.1f},{lxy[1]:+.1f}) layer_z={self.layer_z[did]:.1f}'
            )
        if self.reconfig_has_layers:
            self._enter_phase('lift')
        else:
            self.get_logger().info('返回原始编队无路径冲突，跳过 lift，直接进入 xy')
            self._enter_phase('xy')

    def _all_landed(self):
        for did in range(1, self.num_drones + 1):
            if did not in self.local_pos:
                return False
            if self.local_pos[did][2] > 0.25:
                return False
        return True

    def _publish_land(self, did):
        self.land_pubs[did].publish(Empty())

    def _publish_status(self):
        status = {
            'phase': self.phase,
            'formation': self.current_formation,
            'mission_active': self.mission_active,
            'mission_speed_mode': self.mission_speed_mode,
            'reconfig_has_layers': self.reconfig_has_layers,
            'mission_goal': self.mission_center_goal,
            'mission_center': self.mission_center_current,
            'search_active': self.search_active,
            'search_targets': self.search_targets,
            'search_assignments': self.search_assignments,
            'search_target_counts': self.search_target_counts,
            'orbit_active': self.orbit_active,
            'orbit_pending': self.orbit_pending_spec,
            'orbit_center': self.orbit_center,
            'orbit_radius': self.orbit_radius,
            'orbit_angular_speed': self.orbit_angular_speed,
            'queued_request': self.pending_request,
            'land_after_merge': self.land_after_merge,
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True)
        self.status_pub.publish(msg)

    def _update_mission_center(self):
        if not self.mission_active or self.mission_center_goal is None:
            return
        if self.mission_center_current is None:
            c = self._compute_centroid()
            if c is None:
                return
            self.mission_center_current = [c[0], c[1]]
        dx = self.mission_center_goal[0] - self.mission_center_current[0]
        dy = self.mission_center_goal[1] - self.mission_center_current[1]
        dist = math.hypot(dx, dy)
        if dist <= self.mission_tolerance:
            self.mission_center_current = [self.mission_center_goal[0], self.mission_center_goal[1]]
            self.mission_active = False
            self._set_mission_speed_mode('cruise')
            self.get_logger().info('>>> 编队到达目标中心')
            return
        step_limit = self._mission_step_limit()
        step = min(step_limit, dist)
        self.mission_center_current[0] += dx * step / max(dist, 1e-6)
        self.mission_center_current[1] += dy * step / max(dist, 1e-6)
        if step >= dist - 1e-6:
            self.mission_center_current = [self.mission_center_goal[0], self.mission_center_goal[1]]

    def _publish_orbit_targets(self):
        if not self.orbit_active or self.orbit_center is None or not self.orbit_phase_map:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        theta = self.orbit_angular_speed * max(0.0, now - self.orbit_start_time)
        for did in range(1, self.num_drones + 1):
            if did not in self.orbit_phase_map or did not in self.home_offset or did not in self.local_pos:
                continue
            angle = self.orbit_phase_map[did] + theta
            gx = self.orbit_center[0] + self.orbit_radius * math.cos(angle)
            gy = self.orbit_center[1] + self.orbit_radius * math.sin(angle)
            self.steady_global_xy[did] = [gx, gy]
            target_lxy = self._local_xy_from_global(did, gx, gy)
            self._publish_guided_target(did, target_lxy, self.target_alt)

    def _publish_mission_targets(self):
        if self.mission_center_current is None or not self.active_offset_xy:
            return
        for did in range(1, self.num_drones + 1):
            if did not in self.active_offset_xy or did not in self.home_offset or did not in self.local_pos:
                continue
            gx = self.mission_center_current[0] + self.active_offset_xy[did][0]
            gy = self.mission_center_current[1] + self.active_offset_xy[did][1]
            target_lxy = self._local_xy_from_global(did, gx, gy)
            self._publish_guided_target(did, target_lxy, self.target_alt)

    def _publish_search_targets(self):
        if not self.steady_global_xy:
            return
        for did in range(1, self.num_drones + 1):
            target_gxy = self.steady_global_xy.get(did)
            if target_gxy is None or did not in self.home_offset or did not in self.local_pos:
                continue
            target_lxy = self._local_xy_from_global(did, target_gxy[0], target_gxy[1])
            self._publish_guided_target(did, target_lxy, self.target_alt)

    def _publish_guided_target(self, did, target_local_xy, target_z, max_step_xy=None, max_step_z=None):
        if self._ref is None:
            return

        if max_step_xy is None:
            max_step_xy = self.max_step_xy
        if max_step_z is None:
            max_step_z = self.max_step_z

        lo = self.local_pos[did]
        dx = target_local_xy[0] - lo[0]
        dy = target_local_xy[1] - lo[1]
        dz = target_z - lo[2]

        cmd_x = target_local_xy[0]
        cmd_y = target_local_xy[1]
        cmd_z = target_z

        dxy = math.hypot(dx, dy)
        if dxy > max_step_xy and dxy > 1e-6:
            scale = max_step_xy / dxy
            cmd_x = lo[0] + dx * scale
            cmd_y = lo[1] + dy * scale

        if abs(dz) > max_step_z:
            cmd_z = lo[2] + (max_step_z if dz > 0.0 else -max_step_z)

        # Convert local command point to shared global ENU, then to LLH for waypoint command.
        ho = self.home_offset[did]
        cmd_gx = cmd_x + ho[0]
        cmd_gy = cmd_y + ho[1]
        lat, lon, _ = _enu_to_llh(cmd_gx, cmd_gy, 0.0, self._ref)

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        # mavlink_tx uses GLOBAL_RELATIVE_ALT_INT by default; altitude is relative to home.
        msg.altitude = cmd_z
        self.waypoint_pubs[did].publish(msg)

    # ================================================================
    #  控制循环
    # ================================================================

    def _control_loop(self):
        self._publish_status()

        if not self._calibrated:
            return

        if self.mission_active:
            self._update_mission_center()
            if self.phase != 'idle' and self.target_offset_xy:
                self._refresh_active_global_xy()
            if not self.mission_active and self.orbit_pending_spec is not None and self.phase == 'idle':
                spec = dict(self.orbit_pending_spec)
                self.orbit_pending_spec = None
                self.get_logger().info('>>> 到达盘旋搜索点，开始建立盘旋编队')
                self._activate_orbit_pattern(spec)

        if self.auto_start_default and not self._default_started and self._all_airborne():
            self._default_started = True
            self._start_reconfiguration(self.current_formation)

        if self.phase == 'idle':
            if not self._all_airborne():
                if (
                    self.mission_active
                    or self.search_active
                    or self.orbit_active
                    or self.orbit_pending_spec is not None
                    or self.target_global_xy
                    or self.target_offset_xy
                    or self.steady_global_xy
                ):
                    self.get_logger().info('检测到非全机空中状态，清除悬挂任务与稳态引导')
                    self.mission_active = False
                    self.mission_center_goal = None
                    self.mission_center_current = None
                    self.pending_request = None
                    self.land_after_merge = False
                    self.reconfig_has_layers = False
                    self._set_mission_speed_mode('cruise')
                    self._clear_search_state()
                    self._clear_orbit_state()
                    self._clear_motion_targets()
                return
            if self.mission_active:
                if not self.active_offset_xy and not self._bootstrap_offsets_from_current():
                    return
                self._publish_mission_targets()
            elif self.orbit_active:
                self._publish_orbit_targets()
            elif self.search_active:
                self._publish_search_targets()
            return

        if self.phase != 'land' and not self._all_airborne():
            self.get_logger().warn('机群高度状态异常，终止当前队形切换并回到 idle')
            self.phase = 'idle'
            self.active_global_xy = {}
            self.target_offset_xy = {}
            self.phase_start_offset_xy = {}
            self.layer_z = {}
            self.lift_xy_local = {}
            self.reconfig_has_layers = False
            self._set_mission_speed_mode('cruise')
            self._reset_phase_hold()
            return

        for did in range(1, self.num_drones + 1):
            if did not in self.local_pos or did not in self.home_offset:
                continue

            lo = self.local_pos[did]
            if lo[2] < self.min_takeoff_z:
                self._update_hold(did, False)
                continue

            if self.phase == 'lift':
                gxy = self._target_global_xy_for_phase(did, use_start=True)
                if gxy is None:
                    self._update_hold(did, False)
                    continue
                target_lxy = self._local_xy_from_global(did, gxy[0], gxy[1])
                target_z = self.layer_z[did]
                reached = self._within(did, target_lxy, target_z, check_xy=False)
                self._update_hold(did, reached)
                self._publish_guided_target(
                    did, target_lxy, target_z,
                    max_step_xy=self.max_step_xy_reconfig,
                    max_step_z=self.max_step_z_reconfig,
                )

            elif self.phase == 'xy':
                gxy = self._target_global_xy_for_phase(did, use_start=False)
                if gxy is None:
                    self._update_hold(did, False)
                    continue
                target_lxy = self._local_xy_from_global(did, gxy[0], gxy[1])
                target_z = self.layer_z[did]
                if self.search_active:
                    reached = self._within(did, target_lxy, target_z, check_xy=True)
                elif self.mission_active and did in self.target_offset_xy:
                    reached = self._within_relative_shape(did, self.target_offset_xy[did], target_z)
                else:
                    reached = self._within(did, target_lxy, target_z, check_xy=True)
                self._update_hold(did, reached)
                self._publish_guided_target(
                    did, target_lxy, target_z,
                    max_step_xy=self.max_step_xy_reconfig,
                    max_step_z=self.max_step_z_reconfig,
                )

            elif self.phase == 'merge':
                gxy = self._target_global_xy_for_phase(did, use_start=False)
                if gxy is None:
                    self._update_hold(did, False)
                    continue
                target_lxy = self._local_xy_from_global(did, gxy[0], gxy[1])
                target_z = self.target_alt
                if self.search_active:
                    reached = self._within(did, target_lxy, target_z, check_xy=True)
                elif self.mission_active and did in self.target_offset_xy:
                    reached = self._within_relative_shape(did, self.target_offset_xy[did], target_z)
                else:
                    reached = self._within(did, target_lxy, target_z, check_xy=True)
                self._update_hold(did, reached)
                self._publish_guided_target(
                    did, target_lxy, target_z,
                    max_step_xy=self.max_step_xy_reconfig,
                    max_step_z=self.max_step_z_reconfig,
                )

            elif self.phase == 'land':
                if did not in self.land_sent:
                    self._publish_land(did)
                    self.land_sent.add(did)

        if self._phase_timeout():
            self.get_logger().warn(f'阶段超时: {self.phase}，中止本次切换并回到 idle')
            self.phase = 'idle'
            self._clear_motion_targets()
            self.land_after_merge = False
            self.land_sent = set()
            self.pending_request = None
            self.reconfig_has_layers = False
            self._set_mission_speed_mode('cruise')
            self._clear_orbit_state()
            self._reset_phase_hold()
            return

        if self.phase == 'land':
            if self._all_landed():
                self.get_logger().info('>>> 全体已降落，进入 idle')
                self.phase = 'idle'
                self._clear_motion_targets()
                self.land_after_merge = False
                self.land_sent = set()
                self.pending_request = None
                self.mission_active = False
                self.reconfig_has_layers = False
                self._set_mission_speed_mode('cruise')
                self.mission_center_goal = None
                self.mission_center_current = None
                self._clear_search_state()
                self._clear_orbit_state()
                self._reset_phase_hold()
            return

        if self._phase_all_reached():
            if self.phase == 'lift':
                self._enter_phase('xy')
            elif self.phase == 'xy':
                if self.reconfig_has_layers:
                    self._enter_phase('merge')
                elif self.land_after_merge:
                    self.get_logger().info('>>> 已回原始编队，直接进入 land')
                    self._enter_phase('land')
                elif self.pending_request is not None:
                    pending = self.pending_request
                    self.pending_request = None
                    self.phase = 'idle'
                    if pending[0] == 'origin':
                        self._start_return_to_origin(do_land=pending[1])
                    elif pending[0] == 'search':
                        self._start_search_mission(pending[1])
                    elif pending[0] == 'orbit':
                        self._start_search_orbit(pending[1])
                    else:
                        self._start_reconfiguration(pending[1])
                else:
                    if self.orbit_active and self.target_global_xy:
                        self.steady_global_xy = {
                            did: [self.target_global_xy[did][0], self.target_global_xy[did][1]]
                            for did in self.target_global_xy
                        }
                    elif self.search_active and self.target_global_xy:
                        self.steady_global_xy = {
                            did: [self.target_global_xy[did][0], self.target_global_xy[did][1]]
                            for did in self.target_global_xy
                        }
                    elif self.last_plan_center is not None and self.target_offset_xy:
                        self.active_offset_xy = {
                            did: [self.target_offset_xy[did][0], self.target_offset_xy[did][1]]
                            for did in self.target_offset_xy
                        }
                        if not self.mission_active:
                            self.mission_center_current = [self.last_plan_center[0], self.last_plan_center[1]]
                    self.reconfig_has_layers = False
                    self._set_mission_speed_mode('cruise')
                    self.get_logger().info('>>> 队形切换完成，进入 idle')
                    self.phase = 'idle'
            elif self.phase == 'merge':
                if self.land_after_merge:
                    self.get_logger().info('>>> 已回原始编队，进入 land')
                    self._enter_phase('land')
                elif self.pending_request is not None:
                    pending = self.pending_request
                    self.pending_request = None
                    self.phase = 'idle'
                    if pending[0] == 'origin':
                        self._start_return_to_origin(do_land=pending[1])
                    elif pending[0] == 'search':
                        self._start_search_mission(pending[1])
                    elif pending[0] == 'orbit':
                        self._start_search_orbit(pending[1])
                    else:
                        self._start_reconfiguration(pending[1])
                else:
                    if self.orbit_active and self.target_global_xy:
                        self.steady_global_xy = {
                            did: [self.target_global_xy[did][0], self.target_global_xy[did][1]]
                            for did in self.target_global_xy
                        }
                    elif self.search_active and self.target_global_xy:
                        self.steady_global_xy = {
                            did: [self.target_global_xy[did][0], self.target_global_xy[did][1]]
                            for did in self.target_global_xy
                        }
                    elif self.last_plan_center is not None and self.target_offset_xy:
                        self.active_offset_xy = {
                            did: [self.target_offset_xy[did][0], self.target_offset_xy[did][1]]
                            for did in self.target_offset_xy
                        }
                        if not self.mission_active:
                            self.mission_center_current = [self.last_plan_center[0], self.last_plan_center[1]]
                    self.reconfig_has_layers = False
                    self._set_mission_speed_mode('cruise')
                    self.get_logger().info('>>> 队形切换完成，进入 idle')
                    self.phase = 'idle'


def main(args=None):
    rclpy.init(args=args)
    node = FormationCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
