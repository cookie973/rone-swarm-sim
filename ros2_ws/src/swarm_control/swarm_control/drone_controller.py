"""drone_controller.py — 单架无人机控制节点（蜂群大脑）

每架无人机运行一个实例，通过 drone_id 区分。
与 uav_bridge 对接：
    订阅  /{ns_prefix}{drone_id}/uav/odom           (nav_msgs/Odometry)
    发布  /{ns_prefix}{drone_id}/uav/cmd/velocity    (geometry_msgs/TwistStamped)
蜂群通信：
    发布  /swarm/drone_{drone_id}/state              (swarm_control/SwarmState)
    订阅  /swarm/drone_{i}/state  (i ≠ drone_id)

控制循环 (默认 10Hz)：
    1. 读取自身位姿
    2. 广播自身状态
    3. 收集有效邻居
    4. 计算 Boid 转向力
    5. 合成速度指令并发送
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Vector3
from std_msgs.msg import String

from swarm_msgs.msg import SwarmState
from swarm_control.swarm_algorithms.boid_rules import BoidRules
from swarm_control.swarm_algorithms.communication import SwarmCommunication
from swarm_control.swarm_algorithms.formation_control import FormationController
from swarm_control.swarm_algorithms.obstacle_avoidance import ObstacleAvoidance
from swarm_control.utils.math_utils import (
    vec_add, vec_scale, vec_norm, vec_limit, quaternion_to_yaw,
)
from ament_index_python.packages import get_package_share_directory


class DroneController(Node):
    """分布式蜂群控制 — 单机节点"""

    def __init__(self):
        super().__init__('drone_controller')

        # ==================== 参数声明 ====================
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 6)
        self.declare_parameter('ns_prefix', 'uav')
        self.declare_parameter('comm_radius', 20.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_force', 2.0)
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('alt_kp', 1.0)

        # Boid 参数
        self.declare_parameter('boid.separation_dist', 3.0)
        self.declare_parameter('boid.cohesion_dist', 15.0)
        self.declare_parameter('boid.alignment_dist', 10.0)
        self.declare_parameter('boid.w_separation', 1.5)
        self.declare_parameter('boid.w_cohesion', 1.0)
        self.declare_parameter('boid.w_alignment', 1.0)

        # 队形参数
        self.declare_parameter('formation.default', 'diamond')
        self.declare_parameter('formation.kp', 0.8)
        self.declare_parameter('formation.kd', 0.6)
        self.declare_parameter('formation.w_formation', 1.2)

        # 通信参数
        self.declare_parameter('communication.delay_ms', 0)
        self.declare_parameter('communication.drop_rate', 0.0)
        self.declare_parameter('communication.stale_threshold', 0.5)

        # 避障参数
        self.declare_parameter('obstacle_avoidance.safe_distance', 2.0)
        self.declare_parameter('obstacle_avoidance.influence_distance', 8.0)
        self.declare_parameter('obstacle_avoidance.repulsion_gain', 5.0)
        self.declare_parameter('obstacle_avoidance.drone_safe_distance', 2.5)
        self.declare_parameter('obstacle_avoidance.drone_influence_distance', 6.0)
        self.declare_parameter('obstacle_avoidance.drone_repulsion_gain', 4.0)
        self.declare_parameter('obstacle_avoidance.w_avoidance', 1.0)

        # ==================== 读取参数 ====================
        self.drone_id = self.get_parameter('drone_id').value
        self.num_drones = self.get_parameter('num_drones').value
        self.ns_prefix = self.get_parameter('ns_prefix').value
        self.comm_radius = self.get_parameter('comm_radius').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_force = self.get_parameter('max_force').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.alt_kp = self.get_parameter('alt_kp').value

        sep_dist = self.get_parameter('boid.separation_dist').value
        coh_dist = self.get_parameter('boid.cohesion_dist').value
        ali_dist = self.get_parameter('boid.alignment_dist').value
        w_sep = self.get_parameter('boid.w_separation').value
        w_coh = self.get_parameter('boid.w_cohesion').value
        w_ali = self.get_parameter('boid.w_alignment').value

        self.formation_name = self.get_parameter('formation.default').value
        self.formation_kp = self.get_parameter('formation.kp').value
        self.formation_kd = self.get_parameter('formation.kd').value
        self.w_formation = self.get_parameter('formation.w_formation').value

        delay_ms = self.get_parameter('communication.delay_ms').value
        drop_rate = self.get_parameter('communication.drop_rate').value
        stale_th = self.get_parameter('communication.stale_threshold').value

        obs_safe = self.get_parameter('obstacle_avoidance.safe_distance').value
        obs_influence = self.get_parameter('obstacle_avoidance.influence_distance').value
        obs_gain = self.get_parameter('obstacle_avoidance.repulsion_gain').value
        drone_safe = self.get_parameter('obstacle_avoidance.drone_safe_distance').value
        drone_influence = self.get_parameter('obstacle_avoidance.drone_influence_distance').value
        drone_gain = self.get_parameter('obstacle_avoidance.drone_repulsion_gain').value
        self.w_avoidance = self.get_parameter('obstacle_avoidance.w_avoidance').value

        # ==================== 内部状态 ====================
        self.my_position = None   # [x, y, z] ENU
        self.my_velocity = None   # [vx, vy, vz] ENU
        self.my_heading = 0.0
        self.neighbor_states = {}  # {drone_id: state_dict}
        self.anchor_pos = None     # 首次位置锚点（防止漂移）
        self._cmd_vx = 0.0         # 滤波后速度指令
        self._cmd_vy = 0.0

        # ==================== 初始化算法模块 ====================
        self.boid = BoidRules(
            separation_dist=sep_dist,
            cohesion_dist=coh_dist,
            alignment_dist=ali_dist,
            w_sep=w_sep, w_coh=w_coh, w_ali=w_ali,
            max_speed=self.max_speed,
            max_force=self.max_force,
        )

        self.comm = SwarmCommunication(
            drone_id=self.drone_id,
            comm_radius=self.comm_radius,
            delay_ms=delay_ms,
            drop_rate=drop_rate,
            stale_threshold=stale_th,
        )

        # 队形控制器
        pkg_share = get_package_share_directory('swarm_control')
        formation_cfg = os.path.join(pkg_share, 'config', 'formation_configs.yaml')
        self.formation = FormationController(formation_cfg, self.num_drones)
        self.formation_assigned = False
        self._locked_leader = None  # 锁定后的领航者位置
        self._start_time = 0.0     # 控制器启动时间（延后在首次控制时记录）

        # 避障模块
        self.obstacle_avoidance = ObstacleAvoidance(
            safe_distance=obs_safe,
            influence_distance=obs_influence,
            repulsion_gain=obs_gain,
            drone_safe_distance=drone_safe,
            drone_influence_distance=drone_influence,
            drone_repulsion_gain=drone_gain,
        )

        # ==================== ROS 2 接口 ====================
        ns = f'/{self.ns_prefix}{self.drone_id}'

        # QoS：里程计用 BEST_EFFORT（与 uav_bridge 匹配）
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 订阅自身里程计
        self.create_subscription(
            Odometry,
            f'{ns}/uav/odom',
            self._odom_cb,
            sensor_qos,
        )

        # 发布速度指令（备用）
        self.vel_pub = self.create_publisher(
            TwistStamped,
            f'{ns}/uav/cmd/velocity',
            10,
        )

        # 发布位置控制指令（主要）
        self.move_pub = self.create_publisher(
            Vector3,
            f'{ns}/uav/cmd/move_relative',
            10,
        )

        # 发布自身蜂群状态
        self.state_pub = self.create_publisher(
            SwarmState,
            f'/swarm/drone_{self.drone_id}/state',
            10,
        )

        # 订阅队形切换话题
        self.create_subscription(
            String,
            '/swarm/formation',
            self._formation_cb,
            10,
        )

        # 订阅其他无人机蜂群状态
        for i in range(1, self.num_drones + 1):
            if i != self.drone_id:
                self.create_subscription(
                    SwarmState,
                    f'/swarm/drone_{i}/state',
                    lambda msg, did=i: self._neighbor_state_cb(did, msg),
                    10,
                )

        # 控制定时器
        dt = 1.0 / max(1.0, self.control_rate)
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'drone_controller 启动: id={self.drone_id}, '
            f'话题前缀={ns}, 控制频率={self.control_rate}Hz, '
            f'默认队形={self.formation_name}'
        )

    # ===================== 回调函数 =====================

    def _formation_cb(self, msg: String):
        """队形切换回调 — 收到新队形名称后重新分配"""
        name = msg.data.strip().lower()
        avail = self.formation.get_available_formations()
        if name not in avail:
            self.get_logger().warn(f'未知队形 "{name}", 可选: {avail}')
            return
        self.formation_name = name
        self.formation_assigned = False  # 标记需要重新分配
        self.get_logger().info(f'队形切换 -> {name}')

    def _odom_cb(self, msg: Odometry):
        """里程计回调 —— 更新自身位姿和速度"""
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        self.my_position = [p.x, p.y, p.z]
        self.my_velocity = [v.x, v.y, v.z]
        self.my_heading = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        # 锁定首次位置作为锚点
        if self.anchor_pos is None:
            self.anchor_pos = [p.x, p.y, p.z]

    def _neighbor_state_cb(self, drone_id: int, msg: SwarmState):
        """邻居状态回调 —— 模拟丢包后缓存"""
        if self.comm.should_drop():
            return
        self.neighbor_states[msg.drone_id] = {
            'drone_id': msg.drone_id,
            'position': [msg.position.x, msg.position.y, msg.position.z],
            'velocity': [msg.velocity.x, msg.velocity.y, msg.velocity.z],
            'heading': msg.heading,
            'timestamp': msg.timestamp.sec + msg.timestamp.nanosec * 1e-9,
        }

    # ===================== 控制循环 =====================

    def _control_loop(self):
        """主控制循环 (10Hz) — move_relative 位置控制

        使用 ArduCopter 内部位置控制器（move_relative 话题），
        每周期发送 ENU 相对位移指令。ArduCopter 负责速度/姿态控制，
        避免外部速度指令导致的姿态不稳问题。
        """
        if self.my_position is None or self.my_velocity is None:
            return  # 尚未收到里程计

        # 记录启动时间（首次进入控制循环）
        if self._start_time == 0.0:
            self._start_time = self.get_clock().now().nanoseconds / 1e9

        # 始终广播自身状态（即使在起飞阶段也广播，供邻居感知）
        self._publish_state()

        # ---- 起飞保护：高度不足时不发送水平指令 ----
        if self.my_position[2] < self.target_altitude * 0.6:
            return

        # ---- 获取邻居 ----
        now = self.get_clock().now().nanoseconds / 1e9
        all_neighbors = self._get_all_known_neighbors(now)

        # ---- 计算队形目标位置 ----
        target = self._get_formation_target(all_neighbors)
        if target is None:
            return

        # ---- 发送 move_relative 位移指令 ----
        ex = target[0] - self.my_position[0]
        ey = target[1] - self.my_position[1]
        ez = target[2] - self.my_position[2]

        self._publish_move_relative(ex, ey, ez)

    # ===================== 辅助：邻居 =====================

    def _get_all_known_neighbors(self, current_time: float) -> list:
        """获取所有已知邻居状态（不限制通信半径，仅过滤过期数据）

        用于队形控制——需要全局一致的质心和分配。
        """
        stale_th = self.comm.stale_threshold * 5.0  # 队形用宽松超时
        neighbors = []
        for did, state in self.neighbor_states.items():
            if did == self.drone_id:
                continue
            age = current_time - state['timestamp']
            if age < stale_th:
                neighbors.append(state)
        return neighbors

    # ===================== 队形辅助 =====================

    def _get_formation_target(self, all_neighbors: list):
        """计算本机队形目标位置 [x, y, z]，若尚未就绪返回 None

        队形中心固定在起飞原点 (0, 0, target_altitude)。
        目标位置 = 固定中心 + 队形偏移[slot]
        """
        if not self.formation_name:
            return None

        # 收集所有已知位置（用于槽位分配）
        all_positions = {self.drone_id: self.my_position}
        for n in all_neighbors:
            all_positions[n['drone_id']] = n['position']

        # 等邻居到齐再分配（至少 num_drones-1 个邻居，或 5 秒后降级）
        if not self.formation_assigned:
            have_all = len(all_positions) >= self.num_drones
            waited_long = (self.get_clock().now().nanoseconds / 1e9
                           - self._start_time > 5.0)
            if not have_all and not waited_long:
                return None  # 还没到齐，暂不控制

            if self._locked_leader is None:
                self._locked_leader = [0.0, 0.0, self.target_altitude]
            self.formation.assign_positions(all_positions, self.formation_name)
            self.formation_assigned = True
            slot = self.formation.assignment.get(self.drone_id, -1)
            self.get_logger().info(
                f'队形分配: drone_{self.drone_id} -> 槽位 {slot}, '
                f'队形={self.formation_name}, 已知无人机={len(all_positions)}架')

        # 计算目标位置
        self.formation.leader_pos = self._locked_leader
        slot = self.formation.assignment.get(self.drone_id, -1)
        if slot < 0:
            return None
        offsets = self.formation.formations.get(self.formation.current_formation, [])
        if slot >= len(offsets):
            return None
        target = [
            self._locked_leader[0] + offsets[slot][0],
            self._locked_leader[1] + offsets[slot][1],
            self._locked_leader[2] + offsets[slot][2],
        ]
        return target

    # ===================== 辅助函数 =====================

    def _publish_state(self):
        """广播自身蜂群状态"""
        msg = SwarmState()
        msg.drone_id = self.drone_id
        msg.position.x = self.my_position[0]
        msg.position.y = self.my_position[1]
        msg.position.z = self.my_position[2]
        msg.velocity.x = self.my_velocity[0]
        msg.velocity.y = self.my_velocity[1]
        msg.velocity.z = self.my_velocity[2]
        msg.heading = self.my_heading
        msg.timestamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def _publish_velocity(self, vx: float, vy: float, vz: float):
        """发送 ENU 速度指令给 uav_bridge"""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.linear.z = float(vz)
        self.vel_pub.publish(cmd)

    def _publish_move_relative(self, dx: float, dy: float, dz: float):
        """发送 ENU 相对位移指令给 uav_bridge (move_relative)"""
        msg = Vector3()
        msg.x = float(dx)
        msg.y = float(dy)
        msg.z = float(dz)
        self.move_pub.publish(msg)


def main(args=None):
    """ROS 2 entry point"""
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
