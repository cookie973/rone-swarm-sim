"""obstacle_avoidance.py — 改进人工势场法避障

功能：
    - 静态障碍物：从预设列表（或 Gazebo 话题）获取
    - 动态障碍物：支持运行时更新位置
    - 改进人工势场法：
        · 引力场：目标点产生引力
        · 斥力场：障碍物在影响范围内产生排斥力，安全距离外为零
        · 解决局部极小值问题：检测静止/振荡状态时添加随机扰动
    - 同时将其他无人机视为动态障碍物（防碰撞）

设计原则：
    - 仅基于局部感知信息（通信半径内）
    - 输出排斥力向量供 drone_controller 加权合成
"""

import math
import random
from typing import List, Optional

from swarm_control.utils.math_utils import (
    Vec3, vec_sub, vec_add, vec_scale, vec_norm, vec_normalize, vec_limit,
    vec_distance,
)


class ObstacleAvoidance:
    """改进人工势场法避障"""

    def __init__(
        self,
        safe_distance: float = 2.0,
        influence_distance: float = 8.0,
        repulsion_gain: float = 5.0,
        attraction_gain: float = 1.0,
        drone_safe_distance: float = 2.5,
        drone_influence_distance: float = 6.0,
        drone_repulsion_gain: float = 4.0,
        stuck_threshold: float = 0.3,
        stuck_check_cycles: int = 20,
        perturbation_strength: float = 1.5,
    ):
        """
        Args:
            safe_distance:          障碍物最小安全距离 (m)
            influence_distance:     障碍物斥力影响范围 (m)
            repulsion_gain:         障碍物斥力增益
            attraction_gain:        目标引力增益
            drone_safe_distance:    无人机间安全距离 (m)
            drone_influence_distance: 无人机间斥力影响 (m)
            drone_repulsion_gain:   无人机间斥力增益
            stuck_threshold:        判断"卡住"的速度阈值 (m/s)
            stuck_check_cycles:     连续低速多少个周期判定为卡住
            perturbation_strength:  随机扰动强度 (m/s²)
        """
        self.safe_distance = safe_distance
        self.influence_distance = influence_distance
        self.repulsion_gain = repulsion_gain
        self.attraction_gain = attraction_gain
        self.drone_safe_distance = drone_safe_distance
        self.drone_influence_distance = drone_influence_distance
        self.drone_repulsion_gain = drone_repulsion_gain
        self.stuck_threshold = stuck_threshold
        self.stuck_check_cycles = stuck_check_cycles
        self.perturbation_strength = perturbation_strength

        # 障碍物列表: [[x, y, z], ...]
        self.static_obstacles: List[Vec3] = []
        self.dynamic_obstacles: List[Vec3] = []

        # 局部极小值检测
        self._stuck_counter = 0

    # ----------------------------------------------------------
    # 障碍物管理
    # ----------------------------------------------------------

    def set_static_obstacles(self, obstacles: List[Vec3]):
        """设置静态障碍物列表"""
        self.static_obstacles = [list(o) for o in obstacles]

    def set_dynamic_obstacles(self, obstacles: List[Vec3]):
        """更新动态障碍物位置"""
        self.dynamic_obstacles = [list(o) for o in obstacles]

    # ----------------------------------------------------------
    # 斥力计算
    # ----------------------------------------------------------

    def _single_repulsion(
        self,
        my_pos: Vec3,
        obs_pos: Vec3,
        safe_dist: float,
        influence_dist: float,
        gain: float,
    ) -> Vec3:
        """计算单个障碍物的斥力

        斥力公式 (改进):
            当 d <= influence_dist:
                F = gain * (1/d - 1/influence_dist) * (1/d²) * 方向
            当 d <= safe_dist:
                F 额外增强 (紧急排斥)
            当 d > influence_dist:
                F = 0
        """
        diff = vec_sub(my_pos, obs_pos)  # 从障碍物指向自身
        d = vec_norm(diff)

        if d < 0.01:
            # 几乎重合，给一个随机方向的大力
            angle = random.uniform(0, 2 * math.pi)
            return [gain * math.cos(angle), gain * math.sin(angle), 0.0]

        if d > influence_dist:
            return [0.0, 0.0, 0.0]

        direction = vec_scale(diff, 1.0 / d)

        # 基础斥力：反距离
        magnitude = gain * (1.0 / d - 1.0 / influence_dist) / (d * d)

        # 紧急区域加强
        if d < safe_dist:
            emergency = gain * 2.0 * (safe_dist / max(d, 0.1)) ** 2
            magnitude += emergency

        return vec_scale(direction, magnitude)

    def compute_repulsion(self, my_pos: Vec3) -> Vec3:
        """计算所有障碍物(静态+动态)的合斥力"""
        total = [0.0, 0.0, 0.0]

        all_obs = self.static_obstacles + self.dynamic_obstacles
        for obs in all_obs:
            force = self._single_repulsion(
                my_pos, obs,
                self.safe_distance,
                self.influence_distance,
                self.repulsion_gain,
            )
            total = vec_add(total, force)

        return total

    def compute_drone_repulsion(
        self, my_pos: Vec3, neighbors: list
    ) -> Vec3:
        """计算其他无人机产生的排斥力（防碰撞）

        Args:
            neighbors: 邻居状态列表 [{position: [x,y,z], ...}, ...]
        """
        total = [0.0, 0.0, 0.0]

        for n in neighbors:
            force = self._single_repulsion(
                my_pos, n['position'],
                self.drone_safe_distance,
                self.drone_influence_distance,
                self.drone_repulsion_gain,
            )
            total = vec_add(total, force)

        return total

    # ----------------------------------------------------------
    # 引力计算
    # ----------------------------------------------------------

    def compute_attractive(self, my_pos: Vec3, goal_pos: Vec3) -> Vec3:
        """计算目标点引力

        简单线性引力：F = gain * (goal - my_pos)，限幅
        """
        diff = vec_sub(goal_pos, my_pos)
        d = vec_norm(diff)
        if d < 0.1:
            return [0.0, 0.0, 0.0]
        force = vec_scale(diff, self.attraction_gain)
        return vec_limit(force, self.repulsion_gain)  # 限幅同斥力增益

    # ----------------------------------------------------------
    # 局部极小值检测与扰动
    # ----------------------------------------------------------

    def check_and_perturb(self, my_vel: Vec3) -> Vec3:
        """检测是否卡在局部极小值，若是则添加随机扰动

        Args:
            my_vel: 当前速度

        Returns:
            扰动力向量（无卡顿时返回零向量）
        """
        speed = vec_norm(my_vel)
        if speed < self.stuck_threshold:
            self._stuck_counter += 1
        else:
            self._stuck_counter = 0

        if self._stuck_counter >= self.stuck_check_cycles:
            # 添加随机水平方向扰动
            angle = random.uniform(0, 2 * math.pi)
            self._stuck_counter = 0  # 重置计数
            return [
                self.perturbation_strength * math.cos(angle),
                self.perturbation_strength * math.sin(angle),
                0.0,
            ]

        return [0.0, 0.0, 0.0]

    # ----------------------------------------------------------
    # 综合避障力
    # ----------------------------------------------------------

    def compute_total_avoidance(
        self,
        my_pos: Vec3,
        my_vel: Vec3,
        neighbors: list,
        goal_pos: Optional[Vec3] = None,
    ) -> Vec3:
        """计算综合避障力 = 障碍物斥力 + 无人机间斥力 + 引力 + 扰动

        Args:
            my_pos:   本机位置
            my_vel:   本机速度
            neighbors: 邻居列表
            goal_pos: 目标点（可选，用于引力计算）

        Returns:
            综合避障力向量
        """
        total = [0.0, 0.0, 0.0]

        # 障碍物斥力
        obs_force = self.compute_repulsion(my_pos)
        total = vec_add(total, obs_force)

        # 无人机间斥力
        drone_force = self.compute_drone_repulsion(my_pos, neighbors)
        total = vec_add(total, drone_force)

        # 目标引力
        if goal_pos is not None:
            att_force = self.compute_attractive(my_pos, goal_pos)
            total = vec_add(total, att_force)

        # 局部极小值扰动
        perturb = self.check_and_perturb(my_vel)
        total = vec_add(total, perturb)

        return total
