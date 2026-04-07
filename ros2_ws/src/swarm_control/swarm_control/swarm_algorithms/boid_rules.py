"""Boid 模型实现 — 分离 (Separation)、聚合 (Cohesion)、对齐 (Alignment)

所有计算仅基于通信半径内的局部邻居信息，不依赖全局信息。

每个规则返回一个「转向力」(steering force) 向量，三种力加权合成后作用于
无人机当前速度，产生新的速度指令。

邻居数据格式 (dict):
    {
        'drone_id': int,
        'position': [x, y, z],   # ENU (m)
        'velocity': [vx, vy, vz],  # ENU (m/s)
        'heading':  float,         # rad
        'timestamp': float,        # 秒
    }
"""

from swarm_control.utils.math_utils import (
    Vec3, vec_add, vec_sub, vec_scale, vec_norm, vec_normalize, vec_limit,
)


class BoidRules:
    """经典 Boid 三规则 + 幅值限制"""

    def __init__(
        self,
        separation_dist: float = 3.0,
        cohesion_dist: float = 15.0,
        alignment_dist: float = 10.0,
        w_sep: float = 1.5,
        w_coh: float = 1.0,
        w_ali: float = 1.0,
        max_speed: float = 5.0,
        max_force: float = 2.0,
    ):
        """
        Args:
            separation_dist: 分离作用距离 (m)
            cohesion_dist:   聚合作用距离 (m)
            alignment_dist:  对齐作用距离 (m)
            w_sep/w_coh/w_ali: 各规则权重
            max_speed: 最大速度 (m/s)
            max_force: 单规则最大转向力 (m/s²)
        """
        self.separation_dist = separation_dist
        self.cohesion_dist = cohesion_dist
        self.alignment_dist = alignment_dist
        self.w_sep = w_sep
        self.w_coh = w_coh
        self.w_ali = w_ali
        self.max_speed = max_speed
        self.max_force = max_force

    # ----------------------------------------------------------
    def compute_separation(self, my_pos: Vec3, neighbors: list) -> Vec3:
        """分离力：避免与邻居碰撞，距离越近排斥力越大（反距离平方加权）"""
        steer = [0.0, 0.0, 0.0]
        count = 0
        for n in neighbors:
            diff = vec_sub(my_pos, n['position'])
            dist = vec_norm(diff)
            if 1e-3 < dist < self.separation_dist:
                # 反距离平方：越近越强
                repulsion = vec_scale(vec_normalize(diff), 1.0 / (dist * dist))
                steer = vec_add(steer, repulsion)
                count += 1
        if count > 0:
            steer = vec_scale(steer, 1.0 / count)
        return vec_limit(steer, self.max_force)

    # ----------------------------------------------------------
    def compute_cohesion(self, my_pos: Vec3, neighbors: list) -> Vec3:
        """聚合力：向通信范围内邻居的质心移动"""
        center = [0.0, 0.0, 0.0]
        count = 0
        for n in neighbors:
            dist = vec_norm(vec_sub(my_pos, n['position']))
            if dist < self.cohesion_dist:
                center = vec_add(center, n['position'])
                count += 1
        if count == 0:
            return [0.0, 0.0, 0.0]
        center = vec_scale(center, 1.0 / count)
        desired = vec_sub(center, my_pos)
        return vec_limit(desired, self.max_force)

    # ----------------------------------------------------------
    def compute_alignment(self, my_pos: Vec3, my_vel: Vec3,
                          neighbors: list) -> Vec3:
        """对齐力：速度方向趋于与邻居一致"""
        avg_vel = [0.0, 0.0, 0.0]
        count = 0
        for n in neighbors:
            dist = vec_norm(vec_sub(my_pos, n['position']))
            if dist < self.alignment_dist:
                avg_vel = vec_add(avg_vel, n['velocity'])
                count += 1
        if count == 0:
            return [0.0, 0.0, 0.0]
        avg_vel = vec_scale(avg_vel, 1.0 / count)
        steering = vec_sub(avg_vel, my_vel)
        return vec_limit(steering, self.max_force)

    # ----------------------------------------------------------
    def compute_total(self, my_pos: Vec3, my_vel: Vec3,
                      neighbors: list) -> Vec3:
        """加权合成三种转向力

        Returns:
            合成转向力向量 (m/s²)，幅值 <= max_force * 最大权重和
        """
        sep = self.compute_separation(my_pos, neighbors)
        coh = self.compute_cohesion(my_pos, neighbors)
        ali = self.compute_alignment(my_pos, my_vel, neighbors)

        total = [0.0, 0.0, 0.0]
        total = vec_add(total, vec_scale(sep, self.w_sep))
        total = vec_add(total, vec_scale(coh, self.w_coh))
        total = vec_add(total, vec_scale(ali, self.w_ali))

        return total
