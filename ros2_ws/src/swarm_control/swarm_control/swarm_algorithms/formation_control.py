"""formation_control.py — 队形生成、保持与动态变换

功能：
    - 从 YAML 配置文件加载多种队形定义（菱形、V字形、线性、圆形）
    - 使用匈牙利算法求解最优 无人机→队形位置 映射
    - 虚拟领航者模式：领航者位置由外部指定或取群体质心
    - 计算每架无人机维持/回归队形所需的引导力
    - 运行时切换队形，平滑过渡

设计原则：
    - 每架无人机只需知道自己的 drone_id 和邻居状态即可工作
    - 队形分配使用全局最优（匈牙利算法），但只在切换队形时执行一次
    - 日常维持仅用本地位置误差 → 引导力
"""

import yaml
from typing import Dict, List, Optional

from swarm_control.utils.math_utils import (
    Vec3, vec_sub, vec_add, vec_norm, vec_scale, vec_limit,
)

# ---------- 尝试导入 scipy，不可用时退回贪心分配 ----------
try:
    from scipy.optimize import linear_sum_assignment
    import numpy as np
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


class FormationController:
    """队形控制器"""

    def __init__(self, formation_config_path: str, num_drones: int = 6):
        """
        Args:
            formation_config_path: formation_configs.yaml 的绝对路径
            num_drones: 无人机数量
        """
        self.num_drones = num_drones
        self.formations: Dict[str, List[Vec3]] = {}
        self.current_formation: str = ''
        self.assignment: Dict[int, int] = {}  # {drone_id: slot_index}
        self.leader_pos: Vec3 = [0.0, 0.0, 0.0]

        self._load_formations(formation_config_path)

    # ----------------------------------------------------------
    # 配置加载
    # ----------------------------------------------------------

    def _load_formations(self, path: str):
        """从 YAML 加载队形偏移量定义"""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        for name, cfg in data.get('formations', {}).items():
            offsets = cfg.get('offsets', [])
            self.formations[name] = [
                [float(o[0]), float(o[1]), float(o[2])] for o in offsets
            ]

    def get_available_formations(self) -> List[str]:
        """返回所有可用队形名称"""
        return list(self.formations.keys())

    # ----------------------------------------------------------
    # 队形分配
    # ----------------------------------------------------------

    def assign_positions(
        self,
        drone_positions: Dict[int, Vec3],
        target_formation: str,
    ) -> Dict[int, int]:
        """分配每架无人机到队形中的目标槽位

        使用确定性分配：按 drone_id 升序 → slot_index 顺序映射。
        保证所有无人机独立计算出完全一致的结果，无需全局协调。

        当首次切换队形且所有无人机都已知时，可选用匈牙利算法做一次
        最优重分配（暂不启用，优先保证一致性）。

        Args:
            drone_positions: {drone_id: [x, y, z]}  各机当前位置
            target_formation: 队形名称

        Returns:
            {drone_id: slot_index}  分配结果
        """
        if target_formation not in self.formations:
            return self.assignment

        offsets = self.formations[target_formation]
        ids = sorted(drone_positions.keys())
        n = min(len(ids), len(offsets))

        # 确定性分配：drone_id 升序 → slot 0, 1, 2, ...
        assignment = {ids[i]: i for i in range(n)}

        self.assignment = assignment
        self.current_formation = target_formation
        return assignment

    # ----------------------------------------------------------
    # 队形引导力
    # ----------------------------------------------------------

    def compute_formation_force(
        self,
        drone_id: int,
        my_pos: Vec3,
        max_force: float = 2.0,
        kp: float = 0.8,
    ) -> Vec3:
        """计算当前无人机维持队形所需的引导力

        Args:
            drone_id:  本机 ID
            my_pos:    本机当前位置 [x, y, z]
            max_force: 最大力幅值
            kp:        位置误差比例增益

        Returns:
            引导力向量 [fx, fy, fz]
        """
        if not self.current_formation or drone_id not in self.assignment:
            return [0.0, 0.0, 0.0]

        slot = self.assignment[drone_id]
        offsets = self.formations[self.current_formation]
        if slot >= len(offsets):
            return [0.0, 0.0, 0.0]

        # 目标位置 = 领航者位置 + 队形偏移
        target = vec_add(self.leader_pos, offsets[slot])

        # P 控制：力 = kp * (target - my_pos)
        error = vec_sub(target, my_pos)
        force = vec_scale(error, kp)

        return vec_limit(force, max_force)

    # ----------------------------------------------------------
    # 领航者管理
    # ----------------------------------------------------------

    def update_leader_position(self, pos: Vec3):
        """外部设定领航者位置"""
        self.leader_pos = list(pos)

    def compute_leader_from_centroid(
        self, drone_positions: Dict[int, Vec3]
    ) -> Vec3:
        """从所有无人机位置的质心推算领航者位置

        在无明确领航者时，用群体质心作为虚拟领航者。
        """
        if not drone_positions:
            return self.leader_pos

        cx, cy, cz = 0.0, 0.0, 0.0
        for pos in drone_positions.values():
            cx += pos[0]
            cy += pos[1]
            cz += pos[2]
        n = len(drone_positions)
        self.leader_pos = [cx / n, cy / n, cz / n]
        return self.leader_pos

    # ----------------------------------------------------------
    # 队形切换
    # ----------------------------------------------------------

    def switch_formation(
        self,
        new_formation: str,
        drone_positions: Dict[int, Vec3],
    ) -> bool:
        """动态切换队形

        Args:
            new_formation:    新队形名称
            drone_positions:  各机当前位置

        Returns:
            True 切换成功，False 队形不存在
        """
        if new_formation not in self.formations:
            return False

        # 重新分配槽位
        self.assign_positions(drone_positions, new_formation)
        return True
