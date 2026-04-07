"""邻居发现与局部信息交换模块

功能：
    - 根据通信半径筛选有效邻居
    - 模拟通信延迟与随机丢包
    - 过滤过期信息（超过 stale_threshold 的数据丢弃）
"""

import random
from swarm_control.utils.math_utils import vec_distance


class SwarmCommunication:
    """去中心化通信模拟"""

    def __init__(
        self,
        drone_id: int,
        comm_radius: float = 20.0,
        delay_ms: int = 0,
        drop_rate: float = 0.0,
        stale_threshold: float = 0.5,
    ):
        """
        Args:
            drone_id:        本机编号
            comm_radius:     通信半径 (m)
            delay_ms:        模拟通信延迟 (ms)，暂留接口，第一步不启用
            drop_rate:       丢包率 [0, 1)
            stale_threshold: 数据过期阈值 (s)
        """
        self.drone_id = drone_id
        self.comm_radius = comm_radius
        self.delay_ms = delay_ms
        self.drop_rate = drop_rate
        self.stale_threshold = stale_threshold

    # ----------------------------------------------------------
    def should_drop(self) -> bool:
        """模拟随机丢包。返回 True 表示丢弃这条消息。"""
        if self.drop_rate <= 0.0:
            return False
        return random.random() < self.drop_rate

    # ----------------------------------------------------------
    def get_neighbors(self, my_pos: list, all_states: dict,
                      current_time: float) -> list:
        """筛选通信半径内且未过期的邻居

        Args:
            my_pos:       本机位置 [x, y, z]
            all_states:   {drone_id: state_dict}  已缓存的邻居状态
            current_time: 当前时间戳 (s)

        Returns:
            有效邻居列表 [state_dict, ...]
        """
        neighbors = []
        for did, state in all_states.items():
            if did == self.drone_id:
                continue

            # 过期检查
            age = current_time - state['timestamp']
            if age > self.stale_threshold:
                continue

            # 距离检查
            dist = vec_distance(my_pos, state['position'])
            if dist <= self.comm_radius:
                neighbors.append(state)

        return neighbors
