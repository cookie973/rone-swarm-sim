"""向量运算与坐标变换工具函数

所有位置/速度统一使用 ENU (East-North-Up) 坐标系，与 uav_bridge 一致。
"""

import math
from typing import List

# ---------- 类型别名 ----------
Vec3 = List[float]  # [x, y, z]


# ---------- 基础向量运算 ----------

def vec_add(a: Vec3, b: Vec3) -> Vec3:
    """向量加法"""
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    """向量减法  a - b"""
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a: Vec3, s: float) -> Vec3:
    """标量乘法"""
    return [a[0] * s, a[1] * s, a[2] * s]


def vec_norm(a: Vec3) -> float:
    """向量模长"""
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def vec_normalize(a: Vec3) -> Vec3:
    """归一化，零向量返回 [0,0,0]"""
    n = vec_norm(a)
    if n < 1e-9:
        return [0.0, 0.0, 0.0]
    return vec_scale(a, 1.0 / n)


def vec_distance(a: Vec3, b: Vec3) -> float:
    """两点距离"""
    return vec_norm(vec_sub(a, b))


def vec_limit(a: Vec3, max_mag: float) -> Vec3:
    """将向量模长限制在 max_mag 以内"""
    n = vec_norm(a)
    if n > max_mag and n > 1e-9:
        return vec_scale(a, max_mag / n)
    return list(a)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> 偏航角 (rad, ENU)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
