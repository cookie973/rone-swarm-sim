#!/usr/bin/env python3
import itertools
import json
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Empty, Float32, String


class Test1SwarmReconfig(Node):
    """测试1: 自动起飞后原地进行 X/Y/V/T 连续变换并统计关键指标。"""

    def __init__(self, num_uav: int = 6, takeoff_alt: float = 5.0):
        super().__init__("test1_swarm_reconfig")
        self.num_uav = num_uav
        self.takeoff_alt = float(takeoff_alt)

        # X/Y/V/T 映射到工程内的编队名
        self.formation_seq = ["line_x", "line_y", "v_shape", "t_shape"]

        # 统计阈值
        self.collision_threshold_m = 1.0

        # 状态
        self.phase = "unknown"
        self.current_formation = "unknown"
        self.status = {}
        self.odom_pos: Dict[int, Optional[Tuple[float, float, float]]] = {i: None for i in range(1, num_uav + 1)}
        self.gps_llh: Dict[int, Optional[Tuple[float, float, float]]] = {i: None for i in range(1, num_uav + 1)}

        # ENU 参考
        self._ref = None  # (lat0, lon0, x0, y0, z0)

        # 轨迹长度累积（用于无效位移）
        self._seg_path_len = {i: 0.0 for i in range(1, num_uav + 1)}
        self._seg_last_pos = {i: None for i in range(1, num_uav + 1)}

        # 碰撞率统计（按采样占比）
        self._collision_samples = 0
        self._total_distance_samples = 0

        # 全程最小机间距离
        self.min_pair_distance_m = float("inf")

        # 命令响应时延与时长
        self.command_latency_s = []
        self.command_duration_s = []

        # ROS 接口
        self.clear_pub = self.create_publisher(Empty, "/swarm/clear_task", 10)
        self.formation_pub = self.create_publisher(String, "/swarm/formation", 10)

        self.arm_pubs = {}
        self.mode_pubs = {}
        self.takeoff_pubs = {}

        self.create_subscription(String, "/swarm/status", self._on_status, 20)

        for i in range(1, num_uav + 1):
            self.create_subscription(Odometry, f"/uav{i}/uav/odom", self._mk_odom_cb(i), 20)
            self.create_subscription(NavSatFix, f"/uav{i}/uav/navsatfix", self._mk_gps_cb(i), 20)

            self.arm_pubs[i] = self.create_publisher(Bool, f"/uav{i}/uav/cmd/arm", 10)
            self.mode_pubs[i] = self.create_publisher(String, f"/uav{i}/uav/cmd/mode", 10)
            self.takeoff_pubs[i] = self.create_publisher(Float32, f"/uav{i}/uav/cmd/takeoff", 10)

    # ---------- 坐标转换 ----------
    @staticmethod
    def _llh_to_ecef(lat_deg, lon_deg, alt_m):
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = 2.0 * f - f * f

        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        slat, clat = math.sin(lat), math.cos(lat)
        slon, clon = math.sin(lon), math.cos(lon)
        n = a / math.sqrt(1.0 - e2 * slat * slat)
        return (
            (n + alt_m) * clat * clon,
            (n + alt_m) * clat * slon,
            (n * (1.0 - e2) + alt_m) * slat,
        )

    @staticmethod
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

    def _global_xyz(self, did: int):
        # 优先用 GPS->ENU（跨机同一全局系），没有则退化 odom
        llh = self.gps_llh.get(did)
        if llh is not None:
            lat, lon, alt = llh
            if self._ref is None:
                x0, y0, z0 = self._llh_to_ecef(lat, lon, alt)
                self._ref = (lat, lon, x0, y0, z0)
            x, y, z = self._llh_to_ecef(lat, lon, alt)
            e, n, u = self._ecef_to_enu(x, y, z, self._ref[0], self._ref[1], self._ref[2], self._ref[3], self._ref[4])
            return (e, n, u)

        return self.odom_pos.get(did)

    # ---------- 回调 ----------
    def _on_status(self, msg: String):
        try:
            self.status = json.loads(msg.data)
            self.phase = self.status.get("phase", "unknown")
            self.current_formation = self.status.get("formation", "unknown")
        except Exception:
            self.phase = "decode_error"

    def _mk_odom_cb(self, did: int):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            self.odom_pos[did] = (float(p.x), float(p.y), float(p.z))

        return _cb

    def _mk_gps_cb(self, did: int):
        def _cb(msg: NavSatFix):
            lat, lon, alt = float(msg.latitude), float(msg.longitude), float(msg.altitude)
            if abs(lat) < 1e-3 and abs(lon) < 1e-3:
                return
            self.gps_llh[did] = (lat, lon, alt)

        return _cb

    # ---------- 统计 ----------
    def _update_pair_metrics(self):
        pts = {}
        for i in range(1, self.num_uav + 1):
            p = self._global_xyz(i)
            if p is not None:
                pts[i] = p

        if len(pts) < 2:
            return

        min_d = float("inf")
        for i, j in itertools.combinations(pts.keys(), 2):
            dx = pts[i][0] - pts[j][0]
            dy = pts[i][1] - pts[j][1]
            dz = pts[i][2] - pts[j][2]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            min_d = min(min_d, d)

        self.min_pair_distance_m = min(self.min_pair_distance_m, min_d)
        self._total_distance_samples += 1
        if min_d < self.collision_threshold_m:
            self._collision_samples += 1

    def _reset_segment_path(self):
        for i in range(1, self.num_uav + 1):
            self._seg_path_len[i] = 0.0
            self._seg_last_pos[i] = self._global_xyz(i)

    def _accumulate_segment_path(self):
        for i in range(1, self.num_uav + 1):
            cur = self._global_xyz(i)
            prev = self._seg_last_pos[i]
            if cur is not None and prev is not None:
                dx = cur[0] - prev[0]
                dy = cur[1] - prev[1]
                dz = cur[2] - prev[2]
                self._seg_path_len[i] += math.sqrt(dx * dx + dy * dy + dz * dz)
            self._seg_last_pos[i] = cur

    # ---------- 控制辅助 ----------
    def _spin_until(self, cond, timeout_s: float, step_s: float = 0.05):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=step_s)
            self._update_pair_metrics()
            self._accumulate_segment_path()
            if cond():
                return True
        return False

    def _brief_spin(self, rounds=1, timeout_sec=0.08):
        for _ in range(rounds):
            rclpy.spin_once(self, timeout_sec=timeout_sec)
            self._update_pair_metrics()

    def wait_topics_ready(self):
        self.get_logger().info("等待全部无人机 odom 就绪...")
        ok = self._spin_until(
            lambda: all(self.odom_pos[i] is not None for i in range(1, self.num_uav + 1)),
            timeout_s=60,
        )
        if not ok:
            raise RuntimeError("未在超时内收到全部 odom 数据")

    def clear_task(self, rounds=4):
        for _ in range(rounds):
            self.clear_pub.publish(Empty())
            self._brief_spin(rounds=1, timeout_sec=0.05)

    def takeoff_all(self):
        self.get_logger().info("执行自动起飞...")
        self.clear_task(rounds=4)

        # 串行稳态 GUIDED + ARM
        for did in range(1, self.num_uav + 1):
            for _ in range(4):
                m = String(); m.data = "GUIDED"
                a = Bool(); a.data = True
                self.mode_pubs[did].publish(m)
                self.arm_pubs[did].publish(a)
                self._brief_spin(rounds=2, timeout_sec=0.08)

        # 并行重试 TAKEOFF
        for _ in range(8):
            for did in range(1, self.num_uav + 1):
                m = String(); m.data = "GUIDED"
                a = Bool(); a.data = True
                t = Float32(); t.data = self.takeoff_alt
                self.mode_pubs[did].publish(m)
                self.arm_pubs[did].publish(a)
                self.takeoff_pubs[did].publish(t)
            self._brief_spin(rounds=2, timeout_sec=0.08)

        self.get_logger().info("等待全体达到起飞高度...")
        ok = self._spin_until(
            lambda: all(self.odom_pos[i] is not None and self.odom_pos[i][2] > 1.2 for i in range(1, self.num_uav + 1)),
            timeout_s=90,
        )
        if not ok:
            raise RuntimeError("起飞未在超时内完成")

    def run_sequence(self):
        self.get_logger().info("开始连续编队变换: X -> Y -> V -> T")
        sequence_start = time.time()

        invalid_displacement_total = 0.0
        per_cmd = []

        for form in self.formation_seq:
            self.get_logger().info(f"下发编队指令: {form}")
            self._reset_segment_path()
            seg_start_pos = {i: self._global_xyz(i) for i in range(1, self.num_uav + 1)}

            cmd_msg = String()
            cmd_msg.data = form
            t_cmd = time.time()
            self.formation_pub.publish(cmd_msg)

            # 若当前已是目标编队且 phase 处于 idle，则该命令可视为无动作。
            if self.phase == "idle" and self.current_formation == form:
                self._spin_until(lambda: False, timeout_s=0.2)
                per_cmd.append({
                    "formation": form,
                    "response_latency_s": 0.0,
                    "duration_s": 0.0,
                    "invalid_displacement_m": 0.0,
                    "note": "no_op_same_formation",
                })
                self.command_latency_s.append(0.0)
                self.command_duration_s.append(0.0)
                continue

            # 响应时延：指令下发到 phase 首次进入非 idle
            active_seen = {"v": False}

            def response_seen():
                if self.phase in ("lift", "xy", "merge"):
                    active_seen["v"] = True
                    return True
                return False

            if not self._spin_until(response_seen, timeout_s=20):
                latency = float("nan")
            else:
                latency = time.time() - t_cmd

            # 完成时长：从下发到本次切换回 idle（且经历过活跃阶段）
            def seg_done():
                if self.phase in ("lift", "xy", "merge"):
                    active_seen["v"] = True
                return active_seen["v"] and self.phase == "idle"

            ok = self._spin_until(seg_done, timeout_s=240)
            if not ok:
                raise RuntimeError(f"编队 {form} 未在超时内完成，phase={self.phase}")

            duration = time.time() - t_cmd

            seg_end_pos = {i: self._global_xyz(i) for i in range(1, self.num_uav + 1)}

            # 无效位移：sum(max(0, path_len - straight_len))
            seg_invalid = 0.0
            for i in range(1, self.num_uav + 1):
                p0 = seg_start_pos[i]
                p1 = seg_end_pos[i]
                if p0 is None or p1 is None:
                    continue
                dx = p1[0] - p0[0]
                dy = p1[1] - p0[1]
                dz = p1[2] - p0[2]
                straight = math.sqrt(dx * dx + dy * dy + dz * dz)
                seg_invalid += max(0.0, self._seg_path_len[i] - straight)

            invalid_displacement_total += seg_invalid
            self.command_latency_s.append(latency)
            self.command_duration_s.append(duration)

            per_cmd.append({
                "formation": form,
                "response_latency_s": latency,
                "duration_s": duration,
                "invalid_displacement_m": seg_invalid,
            })

        total_duration = time.time() - sequence_start
        collision_rate = (
            100.0 * self._collision_samples / self._total_distance_samples
            if self._total_distance_samples > 0
            else 0.0
        )

        result = {
            "test_name": "测试1",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "formation_sequence": self.formation_seq,
            "metrics": {
                "total_reconfig_time_s": total_duration,
                "min_inter_uav_distance_m": self.min_pair_distance_m,
                "avg_command_response_latency_s": _nanmean(self.command_latency_s),
                "total_invalid_displacement_m": invalid_displacement_total,
                "collision_rate_percent": collision_rate,
            },
            "per_command": per_cmd,
            "collision_threshold_m": self.collision_threshold_m,
            "samples": {
                "collision_samples": self._collision_samples,
                "total_distance_samples": self._total_distance_samples,
            },
        }

        return result


def _nanmean(vals):
    vv = [v for v in vals if isinstance(v, (int, float)) and not math.isnan(v)]
    if not vv:
        return float("nan")
    return sum(vv) / len(vv)


def main():
    rclpy.init()
    node = Test1SwarmReconfig(num_uav=6, takeoff_alt=5.0)

    try:
        node.wait_topics_ready()
        node.takeoff_all()
        result = node.run_sequence()

        print("\n=== 测试1结果 ===")
        print(json.dumps(result, ensure_ascii=False, indent=2))

        out_path = "/home/cookie/ros2_ws/src/swarm_control/tools/test1_result.json"
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(f"\n结果已保存: {out_path}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
