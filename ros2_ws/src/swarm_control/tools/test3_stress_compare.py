#!/usr/bin/env python3
import json
import math
import random
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String


class StressReplanCompare(Node):
    """测试3: 高频随机干预下，对比快照重规划与回原点重规划。"""

    def __init__(self, num_uav: int = 6):
        super().__init__("test3_stress_replan_compare")
        self.num_uav = num_uav

        # 干预设置
        self.freq_list_hz = [2.0, 5.0, 10.0]
        self.intervention_duration_s = 18.0
        self.response_timeout_s = 2.0
        self.deadlock_timeout_s = 6.0
        self.settle_timeout_s = 20.0
        self.random_seed = 20260408

        # 状态
        self.phase = "unknown"
        self.current_formation = "unknown"
        self.mission_active = False
        self.mission_goal_xy = None
        self.status_seq = 0

        self.odom_pos: Dict[int, Optional[Tuple[float, float, float]]] = {i: None for i in range(1, num_uav + 1)}

        # 轨迹累积
        self._last_pos = {i: None for i in range(1, num_uav + 1)}
        self._path_len = {i: 0.0 for i in range(1, num_uav + 1)}

        # ROS 接口
        self.clear_pub = self.create_publisher(Empty, "/swarm/clear_task", 10)
        self.form_pub = self.create_publisher(String, "/swarm/formation", 10)
        self.mission_pub = self.create_publisher(Point, "/swarm/mission_center", 10)

        self.create_subscription(String, "/swarm/status", self._on_status, 20)
        for i in range(1, num_uav + 1):
            self.create_subscription(Odometry, f"/uav{i}/uav/odom", self._mk_odom_cb(i), 20)

    # ---------- 回调 ----------
    def _on_status(self, msg: String):
        self.status_seq += 1
        try:
            data = json.loads(msg.data)
            self.phase = data.get("phase", "unknown")
            self.current_formation = data.get("formation", "unknown")
            self.mission_active = bool(data.get("mission_active", False))
            mg = data.get("mission_goal", None)
            if isinstance(mg, list) and len(mg) >= 2:
                self.mission_goal_xy = (float(mg[0]), float(mg[1]))
            else:
                self.mission_goal_xy = None
        except Exception:
            self.phase = "decode_error"

    def _mk_odom_cb(self, did: int):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            self.odom_pos[did] = (float(p.x), float(p.y), float(p.z))

        return _cb

    # ---------- 通用 ----------
    def _spin_step(self, timeout_sec=0.05):
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        for i in range(1, self.num_uav + 1):
            cur = self.odom_pos[i]
            prev = self._last_pos[i]
            if cur is not None and prev is not None:
                dx = cur[0] - prev[0]
                dy = cur[1] - prev[1]
                dz = cur[2] - prev[2]
                self._path_len[i] += math.sqrt(dx * dx + dy * dy + dz * dz)
            self._last_pos[i] = cur

    def _spin_until(self, cond, timeout_s: float, step_s: float = 0.05):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            self._spin_step(step_s)
            if cond():
                return True
        return False

    def _baseline_state(self):
        return {
            "phase": self.phase,
            "formation": self.current_formation,
            "mission_goal": self.mission_goal_xy,
            "status_seq": self.status_seq,
        }

    def _state_changed_since(self, b):
        if self.status_seq != b["status_seq"]:
            if self.phase != b["phase"]:
                return True
            if self.current_formation != b["formation"]:
                return True
            if self.mission_goal_xy != b["mission_goal"]:
                return True
        return False

    def _centroid(self):
        pts = [self.odom_pos[i] for i in range(1, self.num_uav + 1) if self.odom_pos[i] is not None]
        if not pts:
            return None
        x = sum(p[0] for p in pts) / len(pts)
        y = sum(p[1] for p in pts) / len(pts)
        z = sum(p[2] for p in pts) / len(pts)
        return (x, y, z)

    def _snapshot_positions(self):
        return {i: self.odom_pos[i] for i in range(1, self.num_uav + 1)}

    def _segment_invalid_displacement(self, start_pos):
        total = 0.0
        for i in range(1, self.num_uav + 1):
            p0 = start_pos.get(i)
            p1 = self.odom_pos.get(i)
            if p0 is None or p1 is None:
                continue
            straight = math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2 + (p1[2] - p0[2]) ** 2)
            path = max(0.0, self._path_len[i])
            total += max(0.0, path - straight)
        return total

    def _reset_path_acc(self):
        for i in range(1, self.num_uav + 1):
            self._path_len[i] = 0.0
            self._last_pos[i] = self.odom_pos[i]

    def _clear_task(self, rounds=4):
        for _ in range(rounds):
            self.clear_pub.publish(Empty())
            self._spin_step(0.05)

    def _publish_formation(self, name: str):
        m = String()
        m.data = name
        self.form_pub.publish(m)

    def _publish_random_mission(self, rng: random.Random):
        c = self._centroid()
        if c is None:
            return None
        # 高频干预下在当前位置附近随机给新目标
        goal = Point()
        goal.x = float(c[0] + rng.uniform(-35.0, 35.0))
        goal.y = float(c[1] + rng.uniform(-35.0, 35.0))
        goal.z = 0.0
        self.mission_pub.publish(goal)
        return (goal.x, goal.y)

    def _set_mission_speed_5mps(self):
        get_cli = self.create_client(GetParameters, "/formation_commander/get_parameters")
        set_cli = self.create_client(SetParameters, "/formation_commander/set_parameters")
        if not get_cli.wait_for_service(timeout_sec=5.0) or not set_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("参数服务不可用，跳过速度设置")
            return False

        get_req = GetParameters.Request()
        get_req.names = ["control_rate", "mission_step_xy"]
        f1 = get_cli.call_async(get_req)
        rclpy.spin_until_future_complete(self, f1, timeout_sec=5.0)
        res1 = f1.result()
        if res1 is None or len(res1.values) < 2:
            return False

        control_rate = float(res1.values[0].double_value)
        if control_rate < 0.5:
            control_rate = 6.0
        new_step = 5.0 / control_rate

        set_req = SetParameters.Request()
        p = Parameter("mission_step_xy", Parameter.Type.DOUBLE, float(new_step))
        set_req.parameters = [p.to_parameter_msg()]
        f2 = set_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, f2, timeout_sec=5.0)
        res2 = f2.result()
        return res2 is not None and bool(res2.results) and res2.results[0].successful

    def _ensure_ready(self):
        ok = self._spin_until(
            lambda: all(self.odom_pos[i] is not None for i in range(1, self.num_uav + 1)) and self.phase != "unknown",
            timeout_s=50,
        )
        if not ok:
            raise RuntimeError("状态未就绪")

    def _ensure_airborne(self):
        ok = self._spin_until(
            lambda: all(self.odom_pos[i] is not None and self.odom_pos[i][2] > 1.2 for i in range(1, self.num_uav + 1)),
            timeout_s=8.0,
        )
        if not ok:
            raise RuntimeError("当前测试要求无人机已在空中")

    def _ensure_line_x(self):
        if self.current_formation == "line_x" and self.phase == "idle":
            return
        self._publish_formation("line_x")

        saw_active = {"v": False}

        def done():
            if self.phase in ("lift", "xy", "merge"):
                saw_active["v"] = True
            return saw_active["v"] and self.phase == "idle" and self.current_formation == "line_x"

        ok = self._spin_until(done, timeout_s=200)
        if not ok:
            raise RuntimeError("切换到 line_x 失败")

    def _start_north_cruise(self):
        c = self._centroid()
        if c is None:
            raise RuntimeError("无法计算编队中心")
        goal = Point()
        goal.x = float(c[0])
        goal.y = float(c[1] + 300.0)
        goal.z = 0.0
        for _ in range(4):
            self.mission_pub.publish(goal)
            self._spin_step(0.05)
        self._spin_until(lambda: self.mission_active, timeout_s=4.0)

    def _cooldown_to_idle(self):
        self._clear_task(rounds=5)
        self._publish_formation("line_x")
        self._spin_until(lambda: self.phase == "idle", timeout_s=self.settle_timeout_s)

    def _run_single_trial(self, mechanism: str, freq_hz: float, rng: random.Random):
        period = 1.0 / freq_hz
        formations = ["line_x", "line_y", "v_shape", "t_shape"]

        self._ensure_line_x()
        self._start_north_cruise()

        records = []
        current_cmd = None

        t_start = time.time()
        t_next = t_start

        while time.time() - t_start < self.intervention_duration_s:
            now = time.time()
            if now >= t_next:
                if current_cmd is not None:
                    current_cmd["invalid_displacement_m"] = self._segment_invalid_displacement(current_cmd["start_pos"])
                    records.append(current_cmd)

                self._reset_path_acc()
                baseline = self._baseline_state()

                is_form_cmd = (rng.random() < 0.5)
                cmd_desc = {}
                if is_form_cmd:
                    target_form = rng.choice(formations)
                    cmd_desc = {"type": "formation", "value": target_form}
                    if mechanism == "return_origin":
                        self._publish_formation("origin")
                        self._spin_step(0.01)
                    self._publish_formation(target_form)
                else:
                    goal = self._publish_random_mission(rng)
                    cmd_desc = {"type": "mission", "value": goal}
                    if mechanism == "return_origin":
                        self._publish_formation("origin")
                        self._spin_step(0.01)
                        if goal is not None:
                            pt = Point(); pt.x = goal[0]; pt.y = goal[1]; pt.z = 0.0
                            self.mission_pub.publish(pt)

                current_cmd = {
                    "sent_at": now,
                    "baseline": baseline,
                    "response_latency_s": float("nan"),
                    "responded": False,
                    "deadlock": False,
                    "invalid_displacement_m": float("nan"),
                    "start_pos": self._snapshot_positions(),
                    "cmd": cmd_desc,
                }
                t_next += period

            if current_cmd is not None and not current_cmd["responded"]:
                if self._state_changed_since(current_cmd["baseline"]):
                    current_cmd["responded"] = True
                    current_cmd["response_latency_s"] = time.time() - current_cmd["sent_at"]
                elif time.time() - current_cmd["sent_at"] > self.deadlock_timeout_s:
                    current_cmd["deadlock"] = True

            self._spin_step(0.01)

        if current_cmd is not None:
            current_cmd["invalid_displacement_m"] = self._segment_invalid_displacement(current_cmd["start_pos"])
            records.append(current_cmd)

        # 统计
        lats = [r["response_latency_s"] for r in records if r["responded"] and isinstance(r["response_latency_s"], float) and not math.isnan(r["response_latency_s"])]
        invs = [r["invalid_displacement_m"] for r in records if isinstance(r["invalid_displacement_m"], float) and not math.isnan(r["invalid_displacement_m"])]
        deadlocks = [r for r in records if r["deadlock"]]

        result = {
            "mechanism": mechanism,
            "frequency_hz": freq_hz,
            "command_count": len(records),
            "avg_command_response_latency_s": (sum(lats) / len(lats)) if lats else float("nan"),
            "avg_single_command_invalid_displacement_m": (sum(invs) / len(invs)) if invs else float("nan"),
            "state_deadlock_rate_percent": (100.0 * len(deadlocks) / len(records)) if records else float("nan"),
            "responded_count": len(lats),
            "deadlock_count": len(deadlocks),
        }
        return result

    def run_compare(self):
        self.get_logger().info("等待状态就绪...")
        self._ensure_ready()
        self._ensure_airborne()
        self._set_mission_speed_5mps()

        rng = random.Random(self.random_seed)

        all_results = []
        for freq in self.freq_list_hz:
            for mechanism in ("snapshot_replan", "return_origin_replan"):
                self.get_logger().info(f"开始对比试验: {mechanism}, {freq:.1f}Hz")
                self._cooldown_to_idle()
                trial = self._run_single_trial(
                    mechanism="return_origin" if mechanism == "return_origin_replan" else "snapshot",
                    freq_hz=freq,
                    rng=rng,
                )
                trial["mechanism"] = mechanism
                all_results.append(trial)
                self.get_logger().info(
                    f"完成: {mechanism} {freq:.1f}Hz | 时延={trial['avg_command_response_latency_s']:.3f}s "
                    f"无效位移={trial['avg_single_command_invalid_displacement_m']:.3f}m "
                    f"死锁率={trial['state_deadlock_rate_percent']:.2f}%"
                )

        return {
            "test_name": "测试3_高频连续干预机制对比",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "config": {
                "frequencies_hz": self.freq_list_hz,
                "intervention_duration_s": self.intervention_duration_s,
                "deadlock_timeout_s": self.deadlock_timeout_s,
                "random_seed": self.random_seed,
                "intervention_types": ["formation", "mission_center"],
            },
            "results": all_results,
        }


def main():
    rclpy.init()
    node = StressReplanCompare(num_uav=6)
    try:
        result = node.run_compare()
        print("\n=== 测试3结果 ===")
        print(json.dumps(result, ensure_ascii=False, indent=2))

        out_path = "/home/cookie/ros2_ws/src/swarm_control/tools/test3_result.json"
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(f"\n结果已保存: {out_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
