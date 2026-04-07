#!/usr/bin/env python3
import itertools
import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String


class RealMeasurement(Node):
    def __init__(self, num_uav=6, target_formation='v_shape', takeoff_alt=5.0):
        super().__init__('real_measurement_runner')
        self.num_uav = num_uav
        self.target_formation = target_formation
        self.takeoff_alt = float(takeoff_alt)

        self.pos = {i: None for i in range(1, num_uav + 1)}
        self.last_pos = {i: None for i in range(1, num_uav + 1)}
        self.last_t = {i: None for i in range(1, num_uav + 1)}
        self.speed_sum = 0.0
        self.speed_count = 0

        self.status = {}
        self.phase = 'unknown'

        self.hover_events = 0
        self.prev_hover_all = False

        self.min_pair_dist_xy = float('inf')

        self.collision_threshold = 0.35
        self.collided = False

        self.exp_t0 = None
        self.exp_t1 = None

        self.clear_pub = self.create_publisher(Empty, '/swarm/clear_task', 10)
        self.form_pub = self.create_publisher(String, '/swarm/formation', 10)
        self.mission_pub = self.create_publisher(Point, '/swarm/mission_center', 10)

        self.arm_pubs = {}
        self.mode_pubs = {}
        self.takeoff_pubs = {}

        for i in range(1, num_uav + 1):
            self.arm_pubs[i] = self.create_publisher(Bool, f'/uav{i}/uav/cmd/arm', 10)
            self.mode_pubs[i] = self.create_publisher(String, f'/uav{i}/uav/cmd/mode', 10)
            self.takeoff_pubs[i] = self.create_publisher(Float32, f'/uav{i}/uav/cmd/takeoff', 10)
            self.create_subscription(Odometry, f'/uav{i}/uav/odom', self._mk_odom_cb(i), 20)

        self.create_subscription(String, '/swarm/status', self._on_status, 20)

    def _mk_odom_cb(self, did):
        def _cb(msg: Odometry):
            now = time.time()
            p = msg.pose.pose.position
            cur = (float(p.x), float(p.y), float(p.z))
            self.pos[did] = cur

            if self.last_pos[did] is not None and self.last_t[did] is not None:
                dt = max(1e-3, now - self.last_t[did])
                dx = cur[0] - self.last_pos[did][0]
                dy = cur[1] - self.last_pos[did][1]
                dz = cur[2] - self.last_pos[did][2]
                v = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
                if self.exp_t0 is not None and self.exp_t1 is None:
                    self.speed_sum += v
                    self.speed_count += 1

            self.last_pos[did] = cur
            self.last_t[did] = now

            self._update_pair_distance_and_collision()
            self._update_hover_events()

        return _cb

    def _on_status(self, msg: String):
        try:
            self.status = json.loads(msg.data)
            self.phase = self.status.get('phase', 'unknown')
        except Exception:
            self.phase = 'decode_error'

    def _update_pair_distance_and_collision(self):
        ids = [i for i in range(1, self.num_uav + 1) if self.pos[i] is not None]
        if len(ids) < 2:
            return
        for i, j in itertools.combinations(ids, 2):
            dx = self.pos[i][0] - self.pos[j][0]
            dy = self.pos[i][1] - self.pos[j][1]
            dxy = math.hypot(dx, dy)
            if dxy < self.min_pair_dist_xy:
                self.min_pair_dist_xy = dxy
            if dxy < self.collision_threshold:
                self.collided = True

    def _update_hover_events(self):
        if self.exp_t0 is None or self.exp_t1 is not None:
            return
        ids = [i for i in range(1, self.num_uav + 1) if self.pos[i] is not None and self.last_pos[i] is not None and self.last_t[i] is not None]
        if len(ids) < self.num_uav:
            return

        # 用最近一次位移近似速度，定义全队悬停: 所有无人机速度<0.15m/s 且高度>1m
        hover_all = True
        now = time.time()
        for i in ids:
            dt = max(1e-3, now - self.last_t[i])
            dx = self.pos[i][0] - self.last_pos[i][0]
            dy = self.pos[i][1] - self.last_pos[i][1]
            dz = self.pos[i][2] - self.last_pos[i][2]
            v = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            if not (v < 0.15 and self.pos[i][2] > 1.0):
                hover_all = False
                break

        if hover_all and not self.prev_hover_all:
            self.hover_events += 1
        self.prev_hover_all = hover_all

    def _spin_until(self, cond, timeout_s, step_s=0.05):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            rclpy.spin_once(self, timeout_sec=step_s)
            if cond():
                return True
        return False

    def wait_topics_ready(self):
        self.get_logger().info('等待 odom 数据...')
        ok = self._spin_until(lambda: all(self.pos[i] is not None for i in range(1, self.num_uav + 1)), timeout_s=40)
        if not ok:
            raise RuntimeError('未在超时内收到全部 odom 数据')

    def clear_task(self):
        for _ in range(4):
            self.clear_pub.publish(Empty())
            self._spin_until(lambda: False, timeout_s=0.1)

    def takeoff_all(self):
        self.get_logger().info('下发 GUIDED + ARM + TAKEOFF...')
        for _ in range(8):
            for i in range(1, self.num_uav + 1):
                mode_msg = String(); mode_msg.data = 'GUIDED'
                arm_msg = Bool(); arm_msg.data = True
                tk_msg = Float32(); tk_msg.data = self.takeoff_alt
                self.mode_pubs[i].publish(mode_msg)
                self.arm_pubs[i].publish(arm_msg)
                self.takeoff_pubs[i].publish(tk_msg)
            self._spin_until(lambda: False, timeout_s=0.25)

        self.get_logger().info('等待全体达到起飞高度...')
        ok = self._spin_until(
            lambda: all(self.pos[i] is not None and self.pos[i][2] > 1.2 for i in range(1, self.num_uav + 1)),
            timeout_s=80,
        )
        if not ok:
            raise RuntimeError('起飞未在超时内完成')

    def run_formation_experiment(self):
        cmd = String(); cmd.data = self.target_formation
        self.exp_t0 = time.time()
        self.form_pub.publish(cmd)

        self.get_logger().info(f'已下发编队切换: {self.target_formation}，等待 phase 回到 idle...')
        saw_active = False

        def done():
            nonlocal saw_active
            if self.phase in ('lift', 'xy', 'merge'):
                saw_active = True
            return saw_active and self.phase == 'idle'

        ok = self._spin_until(done, timeout_s=180)
        if not ok:
            raise RuntimeError(f'编队切换未在超时内完成，当前 phase={self.phase}')

        self.exp_t1 = time.time()

    def print_result(self):
        total_time = (self.exp_t1 - self.exp_t0) if (self.exp_t0 is not None and self.exp_t1 is not None) else float('nan')
        avg_speed = self.speed_sum / self.speed_count if self.speed_count > 0 else 0.0
        min_dist = self.min_pair_dist_xy if self.min_pair_dist_xy < 1e9 else float('nan')
        collision_rate = 100.0 if self.collided else 0.0

        print('\n=== 实测结果（单次） ===')
        print(f'编队完成时间（s）      : {total_time:.2f}')
        print(f'平均飞行速度（m/s）    : {avg_speed:.2f}')
        print(f'全程悬停次数            : {self.hover_events}')
        print(f'最小机间平面距离（m）  : {min_dist:.2f}')
        print(f'碰撞率（10次实验）      : {collision_rate:.0f}%  (当前仅1次，需10次重复)')


def main():
    rclpy.init()
    node = RealMeasurement(num_uav=6, target_formation='v_shape', takeoff_alt=5.0)
    try:
        node.wait_topics_ready()
        node.clear_task()
        node.takeoff_all()
        node.run_formation_experiment()
        node.print_result()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
