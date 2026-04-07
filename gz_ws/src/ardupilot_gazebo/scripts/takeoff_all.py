#!/usr/bin/env python3

import argparse
import os
import sys
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray


@dataclass
class UavState:
    mode: Optional[str] = None
    armed: Optional[bool] = None
    pose_altitude: Optional[float] = None
    odom_altitude: Optional[float] = None
    last_mode_cmd: float = -1e9
    last_arm_cmd: float = -1e9
    last_takeoff_cmd: float = -1e9
    last_velocity_cmd: float = -1e9
    arm_rise_time: float = -1e9
    takeoff_start_time: float = -1e9
    takeoff_burst_remaining: int = 0
    zero_sent: bool = False
    last_rc_override_cmd: float = -1e9
    rc_override_released: bool = False


class SwarmTakeoffCoordinator(Node):

    def __init__(
        self,
        num_uav: int,
        altitude: float,
        success_altitude: float,
        total_timeout: float,
        climb_speed: float,
        climb_assist_delay: float,
        target_altitude_tolerance: float,
        sequential_takeoff: bool,
        use_rc_override: bool,
        overshoot_margin: float,
    ):
        super().__init__("swarm_takeoff_coordinator")
        self.num_uav = num_uav
        self.altitude = altitude
        self.success_altitude = success_altitude
        self.total_timeout = total_timeout
        self.climb_speed = climb_speed
        self.climb_assist_delay = climb_assist_delay
        self.target_altitude_tolerance = target_altitude_tolerance
        self.overshoot_margin = overshoot_margin
        self.sequential_takeoff = sequential_takeoff
        self.use_rc_override = use_rc_override
        self.start_time = self._now()
        self.last_progress_log = self.start_time

        self.mode_period = 0.40
        self.arm_period = 0.55
        self.takeoff_period = 0.75
        self.velocity_period = 0.15
        self.mode_recent_window = 1.5
        self.takeoff_burst_period = 0.25
        self.takeoff_burst_delay = 0.6
        self.takeoff_burst_count = 3
        self.rc_override_period = 0.20
        self.rc_override_idle_throttle_pwm = 1000
        self.rc_override_takeoff_throttle_pwm = 1600
        self.progress_period = 1.0
        self.control_period = 0.10

        self.states: List[UavState] = [UavState() for _ in range(self.num_uav)]
        self.mode_pubs = []
        self.arm_pubs = []
        self.takeoff_pubs = []
        self.velocity_pubs = []
        self.rc_override_pubs = []

        for index in range(self.num_uav):
            ns = f"/uav{index + 1}"
            self.create_subscription(String, f"{ns}/uav/mode", lambda msg, idx=index: self._mode_cb(idx, msg), 10)
            self.create_subscription(Bool, f"{ns}/uav/armed", lambda msg, idx=index: self._armed_cb(idx, msg), 10)
            self.create_subscription(PoseStamped, f"{ns}/uav/pose", lambda msg, idx=index: self._pose_cb(idx, msg), 10)
            self.create_subscription(Odometry, f"{ns}/uav/odom", lambda msg, idx=index: self._odom_cb(idx, msg), 10)
            self.mode_pubs.append(self.create_publisher(String, f"{ns}/uav/cmd/mode", 10))
            self.arm_pubs.append(self.create_publisher(Bool, f"{ns}/uav/cmd/arm", 10))
            self.takeoff_pubs.append(self.create_publisher(Float32, f"{ns}/uav/cmd/takeoff", 10))
            self.velocity_pubs.append(self.create_publisher(TwistStamped, f"{ns}/uav/cmd/velocity", 10))
            self.rc_override_pubs.append(self.create_publisher(UInt16MultiArray, f"{ns}/uav/cmd/rc_override", 10))

        self.finished = False
        self.success = False
        self.timer = self.create_timer(self.control_period, self._control_loop)
        self.get_logger().info(
            f"Swarm takeoff coordinator started: num_uav={self.num_uav}, altitude={self.altitude:.2f}m, success_altitude={self.success_altitude:.2f}m, climb_speed={self.climb_speed:.2f}m/s, sequential_takeoff={self.sequential_takeoff}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _mode_cb(self, index: int, msg: String):
        self.states[index].mode = msg.data.strip().upper()

    def _armed_cb(self, index: int, msg: Bool):
        state = self.states[index]
        armed_now = bool(msg.data)
        if armed_now and state.armed is not True:
            state.arm_rise_time = self._now()
            state.takeoff_start_time = -1e9
            state.takeoff_burst_remaining = self.takeoff_burst_count
        if (not armed_now) and state.armed is True:
            state.takeoff_start_time = -1e9
            state.takeoff_burst_remaining = self.takeoff_burst_count
        state.armed = armed_now

    def _pose_cb(self, index: int, msg: PoseStamped):
        self.states[index].pose_altitude = float(msg.pose.position.z)

    def _odom_cb(self, index: int, msg: Odometry):
        self.states[index].odom_altitude = float(msg.pose.pose.position.z)

    def _altitude(self, state: UavState) -> Optional[float]:
        if state.odom_altitude is not None:
            return state.odom_altitude
        return state.pose_altitude

    def _is_ready(self, state: UavState) -> bool:
        return state.mode is not None and state.armed is not None and self._altitude(state) is not None

    def _is_airborne(self, state: UavState) -> bool:
        altitude = self._altitude(state)
        return altitude is not None and altitude >= self.success_altitude

    def _target_reached(self, state: UavState) -> bool:
        target_altitude = max(self.success_altitude, self.altitude - self.target_altitude_tolerance)
        altitude = self._altitude(state)
        return altitude is not None and altitude >= target_altitude

    def _publish_mode(self, index: int):
        msg = String()
        msg.data = "GUIDED"
        self.mode_pubs[index].publish(msg)
        self.states[index].last_mode_cmd = self._now()

    def _publish_arm(self, index: int):
        msg = Bool()
        msg.data = True
        self.arm_pubs[index].publish(msg)
        self.states[index].last_arm_cmd = self._now()

    def _publish_takeoff(self, index: int):
        msg = Float32()
        msg.data = float(self.altitude)
        self.takeoff_pubs[index].publish(msg)
        now = self._now()
        self.states[index].last_takeoff_cmd = now
        if self.states[index].takeoff_start_time < -1e8:
            self.states[index].takeoff_start_time = now

    def _publish_velocity(self, index: int, vertical_speed: float):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = float(vertical_speed)
        msg.twist.angular.z = 0.0
        self.velocity_pubs[index].publish(msg)
        self.states[index].last_velocity_cmd = self._now()

    def _publish_zero_velocity(self, index: int):
        self._publish_velocity(index, 0.0)
        self.states[index].zero_sent = True

    def _publish_rc_override(self, index: int, throttle_pwm: Optional[int] = None, release: bool = False):
        msg = UInt16MultiArray()
        if release:
            msg.data = [65535] * 8
        else:
            pwm = self.rc_override_idle_throttle_pwm if throttle_pwm is None else int(throttle_pwm)
            msg.data = [1500, 1500, pwm, 1500, 65535, 65535, 65535, 65535]
        self.rc_override_pubs[index].publish(msg)
        state = self.states[index]
        state.last_rc_override_cmd = self._now()
        state.rc_override_released = release

    def _should_send(self, last_sent: float, period: float) -> bool:
        return (self._now() - last_sent) >= period

    def _pending_summary(self) -> str:
        pending = []
        for index, state in enumerate(self.states):
            if self._target_reached(state):
                continue
            pending.append(
                f"uav{index + 1}(mode={state.mode or 'unknown'},armed={state.armed},alt={self._altitude(state) if self._altitude(state) is not None else 'unknown'})"
            )
        return ", ".join(pending)

    def _final_report(self):
        for index, state in enumerate(self.states):
            altitude = self._altitude(state)
            self.get_logger().info(
                f"uav{index + 1}: mode={state.mode or 'unknown'} armed={state.armed} alt={altitude if altitude is not None else 'unknown'}"
            )

    def _control_loop(self):
        now = self._now()
        ready_count = sum(1 for state in self.states if self._is_ready(state))
        airborne_count = sum(1 for state in self.states if self._is_airborne(state))
        target_count = sum(1 for state in self.states if self._target_reached(state))

        if target_count == self.num_uav:
            self.success = True
            self.finished = True
            self.get_logger().info("All UAVs reached target altitude")
            self._final_report()
            return

        if now - self.start_time > self.total_timeout:
            self.success = False
            self.finished = True
            self.get_logger().error("Takeoff timeout reached")
            self._final_report()
            return

        if now - self.last_progress_log >= self.progress_period:
            self.last_progress_log = now
            self.get_logger().info(
                f"Progress: ready={ready_count}/{self.num_uav}, airborne={airborne_count}/{self.num_uav}, target={target_count}/{self.num_uav}; pending={self._pending_summary()}"
            )

        active_takeoff_index = None
        if self.sequential_takeoff:
            for i, state in enumerate(self.states):
                if not self._target_reached(state):
                    active_takeoff_index = i
                    break

        for index, state in enumerate(self.states):
            altitude = self._altitude(state)
            if self._target_reached(state):
                if not state.zero_sent and self._should_send(state.last_velocity_cmd, self.velocity_period):
                    self._publish_zero_velocity(index)
                if self.use_rc_override and (not state.rc_override_released) and self._should_send(state.last_rc_override_cmd, self.rc_override_period):
                    self._publish_rc_override(index, release=True)
                continue
            state.zero_sent = False
            state.rc_override_released = False

            can_attempt_takeoff_now = (not self.sequential_takeoff) or (index == active_takeoff_index)

            if self.use_rc_override and can_attempt_takeoff_now and self._should_send(state.last_rc_override_cmd, self.rc_override_period):
                throttle_pwm = self.rc_override_takeoff_throttle_pwm if state.armed is True else self.rc_override_idle_throttle_pwm
                self._publish_rc_override(index, throttle_pwm=throttle_pwm, release=False)

            if state.mode != "GUIDED" and self._should_send(state.last_mode_cmd, self.mode_period):
                self._publish_mode(index)

            if state.armed is not True and self._should_send(state.last_arm_cmd, self.arm_period):
                self._publish_arm(index)

            guided_recent = (now - state.last_mode_cmd) <= self.mode_recent_window
            can_takeoff = state.armed is True and (state.mode == "GUIDED" or guided_recent)

            overshoot = (
                altitude is not None
                and altitude > (self.altitude + self.overshoot_margin)
            )
            if overshoot and self._should_send(state.last_velocity_cmd, self.velocity_period):
                # Safety clamp: if one vehicle climbs abnormally high, stop vertical assist/takeoff burst.
                self._publish_zero_velocity(index)

            if (not overshoot) and can_takeoff and can_attempt_takeoff_now and self._should_send(state.last_takeoff_cmd, self.takeoff_period):
                self._publish_takeoff(index)

            # After arm rising edge, send a short burst of takeoff commands to
            # survive occasional command loss and state topic lag.
            should_burst_takeoff = (
                (not overshoot)
                and can_takeoff
                and can_attempt_takeoff_now
                and state.takeoff_burst_remaining > 0
                and (now - state.arm_rise_time) >= self.takeoff_burst_delay
                and self._should_send(state.last_takeoff_cmd, self.takeoff_burst_period)
            )
            if should_burst_takeoff:
                self._publish_takeoff(index)
                state.takeoff_burst_remaining -= 1

            should_climb = (
                (not overshoot)
                and can_takeoff
                and can_attempt_takeoff_now
                and state.armed is True
                and altitude is not None
                and altitude < (self.altitude - self.target_altitude_tolerance)
                and (now - max(state.takeoff_start_time, state.arm_rise_time)) >= self.climb_assist_delay
            )
            if should_climb and self._should_send(state.last_velocity_cmd, self.velocity_period):
                self._publish_velocity(index, self.climb_speed)
            elif self.sequential_takeoff and not self._target_reached(state):
                if self._should_send(state.last_velocity_cmd, self.velocity_period):
                    self._publish_zero_velocity(index)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Swarm takeoff coordinator")
    parser.add_argument("altitude", nargs="?", type=float, default=5.0)
    parser.add_argument("--num-uav", type=int, default=6)
    parser.add_argument("--success-altitude", type=float, default=0.7)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--climb-speed", type=float, default=0.9)
    parser.add_argument("--climb-assist-delay", type=float, default=2.0)
    parser.add_argument("--target-altitude-tolerance", type=float, default=0.5)
    parser.add_argument("--overshoot-margin", type=float, default=1.5)
    parser.add_argument("--sequential-takeoff", action="store_true", default=False)
    parser.add_argument("--parallel-takeoff", action="store_true", default=False)
    parser.add_argument("--internal-rc-override", action="store_true", default=False)
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = SwarmTakeoffCoordinator(
        args.num_uav,
        args.altitude,
        args.success_altitude,
        args.timeout,
        args.climb_speed,
        args.climb_assist_delay,
        args.target_altitude_tolerance,
        sequential_takeoff=(False if args.parallel_takeoff else True),
        use_rc_override=args.internal_rc_override or (os.environ.get("TAKEOFF_INTERNAL_RC_OVERRIDE", "0") == "1"),
        overshoot_margin=args.overshoot_margin,
    )
    result = 1
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = 0 if node.success else 1
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted")
        node.success = False
        result = 1
    except ExternalShutdownException:
        result = 0 if node.success else 1
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return result


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))