#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 -> MAVLink command sender for ArduPilot.

Topics (recommended defaults):
  /uav/cmd/arm         std_msgs/Bool                 True=arm, False=disarm
  /uav/cmd/mode        std_msgs/String               ArduPilot mode (e.g., GUIDED)
  /uav/cmd/takeoff     std_msgs/Float32              takeoff altitude (meters)
  /uav/cmd/land        std_msgs/Empty                land now
  /uav/cmd/velocity    geometry_msgs/TwistStamped    ENU m/s + yaw_rate in angular.z
  /uav/cmd/waypoint    sensor_msgs/NavSatFix         lat/lon/alt (alt in meters)
  /uav/cmd/attitude    geometry_msgs/QuaternionStamped  attitude setpoint
  /uav/cmd/thrust      std_msgs/Float32              0.0-1.0
  /uav/cmd/move_relative geometry_msgs/Vector3       body move (x=right,y=fwd,z=up, meters)
  /uav/cmd/move_relative_yaw geometry_msgs/Twist     body move + relative yaw (angular.z, rad)
  /uav/cmd/goto_enu    geometry_msgs/Vector3          ENU absolute local position (x=east,y=north,z=up)
  /uav/cmd/gimbal_target geometry_msgs/Vector3       pitch/roll/yaw in degrees (x=pitch,y=roll,z=yaw)

Notes:
    - This node sends MAVLink commands and also publishes telemetry from the same link.
  - Velocity input assumes ENU; converted to NED for MAVLink.
  - Attitude quaternion is sent as-is; provide NED-frame quaternion if required.
  - Error flag is logged and also published to /uav/tx_error.
"""

import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32, Empty, UInt16MultiArray, UInt8
from geometry_msgs.msg import TwistStamped, QuaternionStamped, Vector3, Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Image, BatteryState
from nav_msgs.msg import Odometry

try:
    from PIL import Image as PILImage  # Optional: enables JPEG/PNG saving
except ImportError:  # pragma: no cover - optional dependency
    PILImage = None

from pymavlink import mavutil

from uav_bridge.compat import get_compat
from uav_bridge.mavlink_bridge import (
    decode_copter_mode,
    ecef_to_enu,
    euler_to_quaternion,
    is_valid_gps_fix,
    llh_to_ecef,
    quaternion_to_euler,
)


def clamp(val: float, lo: float, hi: float) -> float:
    """限幅工具：把 val 约束到 [lo, hi]。"""
    return max(lo, min(hi, val))


class MavlinkTxNode(Node):
    def __init__(self):
        super().__init__("mavlink_tx")

        self._compat = get_compat()

        d = self._compat.node_parameters_defaults
        self.declare_parameter("mavlink_url", d["mavlink_url"])
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("waypoint_relative_alt", d["waypoint_relative_alt"])
        self.declare_parameter("default_takeoff_alt", d["default_takeoff_alt"])
        self.declare_parameter("default_thrust", d["default_thrust"])
        self.declare_parameter("gimbal_mount_mode", d["gimbal_mount_mode"])
        self.declare_parameter(
            "digicam_command", int(d["digicam_command"])
        )
        self.declare_parameter("digicam_param5_trigger", d["digicam_param5_trigger"])
        self.declare_parameter("screenshot_enable_save", d["screenshot_enable_save"])
        self.declare_parameter("screenshot_image_topic", d["screenshot_image_topic"])
        self.declare_parameter("screenshot_output_dir", d["screenshot_output_dir"])
        self.declare_parameter("screenshot_filename_format", d["screenshot_filename_format"])

        mavlink_url = (
            self.get_parameter("mavlink_url")
            .get_parameter_value()
            .string_value
        )
        self._target_system = int(
            self.get_parameter("target_system")
            .get_parameter_value()
            .integer_value
        )
        self._target_component = int(
            self.get_parameter("target_component")
            .get_parameter_value()
            .integer_value
        )
        self._waypoint_relative_alt = (
            self.get_parameter("waypoint_relative_alt")
            .get_parameter_value()
            .bool_value
        )
        self._default_takeoff_alt = (
            self.get_parameter("default_takeoff_alt")
            .get_parameter_value()
            .double_value
        )
        self._thrust = (
            self.get_parameter("default_thrust")
            .get_parameter_value()
            .double_value
        )
        self._gimbal_mount_mode = int(
            self.get_parameter("gimbal_mount_mode")
            .get_parameter_value()
            .integer_value
        )
        self._digicam_command = int(
            self.get_parameter("digicam_command")
            .get_parameter_value()
            .integer_value
        )
        self._digicam_trigger = (
            self.get_parameter("digicam_param5_trigger")
            .get_parameter_value()
            .double_value
        )
        self._screenshot_enable_save = (
            self.get_parameter("screenshot_enable_save")
            .get_parameter_value()
            .bool_value
        )
        self._screenshot_image_topic = (
            self.get_parameter("screenshot_image_topic")
            .get_parameter_value()
            .string_value
        )
        self._screenshot_output_dir = Path(
            self.get_parameter("screenshot_output_dir")
            .get_parameter_value()
            .string_value
        ).expanduser()
        self._screenshot_filename_format = (
            self.get_parameter("screenshot_filename_format")
            .get_parameter_value()
            .string_value
        )

        # Screenshot cache/counter
        self._latest_image = None
        self._shot_counter = 1

        # 对外暴露的发送异常标志，便于上层节点做降级处理。
        self._error_flag = False
        self._error_pub = self.create_publisher(Bool, "uav/tx_error", self._compat.qos_profile_cmd)

        self.mode_pub = self.create_publisher(String, "uav/mode", 10)
        self.armed_pub = self.create_publisher(Bool, "uav/armed", 10)
        self.sys_status_pub = self.create_publisher(UInt8, "uav/system_status", 10)
        self.navsat_pub = self.create_publisher(NavSatFix, "uav/navsatfix", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "uav/pose", 10)
        self.odom_pub = self.create_publisher(Odometry, "uav/odom", 10)
        self.batt_pub = self.create_publisher(BatteryState, "uav/battery", 10)
        self.gimbal_angle_pub = self.create_publisher(Vector3, "uav/gimbal/angle", 10)

        self._gimbal_seen = False
        self._gimbal_warned = False
        self._startup_ns = self.get_clock().now().nanoseconds
        self._last_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_quat = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self._last_lin_vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_ang_vel = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._enu_ref_set = False
        self._ref_lat = 0.0
        self._ref_lon = 0.0
        self._ref_alt = 0.0
        self._ref_x0 = 0.0
        self._ref_y0 = 0.0
        self._ref_z0 = 0.0

        self.get_logger().info(f"[{self._compat.name}] Connecting to MAVLink at {mavlink_url}")
        try:
            self.master = mavutil.mavlink_connection(mavlink_url)
            self.master.wait_heartbeat(timeout=10)
            if self._target_system > 0:
                self.master.target_system = self._target_system
            if self._target_component > 0:
                self.master.target_component = self._target_component
            self.get_logger().info(
                f"[{self._compat.name}] Heartbeat from system (system {self.master.target_system}, "
                f"component {self.master.target_component})"
            )
            self.get_logger().info(
                f"[{self._compat.name}] Using MAVLink target system/component ({self.master.target_system}, {self.master.target_component})"
            )
            self._mode_mapping = self.master.mode_mapping() or {}
            self._telemetry_request_count = 0
            self._send_gcs_heartbeat()
            self._request_telemetry_streams()
        except Exception as exc:
            self.master = None
            self._mode_mapping = {}
            self._set_error(True, f"[{self._compat.name}] MAVLink connect failed: {exc}")

        self._link_timer = self.create_timer(1.0, self._maintain_mavlink_link)
        self._rx_timer = self.create_timer(0.02, self.read_mavlink)

        # command subscriptions
        self.create_subscription(Bool, "uav/cmd/arm", self.on_arm, 10)
        self.create_subscription(String, "uav/cmd/mode", self.on_mode, 10)
        self.create_subscription(Float32, "uav/cmd/takeoff", self.on_takeoff, 10)
        self.create_subscription(Empty, "uav/cmd/land", self.on_land, 10)
        self.create_subscription(TwistStamped, "uav/cmd/velocity", self.on_velocity, 10)
        self.create_subscription(NavSatFix, "uav/cmd/waypoint", self.on_waypoint, 10)
        self.create_subscription(QuaternionStamped, "uav/cmd/attitude", self.on_attitude, 10)
        self.create_subscription(Float32, "uav/cmd/thrust", self.on_thrust, 10)
        self.create_subscription(UInt16MultiArray, "uav/cmd/rc_override", self.on_rc_override, 10)
        self.create_subscription(Vector3, "uav/cmd/move_relative", self.on_move_relative, 10)
        self.create_subscription(Twist, "uav/cmd/move_relative_yaw", self.on_move_relative_yaw, 10)
        self.create_subscription(Vector3, "uav/cmd/goto_enu", self.on_goto_enu, 10)
        self.create_subscription(Vector3, "uav/cmd/gimbal_target", self.on_gimbal_target, 10)
        self.create_subscription(Empty, "uav/cmd/screenshot", self.on_screenshot, 10)
        if self._screenshot_enable_save:
            self.create_subscription(
                Image,
                self._screenshot_image_topic,
                self.on_image,
                self._compat.qos_profile_sensor,
            )
            if not self._compat.pil_supported or PILImage is None:
                self.get_logger().info(
                    "Pillow not available; screenshots will be saved as PPM/PGM."
                )

        # TODO: placeholders for future command topics (keep for later extension).
        # self.create_subscription(..., "uav/cmd/mission", self.on_mission, 10)
        # self.create_subscription(..., "uav/cmd/rc_override", self.on_rc_override, 10)

    def _maintain_mavlink_link(self):
        if not self._ensure_master():
            return
        self._send_gcs_heartbeat()
        if self._telemetry_request_count < 10:
            self._request_telemetry_streams()

    def _send_gcs_heartbeat(self):
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception as exc:
            self._set_error(True, f"GCS heartbeat send failed: {exc}")

    def _request_telemetry_streams(self):
        target_system = self._target_system
        target_component = self._target_component
        intervals_hz = {
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: 1.0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10.0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 20.0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
            mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 2.0,
            mavutil.mavlink.MAVLINK_MSG_ID_MOUNT_STATUS: 2.0,
        }

        for message_id, hz in intervals_hz.items():
            try:
                self.master.mav.command_long_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(message_id),
                    1e6 / hz,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"[{self._compat.name}] Failed to request MAVLink message interval for msg {message_id}: {exc}"
                )

        try:
            self.master.mav.request_data_stream_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1,
            )
            self._telemetry_request_count += 1
            if self._telemetry_request_count == 1:
                self.get_logger().info(
                    f"[{self._compat.name}] Requested MAVLink telemetry streams from TX link"
                )
        except Exception as exc:
            self.get_logger().warn(
                f"[{self._compat.name}] Failed to request MAVLink data stream from TX link: {exc}"
            )

    def read_mavlink(self):
        if not self._ensure_master():
            return

        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == "HEARTBEAT":
                self.handle_heartbeat(msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self.handle_global_position_int(msg)
            elif msg_type == "ATTITUDE":
                self.handle_attitude_telemetry(msg)
            elif msg_type == "BATTERY_STATUS":
                self.handle_battery_status(msg)
            elif msg_type == "SYS_STATUS":
                self.handle_sys_status(msg)
            elif msg_type == "MOUNT_STATUS":
                self.handle_mount_status(msg)
            elif msg_type == "GIMBAL_REPORT":
                self.handle_gimbal_report(msg)
            elif msg_type == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                self.handle_gimbal_device_attitude_status(msg)

        if not self._gimbal_seen and not self._gimbal_warned:
            elapsed_s = (self.get_clock().now().nanoseconds - self._startup_ns) / 1e9
            if elapsed_s > 5.0:
                self._gimbal_warned = True
                self.get_logger().warn(
                    "No gimbal telemetry yet (MOUNT_STATUS/GIMBAL_REPORT/"
                    "GIMBAL_DEVICE_ATTITUDE_STATUS). Check autopilot stream settings."
                )

    def handle_heartbeat(self, msg):
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        system_status = msg.system_status

        armed_flag = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode_str = decode_copter_mode(custom_mode)

        mode_msg = String()
        mode_msg.data = mode_str
        self.mode_pub.publish(mode_msg)

        armed_msg = Bool()
        armed_msg.data = armed_flag
        self.armed_pub.publish(armed_msg)

        status_msg = UInt8()
        status_msg.data = system_status
        self.sys_status_pub.publish(status_msg)

    def handle_global_position_int(self, msg):
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt_amsl = msg.alt / 1000.0

        navsat = NavSatFix()
        navsat.header.stamp = self.get_clock().now().to_msg()
        navsat.header.frame_id = "map"
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = alt_amsl
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_pub.publish(navsat)

        if not self._enu_ref_set and is_valid_gps_fix(lat, lon):
            self._ref_lat = lat
            self._ref_lon = lon
            self._ref_alt = alt_amsl
            self._ref_x0, self._ref_y0, self._ref_z0 = llh_to_ecef(lat, lon, alt_amsl)
            self._enu_ref_set = True
            self.get_logger().info(
                f"[{self._compat.name}] ENU reference set: lat0={lat:.7f}, lon0={lon:.7f}, alt0={alt_amsl:.2f}m"
            )
        elif not self._enu_ref_set:
            return

        x, y, z = llh_to_ecef(lat, lon, alt_amsl)
        e, n, u = ecef_to_enu(
            x, y, z,
            self._ref_lat, self._ref_lon,
            self._ref_x0, self._ref_y0, self._ref_z0,
        )
        self._last_pos["x"] = e
        self._last_pos["y"] = n
        self._last_pos["z"] = u

        vn = msg.vx / 100.0
        ve = msg.vy / 100.0
        vd = msg.vz / 100.0
        self._last_lin_vel["x"] = ve
        self._last_lin_vel["y"] = vn
        self._last_lin_vel["z"] = -vd

        self.publish_pose()
        self.publish_odom()

    def handle_attitude_telemetry(self, msg):
        qx, qy, qz, qw = euler_to_quaternion(msg.roll, msg.pitch, msg.yaw)
        self._last_quat["x"] = qx
        self._last_quat["y"] = qy
        self._last_quat["z"] = qz
        self._last_quat["w"] = qw
        self._last_ang_vel["x"] = msg.rollspeed
        self._last_ang_vel["y"] = msg.pitchspeed
        self._last_ang_vel["z"] = msg.yawspeed

        self.publish_pose()
        self.publish_odom()

    def handle_battery_status(self, msg):
        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()
        if msg.voltages and msg.voltages[0] != 0xFFFF:
            batt.voltage = msg.voltages[0] / 1000.0
        else:
            batt.voltage = 0.0
        batt.current = msg.current_battery / 100.0 if msg.current_battery != -1 else float("nan")
        batt.percentage = msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else float("nan")
        self.batt_pub.publish(batt)

    def handle_sys_status(self, msg):
        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()
        batt.voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 0xFFFF else 0.0
        batt.current = msg.current_battery / 100.0 if msg.current_battery != -1 else float("nan")
        batt.percentage = msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else float("nan")
        self.batt_pub.publish(batt)

    def handle_mount_status(self, msg):
        self._publish_gimbal_angle(
            float(msg.pointing_a) / 100.0,
            float(msg.pointing_b) / 100.0,
            float(msg.pointing_c) / 100.0,
            "MOUNT_STATUS",
        )

    def handle_gimbal_report(self, msg):
        self._publish_gimbal_angle(
            math.degrees(float(msg.joint_el)),
            math.degrees(float(msg.joint_roll)),
            math.degrees(float(msg.joint_az)),
            "GIMBAL_REPORT",
        )

    def handle_gimbal_device_attitude_status(self, msg):
        q = getattr(msg, "q", None)
        if q is None or len(q) < 4:
            return
        roll, pitch, yaw = quaternion_to_euler(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self._publish_gimbal_angle(
            math.degrees(pitch),
            math.degrees(roll),
            math.degrees(yaw),
            "GIMBAL_DEVICE_ATTITUDE_STATUS",
        )

    def _publish_gimbal_angle(self, pitch_deg: float, roll_deg: float, yaw_deg: float, source: str):
        out = Vector3()
        out.x = pitch_deg
        out.y = roll_deg
        out.z = yaw_deg
        self.gimbal_angle_pub.publish(out)
        if not self._gimbal_seen:
            self._gimbal_seen = True
            self.get_logger().info(f"[{self._compat.name}] Received gimbal angle from {source}")

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self._last_pos["x"]
        pose.pose.position.y = self._last_pos["y"]
        pose.pose.position.z = self._last_pos["z"]
        pose.pose.orientation.x = self._last_quat["x"]
        pose.pose.orientation.y = self._last_quat["y"]
        pose.pose.orientation.z = self._last_quat["z"]
        pose.pose.orientation.w = self._last_quat["w"]
        self.pose_pub.publish(pose)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._last_pos["x"]
        odom.pose.pose.position.y = self._last_pos["y"]
        odom.pose.pose.position.z = self._last_pos["z"]
        odom.pose.pose.orientation.x = self._last_quat["x"]
        odom.pose.pose.orientation.y = self._last_quat["y"]
        odom.pose.pose.orientation.z = self._last_quat["z"]
        odom.pose.pose.orientation.w = self._last_quat["w"]
        odom.twist.twist.linear.x = self._last_lin_vel["x"]
        odom.twist.twist.linear.y = self._last_lin_vel["y"]
        odom.twist.twist.linear.z = self._last_lin_vel["z"]
        odom.twist.twist.angular.x = self._last_ang_vel["x"]
        odom.twist.twist.angular.y = self._last_ang_vel["y"]
        odom.twist.twist.angular.z = self._last_ang_vel["z"]
        self.odom_pub.publish(odom)

    def _now_ms(self) -> int:
        # MAVLink time_boot_ms 是 uint32，这里取毫秒并做 32bit 环绕。
        return int((self.get_clock().now().nanoseconds / 1e6) % 4294967295)

    def _set_error(self, flag: bool, msg: str):
        # 仅在状态翻转时打印日志，避免重复刷屏。
        if flag and not self._error_flag:
            self.get_logger().error(f"TX_ERROR_FLAG=1 {msg}")
        elif not flag and self._error_flag:
            self.get_logger().info("TX_ERROR_FLAG=0")
        self._error_flag = flag
        out = Bool()
        out.data = flag
        self._error_pub.publish(out)

    def _ensure_master(self) -> bool:
        if self.master is None:
            self._set_error(True, "MAVLink master not connected")
            return False
        return True

    def _send_command_long(self, command: int, params):
        # COMMAND_LONG 固定 7 个浮点参数。
        if not self._ensure_master():
            return
        try:
            self.master.mav.command_long_send(
                self._target_system,
                self._target_component,
                command,
                0,
                params[0], params[1], params[2], params[3],
                params[4], params[5], params[6],
            )
        except Exception as exc:
            self._set_error(True, f"COMMAND_LONG failed: {exc}")

    def on_arm(self, msg: Bool):
        arm = 1.0 if msg.data else 0.0
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [arm, 0, 0, 0, 0, 0, 0],
        )

    def on_mode(self, msg: String):
        if not self._ensure_master():
            return
        mode = msg.data.strip().upper()
        if not mode:
            return
        if mode not in self._mode_mapping:
            self._set_error(True, f"Unknown mode: {mode}")
            return
        try:
            self.master.set_mode(self._mode_mapping[mode])
        except Exception as exc:
            self._set_error(True, f"SET_MODE failed: {exc}")

    def on_takeoff(self, msg: Float32):
        alt = msg.data if msg.data > 0.1 else self._default_takeoff_alt
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0, 0, 0, 0, 0, 0, float(alt)],
        )

    def on_land(self, _msg: Empty):
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0, 0, 0, 0, 0, 0, 0],
        )

    def on_velocity(self, msg: TwistStamped):
        if not self._ensure_master():
            return
        # ROS 常见 ENU 速度输入 -> MAVLink LOCAL_NED。
        vx = msg.twist.linear.y
        vy = msg.twist.linear.x
        vz = -msg.twist.linear.z
        yaw_rate = msg.twist.angular.z

        # 仅启用速度 + yaw_rate 控制，屏蔽位置/加速度/yaw 角。
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                self._now_ms(),
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0, 0.0, 0.0,
                float(vx), float(vy), float(vz),
                0.0, 0.0, 0.0,
                0.0, float(yaw_rate),
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_LOCAL_NED failed: {exc}")

    def on_waypoint(self, msg: NavSatFix):
        if not self._ensure_master():
            return
        # GLOBAL_INT 系列消息使用 1e7 缩放的整型经纬度。
        lat = int(msg.latitude * 1e7)
        lon = int(msg.longitude * 1e7)
        alt = float(msg.altitude)

        frame = (
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if self._waypoint_relative_alt
            else mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        )
        # 仅给位置目标，忽略速度/加速度/yaw/yaw_rate。
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_position_target_global_int_send(
                self._now_ms(),
                self._target_system,
                self._target_component,
                frame,
                type_mask,
                lat,
                lon,
                alt,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_GLOBAL_INT failed: {exc}")

    def on_attitude(self, msg: QuaternionStamped):
        if not self._ensure_master():
            return
        q = msg.quaternion
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        if math.isclose(qw, 0.0) and math.isclose(qx, 0.0) and math.isclose(qy, 0.0) and math.isclose(qz, 0.0):
            # 全零四元数无效，直接忽略。
            return
        thrust = float(clamp(self._thrust, 0.0, 1.0))

        # 仅按姿态四元数控制，忽略角速度通道。
        type_mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_attitude_target_send(
                self._now_ms(),
                self._target_system,
                self._target_component,
                type_mask,
                [qw, qx, qy, qz],
                0.0, 0.0, 0.0,
                thrust,
            )
        except Exception as exc:
            self._set_error(True, f"SET_ATTITUDE_TARGET failed: {exc}")

    def on_thrust(self, msg: Float32):
        # 推力作为姿态控制的附加通道，范围固定在 [0, 1]。
        self._thrust = clamp(float(msg.data), 0.0, 1.0)

    def on_rc_override(self, msg: UInt16MultiArray):
        if not self._ensure_master():
            return
        if not msg.data:
            return

        # ArduPilot/PX4 经典 RC_OVERRIDE 为 8 通道；0xFFFF 表示“忽略该通道”。
        channels = [0xFFFF] * 8
        for i in range(min(8, len(msg.data))):
            val = int(msg.data[i])
            if val in (0, 0xFFFF):
                channels[i] = 0xFFFF
                continue
            if val < 1000 or val > 2000:
                self.get_logger().warn(
                    f"RC channel {i+1} out of range ({val}), clamped to 1000-2000"
                )
                val = max(1000, min(2000, val))
            channels[i] = val

        try:
            self.master.mav.rc_channels_override_send(
                self._target_system,
                self._target_component,
                channels[0],
                channels[1],
                channels[2],
                channels[3],
                channels[4],
                channels[5],
                channels[6],
                channels[7],
            )
        except Exception as exc:
            self._set_error(True, f"RC_CHANNELS_OVERRIDE failed: {exc}")

    def on_gimbal_target(self, msg: Vector3):
        if not self._ensure_master():
            return
        # 约定输入单位为度：x=pitch, y=roll, z=yaw。
        pitch = float(msg.x)
        roll = float(msg.y)
        yaw = float(msg.z)
        try:
            self.master.mav.command_long_send(
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch,
                roll,
                yaw,
                0.0,
                0.0,
                0.0,
                float(self._gimbal_mount_mode),
            )
        except Exception as exc:
            self._set_error(True, f"DO_MOUNT_CONTROL failed: {exc}")

    def on_screenshot(self, _msg: Empty):
        # 触发一次相机快门（DIGICAM_CONTROL，param5=trigger）。
        self._send_command_long(
            self._digicam_command,
            [0.0, 0.0, 0.0, 0.0, float(self._digicam_trigger), 0.0, 0.0],
        )
        if self._screenshot_enable_save:
            self._save_latest_image()

    def on_image(self, msg: Image):
        # 缓存最新图像，用于截帧保存。
        self._latest_image = msg

    def _save_latest_image(self):
        if self._latest_image is None:
            self.get_logger().warn("Screenshot requested but no image received yet")
            return
        msg = self._latest_image
        encoding = msg.encoding.lower()

        try:
            self._screenshot_output_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # pragma: no cover - filesystem specific
            self._set_error(True, f"Create dir failed: {exc}")
            return

        idx = self._shot_counter
        self._shot_counter += 1

        fmt = self._screenshot_filename_format
        if "%d" in fmt:
            name = fmt % idx
        else:
            fmt_path = Path(fmt)
            suffix = fmt_path.suffix
            stem = fmt_path.stem or "shot"
            name = f"{stem}_{idx}{suffix}"
        out_path = self._screenshot_output_dir / name

        # Determine save method
        try:
            if self._compat.pil_supported and PILImage is not None:
                pil_img = self._image_to_pillow(msg, encoding)
                pil_img.save(out_path)
            else:
                # Fallback to raw PPM/PGM
                out_path = self._save_raw_ppm(msg, encoding, out_path)
            self.get_logger().info(
                f"Screenshot saved: {out_path} (encoding={encoding}, pil={self._compat.pil_supported})"
            )
        except ValueError as exc:
            # Unsupported encoding
            self.get_logger().warn(
                f"Screenshot skipped: {exc}. Supported: rgb8/bgr8/mono8."
            )
        except Exception as exc:  # pragma: no cover - I/O errors
            self._set_error(True, f"Save screenshot failed: {exc}")

    def _image_to_pillow(self, msg: Image, encoding: str):
        width = int(msg.width)
        height = int(msg.height)
        data = bytes(msg.data)
        if encoding == "rgb8":
            return PILImage.frombytes("RGB", (width, height), data)
        if encoding == "bgr8":
            return PILImage.frombytes("RGB", (width, height), data, "raw", "BGR")
        if encoding == "mono8":
            return PILImage.frombytes("L", (width, height), data)
        raise ValueError(f"Unsupported image encoding: {encoding}")

    def _save_raw_ppm(self, msg: Image, encoding: str, out_path: Path) -> Path:
        width = int(msg.width)
        height = int(msg.height)
        data = bytes(msg.data)

        if encoding == "rgb8":
            header = f"P6\n{width} {height}\n255\n".encode()
            out_bytes = header + data
        elif encoding == "bgr8":
            # Convert BGR to RGB without external deps.
            rgb = bytearray(len(data))
            for i in range(0, len(data), 3):
                rgb[i] = data[i + 2]
                rgb[i + 1] = data[i + 1]
                rgb[i + 2] = data[i]
            header = f"P6\n{width} {height}\n255\n".encode()
            out_bytes = header + bytes(rgb)
        elif encoding == "mono8":
            header = f"P5\n{width} {height}\n255\n".encode()
            out_bytes = header + data
        else:
            raise ValueError(f"Unsupported image encoding: {encoding}")

        # Ensure extension matches raw format if original was jpg-like
        if out_path.suffix.lower() not in {".ppm", ".pgm"}:
            out_path = out_path.with_suffix(".ppm" if encoding != "mono8" else ".pgm")
        with open(out_path, "wb") as f:
            f.write(out_bytes)
        return out_path

    def _send_move_relative_body(self, x_body: float, y_body: float, z_body: float):
        if not self._ensure_master():
            return

        # 直接使用 BODY_OFFSET_NED：
        # 在“发送命令那一刻”的机体朝向下解释位移（机头为前）。
        # 输入轴约定: x右, y前, z上。
        # BODY_NED 轴: x前, y右, z下。
        target_body_x = y_body
        target_body_y = x_body
        target_body_z = -z_body

        # 仅发送位置目标，并显式给 yaw_rate=0 来保持当前机头朝向。
        # 否则部分飞控在位置移动时会默认“朝运动方向转头”。
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        frame = getattr(
            mavutil.mavlink,
            "MAV_FRAME_BODY_OFFSET_NED",
            mavutil.mavlink.MAV_FRAME_BODY_NED,
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                self._now_ms(),
                self._target_system,
                self._target_component,
                frame,
                type_mask,
                float(target_body_x), float(target_body_y), float(target_body_z),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_LOCAL_NED (pos) failed: {exc}")

    def _send_relative_yaw_deg(self, yaw_deg: float):
        # 相对偏航：正值顺时针，负值逆时针（ArduPilot 约定）。
        if not self._ensure_master():
            return
        if abs(yaw_deg) < 1e-3:
            return
        direction = 1.0 if yaw_deg >= 0.0 else -1.0
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            [abs(yaw_deg), 0.0, direction, 1.0, 0.0, 0.0, 0.0],
        )

    def on_goto_enu(self, msg: Vector3):
        """ENU 绝对本地位置指令 -> MAVLink LOCAL_NED 位置目标。
        输入: x=East, y=North, z=Up (meters)
        """
        if not self._ensure_master():
            return
        # ENU -> NED: ned_x=north=enu_y, ned_y=east=enu_x, ned_z=-up=-enu_z
        ned_x = float(msg.y)
        ned_y = float(msg.x)
        ned_z = -float(msg.z)
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        try:
            self.master.mav.set_position_target_local_ned_send(
                self._now_ms(),
                self._target_system,
                self._target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                ned_x, ned_y, ned_z,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0,
            )
        except Exception as exc:
            self._set_error(True, f"SET_POSITION_TARGET_LOCAL_NED (goto_enu) failed: {exc}")

    def on_move_relative(self, msg: Vector3):
        # 兼容旧接口：仅相对位移，不带偏航。
        self._send_move_relative_body(float(msg.x), float(msg.y), float(msg.z))

    def on_move_relative_yaw(self, msg: Twist):
        # 新接口：linear 为机体系位移（m），angular.z 为相对偏航（rad）。
        self._send_move_relative_body(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.linear.z),
        )
        self._send_relative_yaw_deg(math.degrees(float(msg.angular.z)))


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkTxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
