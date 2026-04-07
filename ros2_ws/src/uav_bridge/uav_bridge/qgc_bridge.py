#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import socket
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Point
import rclpy
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Empty, Float32, String

from uav_bridge.mavlink_bridge import decode_copter_mode


FORMATION_INDEX = {
    0: "line_x",
    1: "line_y",
    2: "v_shape",
    3: "rectangle",
    4: "echelon",
    5: "t_shape",
    6: "origin_grid",
    7: "origin_land",
}

MODE_INDEX = {
    0: "anchored",
    1: "leader_follower",
}

SUPPORTED_GLOBAL_FRAMES = {
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
}

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = 2.0 * _WGS84_F - _WGS84_F * _WGS84_F


def _llh_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    radius = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * slat * slat)
    return (
        (radius + alt_m) * clat * clon,
        (radius + alt_m) * clat * slon,
        (radius * (1.0 - _WGS84_E2) + alt_m) * slat,
    )


def _ecef_to_enu(x: float, y: float, z: float, ref_lat_deg: float, ref_lon_deg: float, rx: float, ry: float, rz: float):
    dx, dy, dz = x - rx, y - ry, z - rz
    slat = math.sin(math.radians(ref_lat_deg))
    clat = math.cos(math.radians(ref_lat_deg))
    slon = math.sin(math.radians(ref_lon_deg))
    clon = math.cos(math.radians(ref_lon_deg))
    east = -slon * dx + clon * dy
    north = -slat * clon * dx - slat * slon * dy + clat * dz
    up = clat * clon * dx + clat * slon * dy + slat * dz
    return east, north, up


def _gps_to_enu(lat: float, lon: float, alt: float, ref):
    x, y, z = _llh_to_ecef(lat, lon, alt)
    return _ecef_to_enu(x, y, z, ref[0], ref[1], ref[2], ref[3], ref[4])


@dataclass
class MissionUploadState:
    target_system: int
    target_component: int
    expected_count: int
    items: Dict[int, object] = field(default_factory=dict)


@dataclass
class LinkState:
    qgc_socket: socket.socket
    vehicle_socket: socket.socket
    qgc_port: int
    vehicle_port: int
    qgc_peer: Optional[Tuple[str, int]] = None
    vehicle_peer: Optional[Tuple[str, int]] = None
    parser: Optional[object] = None
    mission_upload: Optional[MissionUploadState] = None


class QgcBridge(Node):

    def __init__(self):
        super().__init__("qgc_bridge")

        self.declare_parameter("num_uav", 6)
        self.declare_parameter("ns_prefix", "uav")
        self.declare_parameter("qgc_base_port", 14540)
        self.declare_parameter("vehicle_base_port", 14740)
        self.declare_parameter("reference_uav", 1)
        self.declare_parameter("poll_rate_hz", 100.0)
        self.declare_parameter("default_search_weight", 1.0)
        self.declare_parameter("default_orbit_radius", 12.0)
        self.declare_parameter("default_orbit_angular_speed", 0.12)

        self.num_uav = int(self.get_parameter("num_uav").value)
        self.ns_prefix = str(self.get_parameter("ns_prefix").value)
        self.qgc_base_port = int(self.get_parameter("qgc_base_port").value)
        self.vehicle_base_port = int(self.get_parameter("vehicle_base_port").value)
        self.reference_uav = max(1, int(self.get_parameter("reference_uav").value))
        self.poll_rate_hz = max(10.0, float(self.get_parameter("poll_rate_hz").value))
        self.default_search_weight = max(0.01, float(self.get_parameter("default_search_weight").value))
        self.default_orbit_radius = max(6.0, float(self.get_parameter("default_orbit_radius").value))
        self.default_orbit_angular_speed = float(self.get_parameter("default_orbit_angular_speed").value)
        self.reference_index = min(self.num_uav - 1, self.reference_uav - 1)

        self.formation_pub = self.create_publisher(String, "/swarm/formation", 10)
        self.mission_pub = self.create_publisher(Point, "/swarm/mission_center", 10)
        self.search_pub = self.create_publisher(String, "/swarm/search_mission", 10)
        self.orbit_pub = self.create_publisher(String, "/swarm/search_orbit", 10)
        self.clear_task_pub = self.create_publisher(Empty, "/swarm/clear_task", 10)
        self.swarm_mode_pub = self.create_publisher(String, "/swarm/mode", 10)
        self.arm_pubs = []
        self.mode_pubs = []
        self.takeoff_pubs = []
        self.land_pubs = []
        self.odoms: List[Optional[Odometry]] = [None] * self.num_uav
        self.gps_fixes: List[Optional[NavSatFix]] = [None] * self.num_uav
        self._ref = None
        self._last_uploaded_mission: List[object] = []

        for i in range(self.num_uav):
            ns = f"/{self.ns_prefix}{i + 1}"
            self.arm_pubs.append(self.create_publisher(Bool, f"{ns}/uav/cmd/arm", 10))
            self.mode_pubs.append(self.create_publisher(String, f"{ns}/uav/cmd/mode", 10))
            self.takeoff_pubs.append(self.create_publisher(Float32, f"{ns}/uav/cmd/takeoff", 10))
            self.land_pubs.append(self.create_publisher(Empty, f"{ns}/uav/cmd/land", 10))
            self.create_subscription(Odometry, f"{ns}/uav/odom", lambda msg, idx=i: self._odom_cb(idx, msg), 10)
            self.create_subscription(NavSatFix, f"{ns}/uav/navsatfix", lambda msg, idx=i: self._gps_cb(idx, msg), 10)

        self.links: List[LinkState] = []
        for i in range(self.num_uav):
            qgc_port = self.qgc_base_port + i
            vehicle_port = self.vehicle_base_port + i
            qgc_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            qgc_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            qgc_socket.bind(("0.0.0.0", qgc_port))
            qgc_socket.setblocking(False)

            vehicle_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            vehicle_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            vehicle_socket.bind(("127.0.0.1", vehicle_port))
            vehicle_socket.setblocking(False)

            parser = mavutil.mavlink.MAVLink(None)
            self.links.append(
                LinkState(
                    qgc_socket=qgc_socket,
                    vehicle_socket=vehicle_socket,
                    qgc_port=qgc_port,
                    vehicle_port=vehicle_port,
                    parser=parser,
                )
            )

        self.timer = self.create_timer(1.0 / self.poll_rate_hz, self._poll_once)
        self.get_logger().info(
            f"QGC UDP relay 启动: 外部端口 {self.qgc_base_port}-{self.qgc_base_port + self.num_uav - 1}, "
            f"内部端口 {self.vehicle_base_port}-{self.vehicle_base_port + self.num_uav - 1}, 参考机=uav{self.reference_index + 1}"
        )

    def destroy_node(self):
        for link in self.links:
            link.qgc_socket.close()
            link.vehicle_socket.close()
        return super().destroy_node()

    def _odom_cb(self, idx: int, msg: Odometry):
        self.odoms[idx] = msg

    def _gps_cb(self, idx: int, msg: NavSatFix):
        if not self._is_valid_fix(msg):
            return
        self.gps_fixes[idx] = msg
        if self._ref is None:
            rx, ry, rz = _llh_to_ecef(msg.latitude, msg.longitude, msg.altitude)
            self._ref = (msg.latitude, msg.longitude, rx, ry, rz)
            self.get_logger().info(
                f"QGC bridge 已建立 GPS 参考点: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}"
            )

    def _poll_once(self):
        for index, link in enumerate(self.links):
            self._pump_vehicle_socket(index, link)
            self._pump_qgc_socket(index, link)

    def _pump_vehicle_socket(self, index: int, link: LinkState):
        while True:
            try:
                payload, peer = link.vehicle_socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError:
                return

            link.vehicle_peer = peer
            if link.qgc_peer is not None:
                try:
                    link.qgc_socket.sendto(payload, link.qgc_peer)
                except OSError:
                    pass

    def _pump_qgc_socket(self, index: int, link: LinkState):
        while True:
            try:
                payload, peer = link.qgc_socket.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError:
                return

            if link.qgc_peer != peer:
                link.qgc_peer = peer
                self.get_logger().info(
                    f"uav{index + 1} QGC 客户端已连接: {peer[0]}:{peer[1]} -> UDP {link.qgc_port}"
                )

            swallow = self._consume_qgc_packet(index, link, payload)
            if swallow or link.vehicle_peer is None:
                continue

            try:
                link.vehicle_socket.sendto(payload, link.vehicle_peer)
            except OSError:
                pass

    def _consume_qgc_packet(self, index: int, link: LinkState, payload: bytes) -> bool:
        if link.parser is None:
            return False

        saw_message = False
        swallow_all = True

        for byte in payload:
            try:
                message = link.parser.parse_char(bytes([byte]))
            except Exception:
                return False
            if message is None:
                continue
            saw_message = True
            swallow_all = swallow_all and self._handle_qgc_message(index, message)

        return saw_message and swallow_all

    def _handle_qgc_message(self, index: int, message) -> bool:
        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return False

        if message_type == "MISSION_COUNT":
            return self._handle_mission_count(index, self.links[index], message)

        if message_type == "MISSION_ITEM_INT":
            return self._handle_mission_item_int(index, self.links[index], message)

        if message_type == "MISSION_CLEAR_ALL":
            return self._handle_mission_clear_all(index, self.links[index], message)

        if message_type == "COMMAND_LONG":
            return self._handle_command_long(index, message)

        if message_type == "SET_MODE" and self._is_broadcast(getattr(message, "target_system", 0)):
            mode_name = decode_copter_mode(int(message.custom_mode))
            if mode_name and mode_name != "UNKNOWN":
                self._publish_mode_all(mode_name)
                self.get_logger().info(f"QGC 广播模式切换 -> {mode_name}")
                return True
        return False

    def _handle_command_long(self, index: int, message) -> bool:
        command = int(message.command)
        target_system = int(getattr(message, "target_system", 0))

        if command == mavutil.mavlink.MAV_CMD_USER_1:
            formation_name = FORMATION_INDEX.get(int(round(message.param1)))
            if formation_name is not None:
                out = String()
                out.data = formation_name
                self.formation_pub.publish(out)
                self.get_logger().info(f"QGC 编队命令 -> {formation_name}")
                return True
            return False

        if command == mavutil.mavlink.MAV_CMD_USER_2:
            mode_name = MODE_INDEX.get(int(round(message.param1)))
            if mode_name is not None:
                out = String()
                out.data = mode_name
                self.swarm_mode_pub.publish(out)
                self.get_logger().info(f"QGC 编队模式命令 -> {mode_name}")
                return True
            return False

        if command == mavutil.mavlink.MAV_CMD_USER_3:
            self._publish_arm_all(message.param1 >= 0.5)
            self.get_logger().info(f"QGC 批量 arm 命令 -> {'ARM' if message.param1 >= 0.5 else 'DISARM'}")
            return True

        if command == mavutil.mavlink.MAV_CMD_USER_4:
            if message.param1 >= 0.5:
                altitude = message.param2 if message.param2 > 0.0 else 5.0
                self._publish_takeoff_all(altitude)
                self.get_logger().info(f"QGC 批量起飞命令 -> {altitude:.1f}m")
            else:
                self._publish_land_all()
                self.get_logger().info("QGC 批量降落命令")
            return True

        if self._is_broadcast(target_system):
            if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                self._publish_arm_all(message.param1 >= 0.5)
                return True
            if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                altitude = message.param7 if message.param7 > 0.0 else 5.0
                self._publish_takeoff_all(altitude)
                return True
            if command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                self._publish_land_all()
                return True

        if index == self.reference_index:
            if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.get_logger().info("QGC 正在控制参考机起飞，followers 将通过 leader_follower 保持队形")
            elif command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                self.get_logger().info("QGC 正在控制参考机降落，followers 将通过 leader_follower 保持队形")

        return False

    def _handle_mission_count(self, index: int, link: LinkState, message) -> bool:
        if index != self.reference_index:
            return False

        count = int(getattr(message, "count", 0))
        target_system = int(getattr(message, "target_system", 0))
        target_component = int(getattr(message, "target_component", 0))
        link.mission_upload = MissionUploadState(
            target_system=target_system,
            target_component=target_component,
            expected_count=max(0, count),
        )

        self.get_logger().info(f"QGC mission upload 开始: {count} 个任务项")
        if count <= 0:
            self._last_uploaded_mission = []
            self._send_mission_ack(link, target_system, target_component, mavutil.mavlink.MAV_MISSION_ACCEPTED)
            return True

        self._send_mission_request_int(link, target_system, target_component, 0)
        return True

    def _handle_mission_item_int(self, index: int, link: LinkState, message) -> bool:
        if index != self.reference_index or link.mission_upload is None:
            return False

        seq = int(getattr(message, "seq", -1))
        state = link.mission_upload
        if seq < 0 or seq >= state.expected_count:
            self._send_mission_ack(link, state.target_system, state.target_component, mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE)
            link.mission_upload = None
            return True

        state.items[seq] = message
        next_seq = len(state.items)
        if next_seq < state.expected_count:
            self._send_mission_request_int(link, state.target_system, state.target_component, next_seq)
            return True

        items = [state.items[idx] for idx in sorted(state.items)]
        self._last_uploaded_mission = items
        try:
            self._dispatch_uploaded_mission(items)
            ack_type = mavutil.mavlink.MAV_MISSION_ACCEPTED
        except ValueError as exc:
            self.get_logger().warn(f"QGC mission 解析失败: {exc}")
            ack_type = mavutil.mavlink.MAV_MISSION_ERROR

        self._send_mission_ack(link, state.target_system, state.target_component, ack_type)
        link.mission_upload = None
        return True

    def _handle_mission_clear_all(self, index: int, link: LinkState, message) -> bool:
        if index != self.reference_index:
            return False

        target_system = int(getattr(message, "target_system", 0))
        target_component = int(getattr(message, "target_component", 0))
        self._last_uploaded_mission = []
        link.mission_upload = None
        self._publish_clear_task()
        self.get_logger().info("QGC 请求清空任务 -> clear_task")
        self._send_mission_ack(link, target_system, target_component, mavutil.mavlink.MAV_MISSION_ACCEPTED)
        return True

    def _dispatch_uploaded_mission(self, items: List[object]):
        rtl_item = next(
            (item for item in items if int(getattr(item, "command", 0)) == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH),
            None,
        )
        if rtl_item is not None and len(items) == 1:
            self._publish_formation("origin_land")
            self.get_logger().info("QGC mission -> 原点返航降落")
            return

        loiter_index = next(
            (
                idx for idx, item in enumerate(items)
                if int(getattr(item, "command", 0)) in (
                    mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                    mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                )
            ),
            None,
        )
        if loiter_index is not None:
            loiter_item = items[loiter_index]
            center_xy = self._item_to_xy(loiter_item)
            if center_xy is None and loiter_index > 0:
                center_xy = self._item_to_xy(items[loiter_index - 1])
            if center_xy is None:
                raise ValueError("盘旋任务缺少有效坐标，且前一任务点不可用")
            radius = abs(float(getattr(loiter_item, "param3", 0.0)))
            if radius < 1e-3:
                radius = self.default_orbit_radius
            self._publish_search_orbit(center_xy[0], center_xy[1], radius, self.default_orbit_angular_speed)
            self.get_logger().info(
                f"QGC mission -> 盘旋搜索 center=({center_xy[0]:.1f}, {center_xy[1]:.1f}) radius={radius:.1f}"
            )
            return

        waypoints = []
        for idx, item in enumerate(items):
            if int(getattr(item, "command", 0)) != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                continue
            xy = self._item_to_xy(item)
            if xy is None:
                continue
            weight = self._item_weight(item)
            waypoints.append({
                "id": f"wp_{idx + 1}",
                "x": xy[0],
                "y": xy[1],
                "weight": weight,
            })

        if len(waypoints) == 1:
            waypoint = waypoints[0]
            self._publish_mission_center(waypoint["x"], waypoint["y"])
            self.get_logger().info(
                f"QGC mission -> 编队前往目标中心 ({waypoint['x']:.1f}, {waypoint['y']:.1f})"
            )
            return

        if len(waypoints) > 1:
            self._publish_search_mission(waypoints)
            self.get_logger().info(f"QGC mission -> 多目标搜索，共 {len(waypoints)} 个点")
            return

        if rtl_item is not None:
            self._publish_formation("origin_land")
            self.get_logger().info("QGC mission -> 原点返航降落")
            return

        raise ValueError("未识别到可映射的任务项，支持 waypoint / loiter / RTL")

    def _item_weight(self, item) -> float:
        value = float(getattr(item, "param1", 0.0))
        if not math.isfinite(value) or value <= 0.0:
            return self.default_search_weight
        return max(0.01, value)

    def _item_to_xy(self, item) -> Optional[Tuple[float, float]]:
        if self._ref is None:
            raise ValueError("尚未收到 GPS 参考点，暂时无法解析 QGC 地图任务")

        frame = int(getattr(item, "frame", -1))
        if frame not in SUPPORTED_GLOBAL_FRAMES:
            return None

        lat = float(getattr(item, "x", 0.0)) / 1e7
        lon = float(getattr(item, "y", 0.0)) / 1e7
        alt = float(getattr(item, "z", 0.0))
        east, north, _ = _gps_to_enu(lat, lon, alt, self._ref)
        return float(east), float(north)

    def _publish_formation(self, name: str):
        message = String()
        message.data = name
        self.formation_pub.publish(message)

    def _publish_mission_center(self, x: float, y: float):
        message = Point()
        message.x = float(x)
        message.y = float(y)
        message.z = 0.0
        self.mission_pub.publish(message)

    def _publish_search_mission(self, targets: List[Dict[str, float]]):
        message = String()
        message.data = json.dumps({"request_id": "qgc_mission", "targets": targets}, ensure_ascii=True)
        self.search_pub.publish(message)

    def _publish_search_orbit(self, x: float, y: float, radius: float, angular_speed: float):
        message = String()
        message.data = json.dumps(
            {
                "x": float(x),
                "y": float(y),
                "radius": float(radius),
                "angular_speed": float(angular_speed),
            },
            ensure_ascii=True,
        )
        self.orbit_pub.publish(message)

    def _publish_clear_task(self):
        self.clear_task_pub.publish(Empty())

    def _send_mission_request_int(self, link: LinkState, target_system: int, target_component: int, seq: int):
        message = mavutil.mavlink.MAVLink_mission_request_int_message(target_system, target_component, seq)
        self._send_qgc_message(link, message)

    def _send_mission_ack(self, link: LinkState, target_system: int, target_component: int, ack_type: int):
        message = mavutil.mavlink.MAVLink_mission_ack_message(target_system, target_component, ack_type)
        self._send_qgc_message(link, message)

    def _send_qgc_message(self, link: LinkState, message):
        if link.qgc_peer is None:
            return

        encoder = mavutil.mavlink.MAVLink(None)
        encoder.srcSystem = self.reference_index + 1
        encoder.srcComponent = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        try:
            payload = message.pack(encoder)
            link.qgc_socket.sendto(payload, link.qgc_peer)
        except OSError:
            pass

    def _is_valid_fix(self, msg: NavSatFix) -> bool:
        return (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and math.isfinite(msg.altitude)
            and abs(msg.latitude) > 1e-7
            and abs(msg.longitude) > 1e-7
        )

    def _is_broadcast(self, target_system: int) -> bool:
        return target_system in (0, 255)

    def _publish_arm_all(self, armed: bool):
        for publisher in self.arm_pubs:
            msg = Bool()
            msg.data = armed
            publisher.publish(msg)

    def _publish_mode_all(self, mode_name: str):
        for publisher in self.mode_pubs:
            msg = String()
            msg.data = mode_name
            publisher.publish(msg)

    def _publish_takeoff_all(self, altitude: float):
        for publisher in self.takeoff_pubs:
            msg = Float32()
            msg.data = float(altitude)
            publisher.publish(msg)

    def _publish_land_all(self):
        for publisher in self.land_pubs:
            publisher.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = QgcBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()