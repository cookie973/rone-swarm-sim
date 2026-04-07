import json
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String


FORMATIONS = [
    'line_x',
    'line_y',
    'echelon',
    'rectangle',
    'v_shape',
    't_shape',
    'origin',
    'origin_land',
]

SEARCH_ROWS = 5


class SwarmGroundStation:

    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('swarm_ground_station')

        self.status = {
            'phase': 'unknown',
            'formation': 'unknown',
            'mission_active': False,
            'mission_goal': None,
            'mission_center': None,
            'search_active': False,
            'search_target_counts': {},
            'search_assignments': {},
            'orbit_active': False,
            'orbit_pending': None,
            'orbit_center': None,
            'queued_request': None,
        }
        self.drone_state = {
            i: {'x': 0.0, 'y': 0.0, 'z': 0.0, 'armed': False, 'mode': 'unknown'}
            for i in range(1, 7)
        }

        self.formation_pub = self.node.create_publisher(String, '/swarm/formation', 10)
        self.mission_pub = self.node.create_publisher(Point, '/swarm/mission_center', 10)
        self.search_pub = self.node.create_publisher(String, '/swarm/search_mission', 10)
        self.orbit_pub = self.node.create_publisher(String, '/swarm/search_orbit', 10)
        self.clear_task_pub = self.node.create_publisher(Empty, '/swarm/clear_task', 10)
        self.arm_pubs = {}
        self.mode_pubs = {}
        self.takeoff_pubs = {}
        self.land_pubs = {}
        self.node.create_subscription(String, '/swarm/status', self._on_status, 10)

        for i in range(1, 7):
            self.arm_pubs[i] = self.node.create_publisher(Bool, f'/uav{i}/uav/cmd/arm', 10)
            self.mode_pubs[i] = self.node.create_publisher(String, f'/uav{i}/uav/cmd/mode', 10)
            self.takeoff_pubs[i] = self.node.create_publisher(Float32, f'/uav{i}/uav/cmd/takeoff', 10)
            self.land_pubs[i] = self.node.create_publisher(Empty, f'/uav{i}/uav/cmd/land', 10)
            self.node.create_subscription(
                Odometry,
                f'/uav{i}/uav/odom',
                lambda msg, did=i: self._on_odom(did, msg),
                10,
            )
            self.node.create_subscription(
                Bool,
                f'/uav{i}/uav/armed',
                lambda msg, did=i: self._on_armed(did, msg),
                10,
            )
            self.node.create_subscription(
                String,
                f'/uav{i}/uav/mode',
                lambda msg, did=i: self._on_mode(did, msg),
                10,
            )

        self.root = tk.Tk()
        self.root.title('Swarm Ground Station')
        self.root.geometry('1480x900')
        self._build_ui()
        self._tick()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 12))

        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        status_box = ttk.LabelFrame(left, text='Swarm Status', padding=10)
        status_box.pack(fill=tk.X, pady=(0, 12))
        self.phase_var = tk.StringVar(value='phase: unknown')
        self.form_var = tk.StringVar(value='formation: unknown')
        self.goal_var = tk.StringVar(value='goal: none')
        self.search_var = tk.StringVar(value='search: inactive')
        self.orbit_var = tk.StringVar(value='orbit: inactive')
        self.queue_var = tk.StringVar(value='queue: none')
        ttk.Label(status_box, textvariable=self.phase_var).pack(anchor=tk.W)
        ttk.Label(status_box, textvariable=self.form_var).pack(anchor=tk.W)
        ttk.Label(status_box, textvariable=self.goal_var).pack(anchor=tk.W)
        ttk.Label(status_box, textvariable=self.search_var).pack(anchor=tk.W)
        ttk.Label(status_box, textvariable=self.orbit_var).pack(anchor=tk.W)
        ttk.Label(status_box, textvariable=self.queue_var).pack(anchor=tk.W)

        action_box = ttk.LabelFrame(left, text='Quick Actions', padding=10)
        action_box.pack(fill=tk.X, pady=(0, 12))
        self.takeoff_alt = tk.StringVar(value='5.0')
        ttk.Label(action_box, text='Takeoff Alt').grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(action_box, textvariable=self.takeoff_alt, width=10).grid(row=0, column=1, padx=6, pady=4)
        ttk.Button(action_box, text='一键起飞', command=self._takeoff_all).grid(row=1, column=0, columnspan=2, sticky=tk.EW, pady=4)
        ttk.Button(action_box, text='当前点位起飞', command=self._takeoff_all).grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=4)
        ttk.Button(action_box, text='当前点位降落', command=self._land_here).grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=4)
        ttk.Button(action_box, text='返回初始位置降落', command=lambda: self._send_formation('origin_land')).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=4)
        ttk.Button(action_box, text='清空当前任务', command=self._clear_current_task).grid(row=5, column=0, columnspan=2, sticky=tk.EW, pady=4)
        action_box.columnconfigure(0, weight=1)
        action_box.columnconfigure(1, weight=1)

        mission_box = ttk.LabelFrame(left, text='Mission Center', padding=10)
        mission_box.pack(fill=tk.X, pady=(0, 12))
        ttk.Label(mission_box, text='Target X (East)').grid(row=0, column=0, sticky=tk.W)
        ttk.Label(mission_box, text='Target Y (North)').grid(row=1, column=0, sticky=tk.W)
        self.target_x = tk.StringVar(value='0.0')
        self.target_y = tk.StringVar(value='0.0')
        ttk.Entry(mission_box, textvariable=self.target_x, width=12).grid(row=0, column=1, padx=6, pady=4)
        ttk.Entry(mission_box, textvariable=self.target_y, width=12).grid(row=1, column=1, padx=6, pady=4)
        ttk.Button(mission_box, text='编队飞行前往目标点', command=self._send_target).grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))

        formation_box = ttk.LabelFrame(left, text='编队切换（静止/飞行中）', padding=10)
        formation_box.pack(fill=tk.X)
        for idx, name in enumerate(FORMATIONS):
            ttk.Button(
                formation_box,
                text=name,
                command=lambda form=name: self._send_formation(form),
            ).grid(row=idx // 2, column=idx % 2, sticky=tk.EW, padx=4, pady=4)
        formation_box.columnconfigure(0, weight=1)
        formation_box.columnconfigure(1, weight=1)

        search_box = ttk.LabelFrame(right, text='搜寻模式1：多目标点加权搜索', padding=10)
        search_box.pack(fill=tk.X, pady=(0, 12))
        headers = ['ID', 'X', 'Y', 'Weight']
        for col, text in enumerate(headers):
            ttk.Label(search_box, text=text).grid(row=0, column=col, padx=4, pady=2, sticky=tk.W)
        self.search_rows = []
        default_targets = [
            ('target_1', '120', '20', '5.0'),
            ('target_2', '150', '-15', '2.5'),
            ('target_3', '90', '5', '1.0'),
            ('', '', '', ''),
            ('', '', '', ''),
        ]
        for idx in range(SEARCH_ROWS):
            row_vars = [tk.StringVar(value=default_targets[idx][col]) for col in range(4)]
            self.search_rows.append(row_vars)
            for col, var in enumerate(row_vars):
                ttk.Entry(search_box, textvariable=var, width=12).grid(row=idx + 1, column=col, padx=4, pady=2)
        ttk.Button(search_box, text='启动搜寻模式1', command=self._send_search_mode1).grid(row=SEARCH_ROWS + 1, column=0, columnspan=4, sticky=tk.EW, pady=(8, 0))

        orbit_box = ttk.LabelFrame(right, text='搜寻模式2：单点盘旋搜索', padding=10)
        orbit_box.pack(fill=tk.X, pady=(0, 12))
        self.orbit_x = tk.StringVar(value='140.0')
        self.orbit_y = tk.StringVar(value='30.0')
        self.orbit_radius = tk.StringVar(value='12.0')
        self.orbit_speed = tk.StringVar(value='0.12')
        ttk.Label(orbit_box, text='Center X').grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(orbit_box, textvariable=self.orbit_x, width=12).grid(row=0, column=1, padx=6, pady=4)
        ttk.Label(orbit_box, text='Center Y').grid(row=0, column=2, sticky=tk.W)
        ttk.Entry(orbit_box, textvariable=self.orbit_y, width=12).grid(row=0, column=3, padx=6, pady=4)
        ttk.Label(orbit_box, text='Radius').grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(orbit_box, textvariable=self.orbit_radius, width=12).grid(row=1, column=1, padx=6, pady=4)
        ttk.Label(orbit_box, text='Angular Speed').grid(row=1, column=2, sticky=tk.W)
        ttk.Entry(orbit_box, textvariable=self.orbit_speed, width=12).grid(row=1, column=3, padx=6, pady=4)
        ttk.Button(orbit_box, text='启动搜寻模式2', command=self._send_search_mode2).grid(row=2, column=0, columnspan=4, sticky=tk.EW, pady=(8, 0))

        assignment_box = ttk.LabelFrame(right, text='Task Detail', padding=10)
        assignment_box.pack(fill=tk.X, pady=(0, 12))
        self.assignment_var = tk.StringVar(value='No search task')
        ttk.Label(assignment_box, textvariable=self.assignment_var, justify=tk.LEFT).pack(anchor=tk.W)

        state_box = ttk.LabelFrame(right, text='UAV State', padding=10)
        state_box.pack(fill=tk.BOTH, expand=True)

        columns = ('uav', 'mode', 'armed', 'x', 'y', 'z')
        self.tree = ttk.Treeview(state_box, columns=columns, show='headings', height=12)
        for col in columns:
            self.tree.heading(col, text=col.upper())
            self.tree.column(col, width=110, anchor=tk.CENTER)
        self.tree.pack(fill=tk.BOTH, expand=True)

        for i in range(1, 7):
            self.tree.insert('', tk.END, iid=str(i), values=(f'uav{i}', 'unknown', 'False', '0.0', '0.0', '0.0'))

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.status = {'phase': 'decode_error', 'formation': 'unknown'}

    def _on_odom(self, did, msg):
        p = msg.pose.pose.position
        self.drone_state[did]['x'] = p.x
        self.drone_state[did]['y'] = p.y
        self.drone_state[did]['z'] = p.z

    def _on_armed(self, did, msg):
        self.drone_state[did]['armed'] = bool(msg.data)

    def _on_mode(self, did, msg):
        self.drone_state[did]['mode'] = msg.data

    def _send_formation(self, name):
        msg = String()
        msg.data = name
        self.formation_pub.publish(msg)

    def _publish_all_mode(self, mode_name):
        for did in range(1, 7):
            msg = String()
            msg.data = mode_name
            self.mode_pubs[did].publish(msg)

    def _publish_all_arm(self, armed):
        for did in range(1, 7):
            msg = Bool()
            msg.data = armed
            self.arm_pubs[did].publish(msg)

    def _publish_all_takeoff(self, altitude):
        for did in range(1, 7):
            msg = Float32()
            msg.data = altitude
            self.takeoff_pubs[did].publish(msg)

    def _spin_briefly(self, cycles=1, timeout_sec=0.05):
        for _ in range(cycles):
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _clear_current_task(self, rounds=3):
        for _ in range(rounds):
            self.clear_task_pub.publish(Empty())
            self._spin_briefly()

    def _takeoff_all(self):
        try:
            altitude = float(self.takeoff_alt.get())
        except ValueError:
            messagebox.showerror('输入错误', '起飞高度必须是数字')
            return

        self._clear_current_task(rounds=4)

        # First stabilize mode switching and arming serially to avoid partial takeoff.
        for did in range(1, 7):
            for _ in range(4):
                mode_msg = String()
                mode_msg.data = 'GUIDED'
                arm_msg = Bool()
                arm_msg.data = True
                self.mode_pubs[did].publish(mode_msg)
                self.arm_pubs[did].publish(arm_msg)
                self._spin_briefly(cycles=2, timeout_sec=0.08)

        for _ in range(8):
            for did in range(1, 7):
                mode_msg = String()
                mode_msg.data = 'GUIDED'
                self.mode_pubs[did].publish(mode_msg)
                if not self.drone_state[did]['armed']:
                    arm_msg = Bool()
                    arm_msg.data = True
                    self.arm_pubs[did].publish(arm_msg)
                takeoff_msg = Float32()
                takeoff_msg.data = altitude
                self.takeoff_pubs[did].publish(takeoff_msg)
                self._spin_briefly(cycles=2, timeout_sec=0.08)

    def _land_here(self):
        self._clear_current_task(rounds=4)
        for _ in range(6):
            for did in range(1, 7):
                self.land_pubs[did].publish(Empty())
            self._spin_briefly(cycles=2, timeout_sec=0.08)

    def _send_target(self):
        try:
            x = float(self.target_x.get())
            y = float(self.target_y.get())
        except ValueError:
            messagebox.showerror('输入错误', '目标点坐标必须是数字')
            return
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.mission_pub.publish(msg)

    def _send_search_mode1(self):
        targets = []
        for row in self.search_rows:
            target_id = row[0].get().strip()
            x_text = row[1].get().strip()
            y_text = row[2].get().strip()
            weight_text = row[3].get().strip()
            if not any([target_id, x_text, y_text, weight_text]):
                continue
            try:
                x = float(x_text)
                y = float(y_text)
                weight = float(weight_text)
            except ValueError:
                messagebox.showerror('输入错误', '搜寻模式1 的坐标和权重必须是数字')
                return
            targets.append({
                'id': target_id or f'target_{len(targets) + 1}',
                'x': x,
                'y': y,
                'weight': weight,
            })
        if not targets:
            messagebox.showerror('输入错误', '请至少填写一个搜索目标点')
            return
        if len(targets) >= 6:
            messagebox.showerror('输入错误', '目标点数量必须小于 6')
            return
        msg = String()
        msg.data = json.dumps({'request_id': f'gs_search_{int(self.node.get_clock().now().nanoseconds / 1e9)}', 'targets': targets}, ensure_ascii=True)
        for _ in range(3):
            self.search_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _send_search_mode2(self):
        try:
            x = float(self.orbit_x.get())
            y = float(self.orbit_y.get())
            radius = float(self.orbit_radius.get())
            angular_speed = float(self.orbit_speed.get())
        except ValueError:
            messagebox.showerror('输入错误', '搜寻模式2 的参数必须是数字')
            return
        msg = String()
        msg.data = json.dumps({
            'x': x,
            'y': y,
            'radius': radius,
            'angular_speed': angular_speed,
        }, ensure_ascii=True)
        for _ in range(3):
            self.orbit_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _refresh_ui(self):
        self.phase_var.set(f"phase: {self.status.get('phase', 'unknown')}")
        self.form_var.set(f"formation: {self.status.get('formation', 'unknown')}")
        self.goal_var.set(f"goal: {self.status.get('mission_goal', None)}")
        self.search_var.set(f"search: {self.status.get('search_target_counts', {}) if self.status.get('search_active') else 'inactive'}")
        orbit_text = 'inactive'
        if self.status.get('orbit_active'):
            orbit_text = f"active center={self.status.get('orbit_center')}"
        elif self.status.get('orbit_pending'):
            orbit_text = f"pending {self.status.get('orbit_pending')}"
        self.orbit_var.set(f"orbit: {orbit_text}")
        self.queue_var.set(f"queue: {self.status.get('queued_request', None)}")
        assignments = self.status.get('search_assignments', {})
        counts = self.status.get('search_target_counts', {})
        if assignments:
            lines = [f'{target}: {counts.get(target, 0)} 架' for target in sorted(counts)]
            lines.extend([f'uav{did} -> {target}' for did, target in sorted(assignments.items(), key=lambda item: int(item[0]))])
            self.assignment_var.set('\n'.join(lines))
        else:
            self.assignment_var.set('No search task')
        for did in range(1, 7):
            s = self.drone_state[did]
            self.tree.item(
                str(did),
                values=(
                    f'uav{did}',
                    s['mode'],
                    str(s['armed']),
                    f"{s['x']:.2f}",
                    f"{s['y']:.2f}",
                    f"{s['z']:.2f}",
                ),
            )

    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._refresh_ui()
        self.root.after(100, self._tick)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    app = SwarmGroundStation()
    app.run()
