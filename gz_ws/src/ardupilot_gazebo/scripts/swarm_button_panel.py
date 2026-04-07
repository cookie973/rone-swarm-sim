#!/usr/bin/env python3
"""
swarm_button_panel.py

Lightweight GUI button panel for swarm operations while QGC drives target points.
"""

import os
import shlex
import subprocess
import threading
import tkinter as tk
from datetime import datetime
from tkinter import messagebox

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_SETUP = (
    "set +u; "
    "source /opt/ros/humble/setup.bash; "
    "if [ -f \"$HOME/ros2_ws/install/setup.bash\" ]; then source \"$HOME/ros2_ws/install/setup.bash\"; fi; "
    "set -u 2>/dev/null || true; "
)


class SwarmButtonPanel:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Swarm Quick Panel")
        self.root.geometry("760x520")

        self.altitude_var = tk.StringVar(value="5.0")
        self._build_ui()

    def _build_ui(self):
        top = tk.Frame(self.root, padx=10, pady=10)
        top.pack(fill=tk.X)

        tk.Label(top, text="Target Altitude (m):").pack(side=tk.LEFT)
        tk.Entry(top, textvariable=self.altitude_var, width=8).pack(side=tk.LEFT, padx=6)

        tk.Button(top, text="Takeoff All", width=14, command=self.takeoff_all).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Land All", width=14, command=self.land_all).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Arm All", width=14, command=self.arm_all).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Disarm All", width=14, command=self.disarm_all).pack(side=tk.LEFT, padx=4)

        mode_frame = tk.LabelFrame(self.root, text="Swarm Mode", padx=10, pady=10)
        mode_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Button(mode_frame, text="leader_follower", width=20, command=lambda: self.set_mode("leader_follower")).pack(side=tk.LEFT, padx=6)
        tk.Button(mode_frame, text="anchored", width=20, command=lambda: self.set_mode("anchored")).pack(side=tk.LEFT, padx=6)

        form_frame = tk.LabelFrame(self.root, text="Formation", padx=10, pady=10)
        form_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Button(form_frame, text="V-Shape", width=16, command=lambda: self.set_formation("v_shape")).pack(side=tk.LEFT, padx=4)
        tk.Button(form_frame, text="Line", width=16, command=lambda: self.set_formation("line")).pack(side=tk.LEFT, padx=4)
        tk.Button(form_frame, text="Circle", width=16, command=lambda: self.set_formation("circle")).pack(side=tk.LEFT, padx=4)
        tk.Button(form_frame, text="Grid", width=16, command=lambda: self.set_formation("grid")).pack(side=tk.LEFT, padx=4)

        oneclick_frame = tk.LabelFrame(self.root, text="One-Click Presets", padx=10, pady=10)
        oneclick_frame.pack(fill=tk.X, padx=10, pady=6)

        tk.Button(oneclick_frame, text="Go V (leader_follower + v_shape)", width=34, command=self.go_v_shape).pack(side=tk.LEFT, padx=6)
        tk.Button(oneclick_frame, text="Go Line (leader_follower + line)", width=30, command=self.go_line).pack(side=tk.LEFT, padx=6)

        qgc_hint = tk.Label(
            self.root,
            text="Takeoff All uses the validated sequential CLI takeoff flow; after climb-out, send a QGC target to uav1 and switch formation here.",
            fg="#0B4F6C",
            anchor="w",
            justify=tk.LEFT,
            padx=12,
        )
        qgc_hint.pack(fill=tk.X, pady=(2, 6))

        log_frame = tk.LabelFrame(self.root, text="Command Log", padx=8, pady=8)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        self.log_text = tk.Text(log_frame, height=14, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log("Panel ready.")

    def log(self, text: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {text}\n")
        self.log_text.see(tk.END)

    def _run_shell(self, title: str, command: str):
        def worker():
            self.root.after(0, lambda: self.log(f"RUN {title}: {command}"))
            full_cmd = f"{ROS_SETUP}{command}"
            proc = subprocess.run(
                ["bash", "-lc", full_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=SCRIPT_DIR,
            )
            output = (proc.stdout or "").strip()
            if output:
                self.root.after(0, lambda: self.log(output))
            if proc.returncode == 0:
                self.root.after(0, lambda: self.log(f"DONE {title}"))
            else:
                self.root.after(0, lambda: self.log(f"FAILED {title} (code={proc.returncode})"))

        threading.Thread(target=worker, daemon=True).start()

    def set_mode(self, mode: str):
        cmd = f"ros2 topic pub --once /swarm/mode std_msgs/msg/String \"{{data: '{mode}'}}\""
        self._run_shell(f"set mode {mode}", cmd)

    def set_formation(self, formation: str):
        cmd = f"ros2 topic pub --once /swarm/formation std_msgs/msg/String \"{{data: '{formation}'}}\""
        self._run_shell(f"set formation {formation}", cmd)

    def arm_all(self):
        cmd = "for i in 1 2 3 4 5 6; do ros2 topic pub --once /uav${i}/uav/cmd/arm std_msgs/msg/Bool '{data: true}'; done"
        self._run_shell("arm all", cmd)

    def disarm_all(self):
        cmd = "for i in 1 2 3 4 5 6; do ros2 topic pub --once /uav${i}/uav/cmd/arm std_msgs/msg/Bool '{data: false}'; done"
        self._run_shell("disarm all", cmd)

    def takeoff_all(self):
        alt = self.altitude_var.get().strip()
        try:
            altitude = float(alt)
            if altitude <= 0.0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid altitude", "Altitude must be a positive number.")
            return
        cmd = (
            f"TAKEOFF_ENGINE=cli TAKEOFF_RC_OVERRIDE=1 "
            f"{shlex.quote(os.path.join(SCRIPT_DIR, 'takeoff_all.sh'))} {altitude:.2f}"
        )
        self._run_shell(f"takeoff all via cli ({altitude:.2f}m)", cmd)

    def land_all(self):
        cmd = shlex.quote(os.path.join(SCRIPT_DIR, "land_all.sh"))
        self._run_shell("land all", cmd)

    def go_v_shape(self):
        cmd = (
            "ros2 topic pub --once /swarm/mode std_msgs/msg/String \"{data: 'leader_follower'}\"; "
            "sleep 0.8; "
            "ros2 topic pub --once /swarm/formation std_msgs/msg/String \"{data: 'v_shape'}\""
        )
        self._run_shell("preset V-shape", cmd)

    def go_line(self):
        cmd = (
            "ros2 topic pub --once /swarm/mode std_msgs/msg/String \"{data: 'leader_follower'}\"; "
            "sleep 0.8; "
            "ros2 topic pub --once /swarm/formation std_msgs/msg/String \"{data: 'line'}\""
        )
        self._run_shell("preset line", cmd)


def main():
    root = tk.Tk()
    SwarmButtonPanel(root)
    root.mainloop()


if __name__ == "__main__":
    main()
