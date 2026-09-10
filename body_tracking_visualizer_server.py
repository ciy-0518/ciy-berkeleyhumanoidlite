#!/usr/bin/env python3
"""Receive and visualize UE body-tracking UDP packets on another PC."""

from __future__ import annotations

import argparse
import json
import queue
import socket
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import ttk
from typing import Any


JOINT_NAMES = (
    "Head",
    "L_UpperArm",
    "L_LowerArm",
    "L_WristTwist",
    "R_UpperArm",
    "R_LowerArm",
    "R_WristTwist",
)

TORSO_CENTER = "TorsoCenter"

SEGMENTS = (
    ("Head", TORSO_CENTER),
    (TORSO_CENTER, "L_UpperArm"),
    ("L_UpperArm", "L_LowerArm"),
    ("L_LowerArm", "L_WristTwist"),
    (TORSO_CENTER, "R_UpperArm"),
    ("R_UpperArm", "R_LowerArm"),
    ("R_LowerArm", "R_WristTwist"),
)

BACKGROUND = "#111827"
GRID = "#243044"
TEXT = "#e5e7eb"
MUTED = "#94a3b8"
LEFT = "#3b82f6"
RIGHT = "#ef4444"
HEAD = "#22c55e"


def number(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass
class Joint:
    name: str
    pos: tuple[float, float, float]
    relative_deg: dict[str, float] = field(default_factory=dict)


@dataclass
class Packet:
    active: bool
    reason: str = ""
    confidence: float = 0.0
    joints: dict[str, Joint] = field(default_factory=dict)
    sender: tuple[str, int] | None = None
    received_at: float = field(default_factory=time.time)


def parse_packet(raw: bytes, sender: tuple[str, int]) -> Packet:
    payload = json.loads(raw.decode("utf-8"))
    joints: dict[str, Joint] = {}

    for item in payload.get("joints", []):
        name = str(item.get("name", ""))
        pos = item.get("pos", [])
        if name not in JOINT_NAMES or not isinstance(pos, list) or len(pos) < 3:
            continue

        angles = item.get("relativeDeg", {})
        joints[name] = Joint(
            name=name,
            pos=(number(pos[0]), number(pos[1]), number(pos[2])),
            relative_deg={
                "roll": number(angles.get("roll")),
                "pitch": number(angles.get("pitch")),
                "yaw": number(angles.get("yaw")),
            },
        )

    return Packet(
        active=bool(payload.get("active", False)),
        reason=str(payload.get("reason", "")),
        confidence=number(payload.get("confidence")),
        joints=joints,
        sender=sender,
    )


class UdpReceiver(threading.Thread):
    def __init__(self, output: queue.Queue[Packet], host: str, port: int):
        super().__init__(daemon=True)
        self.output = output
        self.host = host
        self.port = port
        self.stop_event = threading.Event()
        self.sock: socket.socket | None = None
        self.error: str | None = None
        self.datagram_count = 0
        self.parse_error_count = 0
        self.last_sender: tuple[str, int] | None = None
        self.last_error: str | None = None

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock = sock

        try:
            sock.bind((self.host, self.port))
        except OSError as exc:
            self.error = str(exc)
            print(f"[ERROR] UDP bind failed: {self.host}:{self.port}")
            print(f"        {exc}")
            print("        Stop any other UDP receiver using the same port.")
            return

        print(f"[UDP] Listening on {self.host}:{self.port}")
        sock.settimeout(0.25)

        while not self.stop_event.is_set():
            try:
                raw, sender = sock.recvfrom(65535)
                self.datagram_count += 1
                self.last_sender = sender
                packet = parse_packet(raw, sender)
                if self.datagram_count == 1 or self.datagram_count % 100 == 0:
                    print(
                        f"[UDP] datagrams={self.datagram_count} "
                        f"sender={sender[0]}:{sender[1]} bytes={len(raw)} "
                        f"active={packet.active} joints={len(packet.joints)}"
                    )
            except socket.timeout:
                continue
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.parse_error_count += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                print(f"[ERROR] Invalid JSON packet: {self.last_error}")
                continue
            except OSError as exc:
                if not self.stop_event.is_set():
                    self.last_error = f"OSError: {exc}"
                    print(f"[ERROR] UDP receive failed: {self.last_error}")
                continue
            except Exception as exc:
                self.parse_error_count += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                print(f"[ERROR] Packet processing failed: {self.last_error}")
                continue

            try:
                self.output.put_nowait(packet)
            except queue.Full:
                try:
                    self.output.get_nowait()
                except queue.Empty:
                    pass
                self.output.put_nowait(packet)

    def stop(self) -> None:
        self.stop_event.set()
        if self.sock:
            self.sock.close()


class Visualizer:
    def __init__(self, root: tk.Tk, host: str, port: int):
        self.root = root
        self.host = host
        self.port = port
        self.packets: queue.Queue[Packet] = queue.Queue(maxsize=4)
        self.receiver = UdpReceiver(self.packets, host, port)
        self.last_packet: Packet | None = None
        self.packet_count = 0
        self.started_at = time.time()
        self.local_addresses = self.find_local_ipv4_addresses()

        root.title(f"UE Body Tracking UDP Server - {host}:{port}")
        root.geometry("1160x700")
        root.configure(bg=BACKGROUND)
        root.protocol("WM_DELETE_WINDOW", self.close)

        local_ip_text = ", ".join(self.local_addresses) or "unknown"
        self.status = tk.StringVar(value=f"Listening on UDP {host}:{port}")
        self.metrics = tk.StringVar(
            value=f"Server IPv4: {local_ip_text} | Set Unreal RemoteIp to one of these addresses"
        )
        self.build_ui()

        self.receiver.start()
        self.root.after(33, self.refresh)

    @staticmethod
    def find_local_ipv4_addresses() -> list[str]:
        addresses: set[str] = set()
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                address = info[4][0]
                if address and not address.startswith("127."):
                    addresses.add(address)
        except OSError:
            pass

        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect(("8.8.8.8", 80))
            addresses.add(probe.getsockname()[0])
            probe.close()
        except OSError:
            pass

        return sorted(addresses)

    def build_ui(self) -> None:
        header = tk.Frame(self.root, bg=BACKGROUND)
        header.pack(fill=tk.X, padx=14, pady=(12, 8))

        tk.Label(
            header,
            textvariable=self.status,
            bg=BACKGROUND,
            fg=TEXT,
            font=("Segoe UI", 13, "bold"),
        ).pack(anchor="w")
        tk.Label(
            header,
            textvariable=self.metrics,
            bg=BACKGROUND,
            fg=MUTED,
            font=("Consolas", 10),
        ).pack(anchor="w", pady=(3, 0))

        body = tk.PanedWindow(
            self.root,
            orient=tk.HORIZONTAL,
            bg=BACKGROUND,
            sashwidth=6,
            bd=0,
        )
        body.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        self.canvas = tk.Canvas(
            body,
            width=680,
            height=600,
            bg=BACKGROUND,
            highlightthickness=0,
        )
        body.add(self.canvas, stretch="always")

        table_frame = tk.Frame(body, bg=BACKGROUND)
        body.add(table_frame, minsize=430)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Treeview",
            background="#172033",
            fieldbackground="#172033",
            foreground=TEXT,
            rowheight=27,
        )
        style.configure("Treeview.Heading", background="#243044", foreground=TEXT)

        self.table = ttk.Treeview(
            table_frame,
            columns=("joint", "pos", "roll", "pitch", "yaw"),
            show="headings",
        )
        self.table.heading("joint", text="Joint")
        self.table.heading("pos", text="Position X, Y, Z")
        self.table.heading("roll", text="Roll")
        self.table.heading("pitch", text="Pitch")
        self.table.heading("yaw", text="Yaw")
        self.table.column("joint", width=100, anchor="w")
        self.table.column("pos", width=190, anchor="center")
        self.table.column("roll", width=55, anchor="e")
        self.table.column("pitch", width=55, anchor="e")
        self.table.column("yaw", width=55, anchor="e")
        self.table.pack(fill=tk.BOTH, expand=True)

        for name in JOINT_NAMES:
            self.table.insert("", tk.END, iid=name, values=(name, "--", "--", "--", "--"))

        tk.Label(
            table_frame,
            text=(
                "UDP server bind: 0.0.0.0 receives packets on all local interfaces.\n"
                "Configure Unreal RemoteIp as this server PC's IP address."
            ),
            justify="left",
            bg=BACKGROUND,
            fg=MUTED,
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(8, 0))

    def refresh(self) -> None:
        while True:
            try:
                self.last_packet = self.packets.get_nowait()
                self.packet_count += 1
            except queue.Empty:
                break

        self.draw()
        self.root.after(33, self.refresh)

    def draw(self) -> None:
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 600)
        height = max(self.canvas.winfo_height(), 520)
        self.draw_grid(width, height)

        if self.receiver.error:
            self.status.set(f"UDP bind failed: {self.receiver.error}")
            self.center_text(width, height, self.receiver.error)
            return

        packet = self.last_packet
        if not packet:
            local_ip_text = ", ".join(self.local_addresses) or "unknown"
            sender_text = (
                f"{self.receiver.last_sender[0]}:{self.receiver.last_sender[1]}"
                if self.receiver.last_sender
                else "none"
            )
            self.metrics.set(
                f"Server IPv4={local_ip_text}  UDP datagrams={self.receiver.datagram_count}  "
                f"parse errors={self.receiver.parse_error_count}  last sender={sender_text}"
            )
            message = (
                "Waiting for Unreal UDP packets...\n\n"
                f"Set Unreal RemoteIp to server IPv4: {local_ip_text}\n"
                f"RemotePort: {self.port}\n"
                "Allow inbound UDP 5000 in Windows Firewall"
            )
            if self.receiver.last_error:
                message += f"\n\nLast receive/parse error:\n{self.receiver.last_error}"
            self.center_text(width, height, message)
            return

        age = time.time() - packet.received_at
        sender = f"{packet.sender[0]}:{packet.sender[1]}" if packet.sender else "unknown"
        hz = self.packet_count / max(time.time() - self.started_at, 0.001)

        if age > 1.0:
            self.status.set(f"No recent packets - last packet {age:.1f}s ago")
        elif packet.active:
            self.status.set(f"Tracking active - sender {sender}")
        else:
            self.status.set(f"Tracking inactive - {packet.reason or 'unknown reason'}")

        self.metrics.set(
            f"valid packets={self.packet_count}  UDP datagrams={self.receiver.datagram_count}  "
            f"parse errors={self.receiver.parse_error_count}  average={hz:.1f} Hz  "
            f"age={age:.3f}s  confidence={packet.confidence:.3f}"
        )
        self.update_table(packet)

        if not packet.joints:
            self.center_text(width, height, packet.reason or "No valid joints")
            return

        visual_joints = self.with_virtual_torso_center(packet.joints)
        points = self.project(visual_joints, width, height)

        for start, end in SEGMENTS:
            if start not in points or end not in points:
                continue
            color = HEAD if end == TORSO_CENTER else LEFT if end.startswith("L_") else RIGHT
            self.canvas.create_line(*points[start], *points[end], fill=color, width=4)

        for name, (x, y) in points.items():
            color = HEAD if name in ("Head", TORSO_CENTER) else LEFT if name.startswith("L_") else RIGHT
            radius = 9 if name == "Head" else 7 if name == TORSO_CENTER else 6
            self.canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=color,
                outline="#f8fafc",
            )
            self.canvas.create_text(
                x + 10,
                y - 9,
                text=name,
                anchor="w",
                fill=TEXT,
                font=("Segoe UI", 9),
            )

    def draw_grid(self, width: int, height: int) -> None:
        for x in range(0, width, 50):
            self.canvas.create_line(x, 0, x, height, fill=GRID)
        for y in range(0, height, 50):
            self.canvas.create_line(0, y, width, y, fill=GRID)
        self.canvas.create_text(
            14,
            14,
            text=f"UDP SERVER {self.host}:{self.port}",
            anchor="nw",
            fill=MUTED,
            font=("Segoe UI", 10, "bold"),
        )

    def center_text(self, width: int, height: int, text: str) -> None:
        self.canvas.create_text(
            width / 2,
            height / 2,
            text=text,
            fill=MUTED,
            font=("Segoe UI", 15, "bold"),
        )

    @staticmethod
    def with_virtual_torso_center(joints: dict[str, Joint]) -> dict[str, Joint]:
        visual_joints = dict(joints)
        left_upper_arm = joints.get("L_UpperArm")
        right_upper_arm = joints.get("R_UpperArm")

        if left_upper_arm and right_upper_arm:
            visual_joints[TORSO_CENTER] = Joint(
                name=TORSO_CENTER,
                pos=tuple(
                    (left + right) / 2.0
                    for left, right in zip(left_upper_arm.pos, right_upper_arm.pos)
                ),
            )

        return visual_joints

    @staticmethod
    def project(
        joints: dict[str, Joint],
        width: int,
        height: int,
    ) -> dict[str, tuple[float, float]]:
        y_values = [joint.pos[1] for joint in joints.values()]
        z_values = [joint.pos[2] for joint in joints.values()]
        min_y, max_y = min(y_values), max(y_values)
        min_z, max_z = min(z_values), max(z_values)
        span_y = max(max_y - min_y, 30.0)
        span_z = max(max_z - min_z, 30.0)
        scale = min((width - 140.0) / span_y, (height - 140.0) / span_z)
        center_y = (min_y + max_y) / 2.0
        center_z = (min_z + max_z) / 2.0

        return {
            name: (
                width / 2.0 + (joint.pos[1] - center_y) * scale,
                height / 2.0 - (joint.pos[2] - center_z) * scale,
            )
            for name, joint in joints.items()
        }

    def update_table(self, packet: Packet) -> None:
        for name in JOINT_NAMES:
            joint = packet.joints.get(name)
            if not joint:
                self.table.item(name, values=(name, "--", "--", "--", "--"))
                continue

            x, y, z = joint.pos
            angles = joint.relative_deg
            self.table.item(
                name,
                values=(
                    name,
                    f"{x:6.1f}, {y:6.1f}, {z:6.1f}",
                    f"{angles['roll']:6.1f}",
                    f"{angles['pitch']:6.1f}",
                    f"{angles['yaw']:6.1f}",
                ),
            )

    def close(self) -> None:
        self.receiver.stop()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="UDP server for UE body-tracking visualization."
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Local bind address. Keep 0.0.0.0 to receive from another PC.",
    )
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    print("=" * 72)
    print(" UE Body Tracking Visualizer UDP Server")
    print("=" * 72)
    print(f" Bind address : {args.host}:{args.port}")
    print(" Important    : Stop udp_body_tracking_receiver_test.py first.")
    print("-" * 72)

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"[ERROR] Tkinter GUI could not start: {exc}")
        print("Ubuntu desktop: sudo apt install python3-tk")
        print("SSH/headless: run with X forwarding or use the console receiver test.")
        return

    Visualizer(root, args.host, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()
