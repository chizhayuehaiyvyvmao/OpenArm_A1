#!/usr/bin/env python3
"""Keyboard-driven terminal UI for Quest_controller_stream (no Qt)."""

from __future__ import annotations

import argparse
from collections import deque
import curses
import json
import pathlib
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Any, Callable


BASE_DIR = pathlib.Path(__file__).resolve().parent
SERVER_SCRIPT = BASE_DIR / "start_stream_server.py"
SETTINGS_FILE = BASE_DIR / ".quest_stream_tui.json"
PACKET_FORMAT = "40f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

DEFAULTS: dict[str, Any] = {
    "mode": "wifi",
    "udp_ip": "127.0.0.1",
    "udp_port": 5005,
    "http_host": "0.0.0.0",
    "http_port": 8080,
    "ws_host": "0.0.0.0",
    "ws_port": 8765,
    "verify_port": 5005,
    "force_https": False,
    "cert_file": ".certs/quest-controller.crt",
    "key_file": ".certs/quest-controller.key",
}


def detect_local_ip() -> str:
    sock: socket.socket | None = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        return str(sock.getsockname()[0])
    except OSError:
        return "127.0.0.1"
    finally:
        if sock is not None:
            sock.close()


def load_settings() -> dict[str, Any]:
    settings = dict(DEFAULTS)
    try:
        payload = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
        if isinstance(payload, dict):
            for key in settings:
                if key in payload:
                    settings[key] = payload[key]
    except (OSError, json.JSONDecodeError):
        pass
    return settings


def save_settings(settings: dict[str, Any]) -> None:
    SETTINGS_FILE.write_text(
        json.dumps(settings, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def tcp_port_in_use(port: int) -> bool:
    """Return True when a local IPv4 TCP listener already owns the port."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.2)
    try:
        return sock.connect_ex(("127.0.0.1", int(port))) == 0
    finally:
        sock.close()


def decode_hand(values: tuple[float, ...]) -> dict[str, Any]:
    return {
        "seq": int(values[0]),
        "valid": bool(int(values[2])),
        "pos": tuple(values[3:6]),
        "quat": tuple(values[6:10]),
        "stick": tuple(values[10:12]),
        "trigger": values[12],
        "squeeze": values[13],
        "stick_click": bool(int(values[14])),
        "primary": bool(int(values[15])),
        "secondary": bool(int(values[16])),
        "trigger_pressed": bool(int(values[17])),
        "squeeze_pressed": bool(int(values[18])),
    }


def fmt_vec(values: tuple[float, ...]) -> str:
    return "(" + ", ".join(f"{value:+.3f}" for value in values) + ")"


class PacketMonitor:
    def __init__(self, port: int, notify: Callable[[str], None]) -> None:
        self.port = int(port)
        self.notify = notify
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.sock: socket.socket | None = None
        self.packet_count = 0
        self.last_packet_at: float | None = None
        self.latest_left: dict[str, Any] | None = None
        self.latest_right: dict[str, Any] | None = None
        self.source = "-"
        self.error: str | None = None
        self.lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self.stop_event.clear()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.thread = None
        self.sock = None

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock = sock
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", self.port))
        except OSError as exc:
            self.error = str(exc)
            self.notify(
                f"UDP {self.port} bind failed: {exc}; stop robot controller before Verify"
            )
            return
        sock.settimeout(0.5)
        self.notify(f"Verify listening on UDP :{self.port}")
        try:
            while not self.stop_event.is_set():
                try:
                    data, address = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if len(data) != PACKET_SIZE:
                    self.notify(
                        f"Ignored {len(data)} bytes from {address[0]}:{address[1]}"
                    )
                    continue
                values = struct.unpack(PACKET_FORMAT, data)
                with self.lock:
                    self.packet_count += 1
                    self.last_packet_at = time.time()
                    self.latest_left = decode_hand(values[:20])
                    self.latest_right = decode_hand(values[20:])
                    self.source = f"{address[0]}:{address[1]}"
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self.notify("Verify stopped")


class QuestStreamTUI:
    PAGES = ("Controls", "Left Hand", "Right Hand", "Logs")
    MENU = (
        "mode",
        "force_https",
        "udp_ip",
        "udp_port",
        "http_host",
        "http_port",
        "ws_host",
        "ws_port",
        "cert_file",
        "key_file",
        "verify_port",
        "start_server",
        "stop_server",
        "start_verify",
        "stop_verify",
        "save",
        "reset",
        "quit",
    )

    def __init__(self) -> None:
        self.settings = load_settings()
        self.server: subprocess.Popen[str] | None = None
        self.server_reader: threading.Thread | None = None
        self.logs: deque[str] = deque(maxlen=200)
        self.log_lock = threading.Lock()
        self.monitor = PacketMonitor(int(self.settings["verify_port"]), self.log)
        self.selected = 0
        self.page = 0
        self.running = True
        self.last_action = "Ready"

    @property
    def server_running(self) -> bool:
        return self.server is not None and self.server.poll() is None

    def log(self, message: str) -> None:
        with self.log_lock:
            self.logs.append(f"{time.strftime('%H:%M:%S')} {message}")
        self.last_action = message

    def current_url(self) -> str:
        mode = str(self.settings["mode"])
        if mode == "usb":
            host = "localhost"
            secure = bool(self.settings["force_https"])
        else:
            host = detect_local_ip()
            secure = True
        page_scheme = "https" if secure else "http"
        ws_scheme = "wss" if secure else "ws"
        return (
            f"{page_scheme}://{host}:{self.settings['http_port']}/index.html"
            f"?ws={ws_scheme}://{host}:{self.settings['ws_port']}"
        )

    def server_command(self) -> list[str]:
        mode = str(self.settings["mode"])
        command = [
            sys.executable,
            "-u",
            str(SERVER_SCRIPT),
            "--http-host",
            str(self.settings["http_host"]),
            "--http-port",
            str(self.settings["http_port"]),
            "--ws-host",
            str(self.settings["ws_host"]),
            "--ws-port",
            str(self.settings["ws_port"]),
            "--udp-ip",
            str(self.settings["udp_ip"]),
            "--udp-port",
            str(self.settings["udp_port"]),
            "--cert-file",
            str(self.settings["cert_file"]),
            "--key-file",
            str(self.settings["key_file"]),
            "--adb",
            "auto" if mode == "usb" else "never",
        ]
        if bool(self.settings["force_https"]):
            command.append("--https")
        return command

    def start_server(self) -> None:
        if self.server_running:
            self.log("Server already running")
            return
        if not SERVER_SCRIPT.is_file():
            self.log(f"Missing {SERVER_SCRIPT}")
            return
        busy_ports = [
            port
            for port in {
                int(self.settings["http_port"]),
                int(self.settings["ws_port"]),
            }
            if tcp_port_in_use(port)
        ]
        if busy_ports:
            ports = ", ".join(str(port) for port in sorted(busy_ports))
            self.log(f"Start blocked: TCP port already in use: {ports}")
            return
        try:
            self.server = subprocess.Popen(
                self.server_command(),
                cwd=str(BASE_DIR),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except OSError as exc:
            self.server = None
            self.log(f"Server start failed: {exc}")
            return
        self.server_reader = threading.Thread(target=self._read_server, daemon=True)
        self.server_reader.start()
        self.log("Server starting")

    def _read_server(self) -> None:
        process = self.server
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self.log("server: " + line.rstrip())
        code = process.wait()
        self.log(f"Server exited with code {code}")

    def stop_server(self) -> None:
        process = self.server
        if process is None or process.poll() is not None:
            self.server = None
            self.log("Server not running")
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        self.server = None
        self.log("Server stopped")

    def reset_monitor_port(self) -> None:
        was_running = self.monitor.running
        self.monitor.stop()
        self.monitor = PacketMonitor(int(self.settings["verify_port"]), self.log)
        if was_running:
            self.monitor.start()

    def edit_value(self, screen: Any, key: str) -> None:
        if self.server_running:
            self.log("Stop server before editing settings")
            return
        height, width = screen.getmaxyx()
        prompt = f"New {key}: "
        curses.echo()
        curses.curs_set(1)
        try:
            screen.move(height - 1, 0)
            screen.clrtoeol()
            screen.addnstr(height - 1, 0, prompt, max(1, width - 1))
            raw = screen.getstr(height - 1, len(prompt), max(1, width - len(prompt) - 2))
            value = raw.decode("utf-8").strip()
        finally:
            curses.noecho()
            curses.curs_set(0)
        if not value:
            return
        try:
            if key.endswith("_port"):
                parsed: Any = int(value)
                if not 1 <= parsed <= 65535:
                    raise ValueError("port must be 1..65535")
            elif key in {"udp_ip", "http_host", "ws_host"}:
                socket.getaddrinfo(value, None)
                parsed = value
            elif key in {"cert_file", "key_file"}:
                parsed = value
            else:
                return
            self.settings[key] = parsed
            save_settings(self.settings)
            if key == "verify_port":
                self.reset_monitor_port()
            self.log(f"Set {key}={parsed}")
        except Exception as exc:
            self.log(f"Invalid {key}: {exc}")

    def adjust_selected(self, direction: int) -> None:
        key = self.MENU[self.selected]
        if self.server_running and key in {
            "mode",
            "udp_ip",
            "udp_port",
            "http_host",
            "http_port",
            "ws_host",
            "ws_port",
            "cert_file",
            "key_file",
            "verify_port",
            "force_https",
        }:
            self.log("Stop server before editing settings")
            return
        if key == "mode":
            self.settings["mode"] = (
                "usb" if self.settings["mode"] == "wifi" else "wifi"
            )
        elif key == "force_https":
            self.settings["force_https"] = not bool(self.settings["force_https"])
        elif key in {"udp_port", "http_port", "ws_port", "verify_port"}:
            value = int(self.settings[key]) + direction
            self.settings[key] = max(1, min(65535, value))
            if key == "verify_port":
                self.reset_monitor_port()
        elif key in {"udp_ip", "http_host", "ws_host"}:
            if key == "udp_ip":
                self.settings[key] = (
                    detect_local_ip()
                    if self.settings[key] == "127.0.0.1"
                    else "127.0.0.1"
                )
            else:
                self.settings[key] = (
                    "127.0.0.1"
                    if self.settings[key] == "0.0.0.0"
                    else "0.0.0.0"
                )
        else:
            return
        save_settings(self.settings)

    def activate_selected(self, screen: Any) -> None:
        key = self.MENU[self.selected]
        self.last_action = f"Enter: {self.item_text(key).strip()}"
        if key == "mode" or key == "force_https":
            self.adjust_selected(1)
            self.last_action = f"Changed: {self.item_text(key).strip()}"
        elif key == "start_server":
            self.start_server()
        elif key == "stop_server":
            self.stop_server()
        elif key == "start_verify":
            self.monitor.start()
        elif key == "stop_verify":
            self.monitor.stop()
        elif key == "save":
            save_settings(self.settings)
            self.log("Settings saved")
        elif key == "reset":
            if self.server_running:
                self.log("Stop server before resetting defaults")
            else:
                self.settings = dict(DEFAULTS)
                save_settings(self.settings)
                self.reset_monitor_port()
                self.log("Defaults restored")
        elif key in {
            "udp_ip",
            "udp_port",
            "http_host",
            "http_port",
            "ws_host",
            "ws_port",
            "cert_file",
            "key_file",
            "verify_port",
        }:
            self.edit_value(screen, key)
        elif key == "quit":
            self.running = False

    @staticmethod
    def is_enter_key(key: Any) -> bool:
        """Accept Enter variants emitted by common SSH and desktop terminals."""
        return key in ("\n", "\r", 10, 13, curses.KEY_ENTER)

    def item_text(self, key: str) -> str:
        if key == "mode":
            return f"Mode                    {str(self.settings['mode']).upper()}"
        if key == "start_server":
            return "Start Server"
        if key == "stop_server":
            return "Stop Server"
        if key == "start_verify":
            return "Start Verify"
        if key == "stop_verify":
            return "Stop Verify"
        if key == "save":
            return "Save Settings"
        if key == "reset":
            return "Reset Defaults"
        if key == "udp_ip":
            return f"UDP destination IP      {self.settings['udp_ip']}"
        if key == "udp_port":
            return f"UDP port                {self.settings['udp_port']}"
        if key == "http_port":
            return f"HTTP port               {self.settings['http_port']}"
        if key == "http_host":
            return f"HTTP host               {self.settings['http_host']}"
        if key == "ws_port":
            return f"WebSocket port          {self.settings['ws_port']}"
        if key == "ws_host":
            return f"WebSocket host          {self.settings['ws_host']}"
        if key == "cert_file":
            return f"Cert file               {self.settings['cert_file']}"
        if key == "key_file":
            return f"Key file                {self.settings['key_file']}"
        if key == "verify_port":
            return f"Verify port             {self.settings['verify_port']}"
        if key == "force_https":
            return f"Force HTTPS/WSS         {bool(self.settings['force_https'])}"
        return "Quit"

    @staticmethod
    def put(screen: Any, row: int, col: int, text: str, attr: int = 0) -> None:
        height, width = screen.getmaxyx()
        if row < 0 or row >= height or col >= width:
            return
        try:
            screen.addnstr(row, col, text, max(0, width - col - 1), attr)
        except curses.error:
            pass

    def box(self, screen: Any, y: int, x: int, h: int, w: int, title: str) -> None:
        if h < 3 or w < 4:
            return
        try:
            screen.addch(y, x, curses.ACS_ULCORNER)
            screen.hline(y, x + 1, curses.ACS_HLINE, w - 2)
            screen.addch(y, x + w - 1, curses.ACS_URCORNER)
            screen.vline(y + 1, x, curses.ACS_VLINE, h - 2)
            screen.vline(y + 1, x + w - 1, curses.ACS_VLINE, h - 2)
            screen.addch(y + h - 1, x, curses.ACS_LLCORNER)
            screen.hline(y + h - 1, x + 1, curses.ACS_HLINE, w - 2)
            screen.addch(y + h - 1, x + w - 1, curses.ACS_LRCORNER)
            self.put(screen, y, x + 2, f" {title} ", curses.A_BOLD)
        except curses.error:
            pass

    def focus_attr(self, key: str) -> int:
        return curses.A_REVERSE if self.MENU[self.selected] == key else 0

    def field(
        self,
        screen: Any,
        y: int,
        x: int,
        label: str,
        key: str,
        width: int,
    ) -> None:
        value = str(self.settings[key])
        text = f"{label:<12} {value}"
        self.put(screen, y, x, text[:width].ljust(width), self.focus_attr(key))

    def button(self, screen: Any, y: int, x: int, key: str, label: str) -> int:
        text = f"[ {label} ]"
        self.put(screen, y, x, text, self.focus_attr(key) | curses.A_BOLD)
        return x + len(text) + 1

    def draw_hand_panel(
        self,
        screen: Any,
        y: int,
        x: int,
        h: int,
        w: int,
        hand: dict[str, Any] | None,
    ) -> None:
        label = "LEFT" if self.page == 1 else "RIGHT"
        self.box(screen, y, x, h, w, f"{label.title()} Controller Telemetry")
        if hand is None:
            self.put(screen, y + 3, x + 3, f"{label} HAND     [missing]", curses.A_BOLD)
            self.put(screen, y + 5, x + 3, "Waiting for Quest controller packets...", curses.A_DIM)
            return
        badge = "tracking" if hand["valid"] else "missing"
        self.put(screen, y + 3, x + 3, f"{label} HAND", curses.A_BOLD)
        self.put(
            screen,
            y + 3,
            x + max(16, w - 16),
            f"[{badge}]",
            curses.A_BOLD if hand["valid"] else curses.A_DIM,
        )
        values = (
            ("Seq", str(hand["seq"])),
            ("Position", fmt_vec(hand["pos"])),
            ("Quaternion", fmt_vec(hand["quat"])),
            ("Thumbstick", fmt_vec(hand["stick"])),
            ("Trigger", f"{hand['trigger']:.3f}"),
            ("Squeeze", f"{hand['squeeze']:.3f}"),
            ("Thumb Click", "ON" if hand["stick_click"] else "OFF"),
            ("Primary", "ON" if hand["primary"] else "OFF"),
            ("Secondary", "ON" if hand["secondary"] else "OFF"),
            ("Trigger Pressed", "ON" if hand["trigger_pressed"] else "OFF"),
            ("Squeeze Pressed", "ON" if hand["squeeze_pressed"] else "OFF"),
        )
        row = y + 5
        for title, value in values:
            if row >= y + h - 2:
                break
            self.put(screen, row, x + 3, f"{title:<18}", curses.A_DIM)
            self.put(screen, row, x + 22, value, curses.A_BOLD)
            row += 2 if title in {"Position", "Quaternion"} else 1

    def draw(self, screen: Any) -> None:
        screen.erase()
        height, width = screen.getmaxyx()
        if height < 24 or width < 80:
            self.put(screen, 0, 0, "Terminal too small; resize to at least 80x24")
            screen.refresh()
            return
        self.put(
            screen,
            0,
            0,
            " Quest Stream Console - Keyboard Terminal UI (no Qt) ",
            curses.A_REVERSE | curses.A_BOLD,
        )
        self.put(
            screen,
            1,
            1,
            "Tab/Shift+Tab or PgUp/PgDn page  1-4 direct  ↑/↓ select  ←/→ adjust  Enter execute  Q quit",
            curses.A_BOLD,
        )
        with self.monitor.lock:
            left = self.monitor.latest_left
            right = self.monitor.latest_right
            count = self.monitor.packet_count
            source = self.monitor.source
            packet_age = (
                time.time() - self.monitor.last_packet_at
                if self.monitor.last_packet_at is not None
                else None
            )
        age_text = f"{packet_age:.2f}s" if packet_age is not None else "never"
        server_state = "RUNNING" if self.server_running else "STOPPED"
        monitor_state = "RUNNING" if self.monitor.running else "STOPPED"
        page_x = 1
        for index, page_name in enumerate(self.PAGES):
            text = f" {index + 1}:{page_name} "
            attr = curses.A_REVERSE | curses.A_BOLD if index == self.page else curses.A_DIM
            self.put(screen, 2, page_x, text, attr)
            page_x += len(text) + 1
        card_width = max(18, (width - 5) // 4)
        summaries = (
            ("Server", server_state),
            ("Monitor", monitor_state),
            ("Packets", str(count)),
            ("Last Packet", age_text),
        )
        for index, (title, value) in enumerate(summaries):
            col = 1 + index * card_width
            self.put(screen, 3, col, f"{title}: ", curses.A_DIM)
            self.put(screen, 3, col + len(title) + 2, value, curses.A_BOLD)
        focus = self.item_text(self.MENU[self.selected]).strip()
        self.put(
            screen,
            4,
            1,
            f"Focus: {focus}  |  Result: {self.last_action}",
            curses.A_BOLD,
        )

        page_y = 5
        if self.page == 0:
            control_h = 9
            self.box(screen, page_y, 0, control_h, width, "Launch Controls")
            half = (width - 6) // 2
            self.field(screen, page_y + 1, 2, "Mode", "mode", half)
            self.field(screen, page_y + 1, 3 + half, "Force HTTPS", "force_https", half)
            self.field(screen, page_y + 2, 2, "UDP IP", "udp_ip", half)
            self.field(screen, page_y + 2, 3 + half, "UDP Port", "udp_port", half)
            self.field(screen, page_y + 3, 2, "HTTP Host", "http_host", half)
            self.field(screen, page_y + 3, 3 + half, "HTTP Port", "http_port", half)
            self.field(screen, page_y + 4, 2, "WS Host", "ws_host", half)
            self.field(screen, page_y + 4, 3 + half, "WS Port", "ws_port", half)
            self.field(screen, page_y + 5, 2, "Cert File", "cert_file", width - 4)
            self.field(screen, page_y + 6, 2, "Key File", "key_file", half)
            self.field(screen, page_y + 6, 3 + half, "Verify Port", "verify_port", half)

            action_y, action_h = page_y + control_h, 4
            self.box(screen, action_y, 0, action_h, width, "Actions")
            x = 2
            x = self.button(screen, action_y + 1, x, "start_server", "Start Server")
            x = self.button(screen, action_y + 1, x, "stop_server", "Stop Server")
            x = self.button(screen, action_y + 1, x, "start_verify", "Start Verify")
            x = self.button(screen, action_y + 1, x, "stop_verify", "Stop Verify")
            x = self.button(screen, action_y + 2, 2, "save", "Save Settings")
            x = self.button(screen, action_y + 2, x, "reset", "Reset Defaults")
            self.button(screen, action_y + 2, x, "quit", "Quit")

            endpoint_y = action_y + action_h
            endpoint_h = height - endpoint_y
            self.box(screen, endpoint_y, 0, endpoint_h, width, "Resolved Endpoints")
            ip = detect_local_ip()
            self.put(screen, endpoint_y + 1, 2, f"USB URL    http://localhost:{self.settings['http_port']}/index.html?ws=ws://localhost:{self.settings['ws_port']}")
            self.put(screen, endpoint_y + 2, 2, f"WiFi URL   https://{ip}:{self.settings['http_port']}/index.html?ws=wss://{ip}:{self.settings['ws_port']}")
            self.put(screen, endpoint_y + 3, 2, f"WS URL     {'wss' if self.settings['mode'] == 'wifi' else 'ws'}://{ip}:{self.settings['ws_port']}")
            self.put(screen, endpoint_y + 4, 2, f"Last Source  {source}")
        elif self.page in {1, 2}:
            active = left if self.page == 1 else right
            self.draw_hand_panel(screen, page_y, 0, height - page_y, width, active)
        else:
            log_h = height - page_y
            self.box(screen, page_y, 0, log_h, width, "Server Log / UDP Verify Log")
            with self.log_lock:
                lines = list(self.logs)[-(log_h - 2) :]
            for offset, line in enumerate(lines, start=1):
                self.put(screen, page_y + offset, 1, line)
        screen.refresh()

    def close(self) -> None:
        self.monitor.stop()
        self.stop_server()

    def curses_main(self, screen: Any) -> None:
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        curses.noecho()
        curses.cbreak()
        screen.keypad(True)
        screen.timeout(100)
        while self.running:
            self.draw(screen)
            try:
                key = screen.get_wch()
            except curses.error:
                continue
            if key in ("\t", 9, curses.KEY_NPAGE):
                self.page = (self.page + 1) % len(self.PAGES)
            elif key in (curses.KEY_BTAB, curses.KEY_PPAGE):
                self.page = (self.page - 1) % len(self.PAGES)
            elif key in ("1", "2", "3", "4", ord("1"), ord("2"), ord("3"), ord("4")):
                self.page = int(chr(key) if isinstance(key, int) else key) - 1
            elif key in (curses.KEY_UP, "k", ord("k")) and self.page == 0:
                self.selected = (self.selected - 1) % len(self.MENU)
            elif key in (curses.KEY_DOWN, "j", ord("j")) and self.page == 0:
                self.selected = (self.selected + 1) % len(self.MENU)
            elif key in (curses.KEY_LEFT, "h", ord("h")) and self.page == 0:
                self.adjust_selected(-1)
            elif key in (curses.KEY_RIGHT, "l", ord("l")) and self.page == 0:
                self.adjust_selected(1)
            elif (self.is_enter_key(key) or key in (" ", ord(" "))) and self.page == 0:
                self.activate_selected(screen)
            elif key in ("e", "E", ord("e"), ord("E")) and self.page == 0:
                selected_key = self.MENU[self.selected]
                if selected_key in {
                    "udp_ip",
                    "udp_port",
                    "http_host",
                    "http_port",
                    "ws_host",
                    "ws_port",
                    "cert_file",
                    "key_file",
                    "verify_port",
                }:
                    self.edit_value(screen, selected_key)
            elif key in ("q", "Q", ord("q"), ord("Q")):
                self.running = False

    def run(self) -> int:
        try:
            curses.wrapper(self.curses_main)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()
        return 0


def check_installation() -> int:
    import websockets

    print("Quest terminal UI check OK")
    print(f"python={sys.executable}")
    print(f"websockets={websockets.__version__}")
    print(f"server_script={SERVER_SCRIPT}")
    print(f"server_script_exists={SERVER_SCRIPT.is_file()}")
    print(f"packet_size={PACKET_SIZE}")
    return 0 if SERVER_SCRIPT.is_file() and PACKET_SIZE == 160 else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate imports and files without opening the interactive TUI.",
    )
    args = parser.parse_args()
    if args.check:
        raise SystemExit(check_installation())
    raise SystemExit(QuestStreamTUI().run())
