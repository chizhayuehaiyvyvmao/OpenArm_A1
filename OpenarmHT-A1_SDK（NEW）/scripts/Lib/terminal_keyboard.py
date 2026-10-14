#!/usr/bin/env python3
"""SSH 终端单键监听，提供 pynput.keyboard 的最小兼容接口。"""

import os
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass


@dataclass(frozen=True)
class _KeyCode:
    char: str

    @classmethod
    def from_char(cls, char):
        return cls(str(char))


@dataclass(frozen=True)
class _NamedKey:
    name: str


class _Key:
    esc = _NamedKey("esc")


class _Listener:
    """在 TTY 中读取按键，并在停止重复后合成松键事件。"""

    def __init__(self, on_press=None, on_release=None, release_timeout=0.18):
        self.on_press = on_press
        self.on_release = on_release
        self.release_timeout = float(release_timeout)
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return self
        if not sys.stdin.isatty():
            raise RuntimeError("当前标准输入不是交互终端，无法读取键盘")
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="terminal-keyboard-listener",
            daemon=True,
        )
        self._thread.start()
        return self

    def stop(self):
        self._stop_event.set()
        if (
            self._thread is not None
            and self._thread.is_alive()
            and threading.current_thread() is not self._thread
        ):
            self._thread.join(timeout=1.0)

    def join(self, timeout=None):
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    def _call(self, callback, key):
        if callback is None:
            return True
        return callback(key) is not False

    def _run(self):
        file_descriptor = sys.stdin.fileno()
        original_settings = termios.tcgetattr(file_descriptor)
        held_keys = {}
        try:
            tty.setcbreak(file_descriptor)
            while not self._stop_event.is_set():
                now = time.monotonic()
                expired = [
                    key for key, deadline in held_keys.items()
                    if now >= deadline
                ]
                for key in expired:
                    held_keys.pop(key, None)
                    if not self._call(self.on_release, key):
                        self._stop_event.set()
                        break

                readable, _, _ = select.select(
                    [file_descriptor],
                    [],
                    [],
                    0.02,
                )
                if not readable:
                    continue

                raw = os.read(file_descriptor, 1)
                if not raw:
                    continue
                if raw == b"\x1b":
                    self._call(self.on_release, _Key.esc)
                    self._stop_event.set()
                    break

                try:
                    char = raw.decode("utf-8")
                except UnicodeDecodeError:
                    continue
                if not char.isprintable():
                    continue

                key = _KeyCode(char)
                if not self._call(self.on_press, key):
                    self._stop_event.set()
                    break
                held_keys[key] = time.monotonic() + self.release_timeout
        finally:
            for key in list(held_keys):
                self._call(self.on_release, key)
            termios.tcsetattr(
                file_descriptor,
                termios.TCSADRAIN,
                original_settings,
            )


class _KeyboardNamespace:
    Key = _Key
    KeyCode = _KeyCode
    Listener = _Listener


keyboard = _KeyboardNamespace()

