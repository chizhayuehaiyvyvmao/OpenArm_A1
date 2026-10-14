# Quest Controller Stream - Terminal UI

This full-screen keyboard terminal entry keeps the configuration, start/stop,
UDP verification, and live left/right controller display of the Qt UI without
requiring PyQt5 or typed commands.

## Start

```bash
cd /home/nvidia/OpenarmHT-A1_SDK
./run_quest_stream_tui.sh
```

The default mode is WiFi, forwarding to the robot-side receiver on
`127.0.0.1:5005`.

```text
Up / Down       select a menu row
Left / Right    switch mode or adjust a value
Enter / Space   activate the selected row
E               edit the selected address or port
Tab / Shift-Tab next / previous page
PageDown/PageUp next / previous page
1 / 2 / 3 / 4   open Controls / Left / Right / Logs directly
Q               quit and stop the stream service
```

Select `Stream server [START]` and press Enter. Open the HTTPS URL shown at the
top in Quest Browser, accept the self-signed certificate the first time, then
click `Start XR Stream` in the page.

## Pages

```text
1 Controls     Qt-like launch form, action buttons and resolved endpoints
2 Left Hand    complete left controller telemetry
3 Right Hand   complete right controller telemetry
4 Logs         full server and UDP verification log
```

On Controls, use Up/Down to highlight fields and buttons. Start/Stop Server,
Start/Stop Verify, Save Settings, Reset Defaults, and Quit match the Qt action
row. The `Focus` line shows the item that Enter will activate, while `Result`
shows the immediate outcome. If HTTP or WebSocket is already listening, Start
is blocked with the occupied TCP port instead of repeatedly launching a process
that exits. The Left/Right pages display pose, quaternion, thumbstick, trigger,
squeeze, and all button states without truncating the Controls page.

Stop Verify before starting `scripts/8_vr_dual_cartesian_qp_control.py`, which
owns the same UDP port while controlling the robot.

## Robot-side controller

After Quest packets are visible and `verify-stop` has released UDP 5005, use
another terminal with the `panthera` environment. The controller initializes
real robot hardware, so run it only when the robot is ready and supervised:

```bash
cd /home/nvidia/OpenarmHT-A1_SDK/scripts
conda run -n panthera python 8_vr_dual_cartesian_qp_control.py
```

## Environment

The terminal UI itself uses only Python's standard library. The underlying
stream service requires `websockets`; it is installed in the remote
`panthera` environment. PyQt5 is intentionally not required.

USB mode additionally needs the system `adb` command. This host currently has
OpenSSL for WiFi HTTPS/WSS but no ADB, so WiFi mode is ready now and USB mode
requires Android platform tools later.
