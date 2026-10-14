# openarm_HT Python SDK

**English** | [简体中文](README.md)

`openarm_HT` is a Python SDK for the OpenArm-HT A1 dual-arm robot. Each side has seven arm joints and one independent gripper motor. The SDK includes motor communication, joint and MIT control, Pinocchio kinematics/dynamics, trajectory generation, constrained QP IK, trajectory recording/playback, SSH-terminal keyboard control, and Quest VR examples.

> [!CAUTION]
> Every motion example connects to real motors. Before a first run, clear the workspace, verify the emergency stop, supervise the robot, and be ready to press `Ctrl+C`. Firmware position timeout/limits are not fully enabled in the current motor configuration; Python and URDF limits do not replace mechanical stops or an emergency stop.

## Project Overview

This repository is a complete stack from the native motor SDK to dual-arm applications, not only a collection of examples:

```text
OpenArm-HT A1 hardware / HighTorque communication board
        │
        ▼
HighTorque native C++ SDK (external dependency)
        │
        ▼
src/bindings.cpp + pybind11
        │
        ▼
hightorque_robot.Robot / Motor (prebuilt wheel or source build)
        │
        ▼
scripts/Lib/Openarm.py (7-DoF control, kinematics, dynamics, trajectories)
        │
        ├── joint / MIT / impedance / moveJ / moveL
        ├── Pinocchio FK / IK / Jacobian / dynamics
        ├── OSQP velocity-level QP IK
        ├── JSONL recording and playback
        ├── SSH keyboard control
        └── Quest VR control
```

The repository also contains:

- the complete OpenarmV2 dual-arm URDF and STL meshes;
- prebuilt motor SDK wheels for x86_64/aarch64 and Python 3.9–3.12;
- an embedded Quest WebXR → WebSocket → UDP relay project;
- an Intel RealSense D435i RGB+Depth web viewer;
- an offline V2 model regression tool and low-level single-motor examples.

## Repository Layout

```text
OpenarmHT-A1_SDK/
├── CMakeLists.txt                     # pybind11 native-extension build
├── setup.py / pyproject.toml          # Python packaging
├── requirements.txt                   # high-level Python dependencies
├── motor_whl/                         # prebuilt hightorque_robot wheels
├── src/
│   └── bindings.cpp                   # C++ Robot/Motor Python bindings
├── OpenarmV2_URDF/
│   ├── urdf/OpenarmV2_URDF.urdf       # 14-joint dual-arm model
│   ├── meshes/                        # base plus left/right J1–J7 STL
│   ├── launch/                        # ROS/ROS 2 display launches
│   └── config/                        # joint-state configuration
├── robot_param/
│   ├── Openarm_left.yaml
│   ├── Openarm_right.yaml
│   └── motor_param/                   # board, motor ID/type, and serial config
├── scripts/
│   ├── Lib/                           # Openarm, QP, recorder, TTY, mirror tools
│   ├── motor_example/                 # low-level single-motor examples
│   ├── 0_* ... 8_*                    # dual-arm application examples
│   └── openarm_trajectory_test1.jsonl
├── Quest_controller_stream/
│   ├── index.html                     # Quest WebXR page
│   ├── quest_stream_ui.py             # PyQt5 desktop UI
│   ├── quest_stream_tui.py            # terminal UI without PyQt5
│   ├── start_stream_server.py         # HTTP(S)+WS(S)→UDP service
│   └── udp_receiver_test.py
├── run_quest_stream_tui.sh            # root-level Quest TUI launcher
├── tools/
│   ├── test_openarm_v2_model.py       # hardware-free model regression
│   ├── d435i_web_viewer               # built viewer
│   ├── d435i_web_viewer.cpp           # RealSense/OpenCV source
│   └── start_d435i_web_viewer.sh
├── images/                            # README assets
├── README.md                          # Chinese
└── README_EN.md                       # English
```

## Current Model and Mirroring Convention

<p align="center">
  <img src="images/base_joint.png" alt="OpenArm base coordinate frame" width="520">
</p>

- Full model: `OpenarmV2_URDF/urdf/OpenarmV2_URDF.urdf`
- Model DOF: seven revolute joints per arm, fourteen total
- Left TCP: `l_joint7_link`
- Right TCP: `r_joint7_link`
- Motor-to-model signs: `joint_signs = [1, -1, 1, 1, 1, -1, 1]`
- Strict mirrored joint target: `q_right = -q_left`
- Right J1/J4 limits are the sign-reversed mirror of the left limits:
  - J1: `[-1.6581, 3.0543] rad`
  - J4: `[-1.0472, 2.2689] rad`
- The OpenarmV2 URDF does not contain gripper collision geometry. The gripper motor is still controlled independently by the low-level SDK.

The mirrored limits, all seven joint directions, FK, Jacobian twists, gravity signs, and a 101-sample startup trajectory have passed offline regression. Physical motor direction must still be confirmed with small supervised steps during the first hardware run.

## Requirements

- Ubuntu 20.04/22.04 on x86_64 or aarch64
- Python 3.9–3.12 (Python 3.10 recommended)
- HighTorque communication board and OpenArm-HT A1 hardware
- A matching `hightorque_robot` wheel
- PyYAML, Pinocchio (pip package `pin`), SciPy, and OSQP
- `matplotlib` for the plotting example
- A Quest data relay for the VR examples

## Installation

```bash
conda create -n panthera python=3.10
conda activate panthera
cd /home/nvidia/OpenarmHT-A1_SDK
```

Install the wheel matching the CPU architecture and Python version. For aarch64 and Python 3.10:

```bash
pip install motor_whl/hightorque_robot-1.0.0-cp310-cp310-linux_aarch64.whl
pip install -r requirements.txt
pip install matplotlib
```

Verify the environment:

```bash
python -c "import hightorque_robot; print('motor SDK OK')"
python -c "import pinocchio, scipy, yaml, osqp; print('robot dependencies OK')"
python -c "import matplotlib; print(matplotlib.__version__)"
```

### Prebuilt Wheels and Source Build

For robot deployment, install a prebuilt wheel from `motor_whl/` whenever possible. The repository provides:

- x86_64: `hightorque_robot 1.2.0` for CPython 3.9/3.10/3.11/3.12;
- aarch64: `hightorque_robot 1.0.0` for CPython 3.9/3.10/3.11/3.12.

Build from source only when the native HighTorque C++ SDK and its CMake package are installed. The build requires:

- CMake ≥ 3.12
- pybind11 ≥ 2.6
- HighTorque C++ SDK (`find_package(hightorque_robot)`)
- LCM
- yaml-cpp
- pthread and the realtime library

Example Ubuntu build dependencies:

```bash
sudo apt install cmake pkg-config liblcm-dev libyaml-cpp-dev
python -m pip install pybind11 setuptools wheel
python -m pip install .
```

`setup.py` invokes CMake to compile `src/bindings.cpp` into the `_hightorque_robot` Python extension. The source build does not download the vendor C++ SDK automatically.

### Version Notes

- CMake/pyproject metadata: `1.0.0`
- x86_64 prebuilt wheels: `1.2.0`
- aarch64 prebuilt wheels: `1.0.0`
- communication-board firmware, motor firmware, and the runtime C++ SDK report independent versions.

## Connection and Running Examples

<p align="center">
  <img src="images/Board.png" alt="HighTorque communication board wiring" width="640">
</p>

Check the communication boards:

```bash
ls /dev/ttyACM*
sudo chmod a+rw /dev/ttyACM*
```

Enter the script directory and run an example directly:

```bash
cd /home/nvidia/OpenarmHT-A1_SDK/scripts
python 0_dual_openarm_HT_get_state.py
```

The consolidated `0_`, `1_`, `2_`, `3_`, `4_`, `6_`, and `7_` examples require no command-line arguments. Select at startup:

```text
1. Left arm
2. Right arm
3. Dual arm
```

Dual-arm mode loads:

- `robot_param/Openarm_left.yaml`
- `robot_param/Openarm_right.yaml`

## Software Interfaces

### Low-Level `hightorque_robot` Bindings

The primary objects exposed by `src/bindings.cpp` are:

| Object | Main capabilities |
|---|---|
| `MotorState` | timestamp, ID, mode, fault, position, velocity, and torque feedback |
| `MotorVersion` | motor firmware version |
| `MotorParams` / `CANPortParams` / `CANBoardParams` / `RobotParams` | YAML configuration structures |
| `Motor` | position, velocity, torque, voltage, current, PosVel, MIT, stop/brake/reset |
| `Robot` | batched send, state/version query, zeroing, timeout, LCM, FDCAN reset, motor lookup |

Low-level examples are under `scripts/motor_example/`. They command motors directly and bypass high-level kinematics/trajectory safeguards; verify IDs, directions, zeros, and limits first.

### High-Level `Openarm` API

```python
from Lib import Openarm

left = Openarm("../robot_param/Openarm_left.yaml")

q = left.get_current_pos()
dq = left.get_current_vel()
tau = left.get_current_torque()

left.Joint_Pos_Vel(q, [0.2] * left.motor_count, left.max_torque)
fk = left.forward_kinematics(q)
J = left.get_jacobian(q)
g = left.get_Gravity(q)

left.gripper_open()
left.set_stop()
```

High-level capabilities include:

- joint position-velocity, velocity, moveJ, and five-parameter MIT control;
- gripper position/MIT, open, and close;
- FK, DLS IK, Jacobian, and manipulability;
- gravity, friction, Coriolis, mass matrix, and full dynamics;
- Cartesian path generation, time parameterization, spline smoothing, and moveL;
- septic interpolation;
- YAML/URDF joint and TCP consistency validation.

## Current Example Index

### 0: State and Zeroing

| File | Purpose |
|---|---|
| `0_dual_openarm_HT_get_state.py` | Left/right/dual position, velocity, torque, and gripper feedback |
| `0_dual_robot_set_zero.py` | Zeroing, keeping saved zero, state monitoring, and return-to-zero testing |

### 1: Joint Control and FK/IK

| File | Purpose |
|---|---|
| `1_dual_forward_kinematics_test.py` | Live FK and TCP pose |
| `1_dual_inverse_kinematics_test.py` | FK→IK consistency plus live joints/TCP |
| `1_dual_Joint_PD_control.py` | Joint MIT/PD control |
| `1_dual_Joint_PosVel_control.py` | Position-velocity and gripper demonstration |
| `1_dual_Joint_Vel_control.py` | J1 velocity oscillation |
| `1_dual_moveJ_control.py` | moveJ target, arrival error, and return to zero |

### 2: Dynamics Compensation and Impedance

| File | Purpose |
|---|---|
| `2_dual_gravity_compensation_control.py` | `0.8 × G(q)` gravity compensation |
| `2_dual_gravity_friction_compensation_control.py` | `0.8 × G(q) + friction` |
| `2_dual_Jointimpendence_control_with_gra_pd.py` | Joint impedance plus gravity feed-forward |
| `2_dual_Jointimpendence_control_with_gra_fri_pd.py` | Joint impedance plus gravity/friction feed-forward |
| `2_dual_inv_PosVel_control.py` | FK-generated reachable target, then IK→moveJ→zero |

### 3: Interpolation, Sine Trajectories, and Plotting

| File | Purpose |
|---|---|
| `3_interpolation_control_zeroVel.py` | Septic interpolation with zero intermediate velocity |
| `3_interpolation_control_nozeroVel.py` | Continuous interpolation with nonzero intermediate velocity |
| `3_dual_sin_trajectory_control.py` | Seven-joint sine trajectory and synchronized grippers |
| `3_sin_trajectory_control_with_plot.py` | Desired/actual plots and per-joint RMSE |
| `3_gravity_compensation_with_fk.py` | Gravity compensation with live FK output |

Plot files are saved automatically under `scripts/`:

- `sin_trajectory_tracking_left.png`
- `sin_trajectory_tracking_right.png`
- `sin_trajectory_tracking_dual.png`

### 4: QP Inverse Kinematics

| File | Purpose |
|---|---|
| `4_dual_QP_inverse_kinematics_control.py` | Left/right/dual MIT + bounded QP overhead raise, overhead circle, and zero return |

QP control uses one `PinocchioOSQPIK` per arm, J4 branch constraints, feedback tracking gates, and YAML/URDF limits. With the corrected mirrored right limits, the J1 overhead endpoints are symmetric.

### 5: Teleoperation and Trajectory Record/Playback

| File | Purpose |
|---|---|
| `5_teleop_control.py` | Legacy Leader/Follower teleoperation |
| `5_trajectory_record_replay.py` | Single/dual JSONL recording, history selection, and MIT playback |

```bash
python 5_trajectory_record_replay.py
```

### 6: Cartesian moveL

| File | Purpose |
|---|---|
| `6_moveL_pos_control.py` | Mirrored line, diagonal, and square paths |
| `6_moveL_rotate_control.py` | Mirrored X/Y/Z orientation and cone motion |

In dual mode, the two blocking `moveL` calls execute the same stage sequentially. This avoids running two Python trajectory loops in parallel and disrupting each arm's MIT timing. Position amplitudes were fully planned against the current V2 model.

### 7: SSH-Terminal Keyboard Control

| File | Purpose |
|---|---|
| `7_keyboard_cartesian_pos_control.py` | Incremental Cartesian pose IK |
| `7_keyboard_cartesian_vel_control.py` | Jacobian/DLS velocity control |
| `7_keyboard_cartesian_vel_collision_control.py` | Velocity control plus exact URDF/FCL predictive collision checks |
| `7_keyboard_cartesian_impedance_control.py` | Cartesian impedance with gravity feed-forward |
| `7_keyboard_cartesian_pd_nullspace_lock_control.py` | DLS, null-space posture lock, and joint PD |

All five scripts use a standard-library TTY listener. They do not require `pynput` or an X desktop and work in an interactive SSH terminal. Holding a key relies on terminal key repeat; about `0.18 s` after repeats stop, a release event is synthesized automatically.

Key map:

| Key | Action |
|---|---|
| `W / S` | X positive/negative |
| `A / D` | Y positive/negative; right arm mirrors in dual mode |
| `Q / E` | Z up/down |
| `1 / 2` | X rotation positive/negative; mirrored on the right |
| `3 / 4` | Y rotation positive/negative |
| `5 / 6` | Z rotation positive/negative; mirrored on the right |
| `R` | Relock the current posture; nullspace-lock example only |
| `Esc` | Exit |
| `Ctrl+C` | Force interruption |

The shared startup posture matches the mirrored VR posture:

```text
Left  [ 0.3, 0, 0, -1.87, 0, 0, 0]
Right [-0.3, 0, 0, +1.87, 0, 0, 0]
```

Collision-aware velocity control:

- 91 non-adjacent exact collision pairs in dual mode, including cross-arm pairs;
- 21 pairs for each single-arm mode;
- `0.20 s` prediction horizon with eight intermediate samples;
- stops selected arms on predicted collision or joint-limit violation;
- first collision-mesh construction takes roughly a dozen seconds;
- the OpenarmV2 model has no gripper collision geometry and measured braking distance is not yet calibrated, so low-speed supervision is still required.

### 8: Quest VR

- `8_vr_single_cartesian_dls_control.py`
- `8_vr_single_cartesian_qp_control.py`
- `8_vr_dual_cartesian_dls_control.py`
- `8_vr_dual_cartesian_qp_control.py`

This repository embeds `Quest_controller_stream/`. Its WebXR page reads left/right controller poses, thumbsticks, Trigger, Squeeze, and buttons in Quest Browser, then relays WebSocket data to robot-side UDP port `5005` by default.

The terminal UI is recommended and does not require PyQt5:

```bash
cd /home/nvidia/OpenarmHT-A1_SDK
./run_quest_stream_tui.sh
```

It supports USB/WiFi mode, HTTP(S), WS(S), UDP targets, service start/stop, packet verification, and live left/right controller pages. WiFi mode uses a self-signed OpenSSL certificate; USB mode additionally requires `adb`. The underlying stream service requires:

```bash
pip install websockets
```

The optional desktop UI additionally requires:

```bash
pip install PyQt5
python Quest_controller_stream/quest_stream_ui.py
```

Stop TUI/UI UDP Verify before starting a robot controller so both processes do not bind UDP port `5005`:

```bash
cd scripts
python 8_vr_dual_cartesian_qp_control.py
```

## Configuration and Development Entry Points

- High-level control class: `scripts/Lib/Openarm.py`
- QP IK: `scripts/Lib/qp_ik.py`
- Trajectory recorder: `scripts/Lib/recorder.py`
- SSH keyboard listener: `scripts/Lib/terminal_keyboard.py`
- Arm selection/mirroring: `scripts/Lib/keyboard_arm_utils.py`
- Arm configuration: `robot_param/Openarm_left.yaml`, `robot_param/Openarm_right.yaml`

### Configuration Layers

```text
Openarm_left/right.yaml
├── motor_param configuration path
├── joint/gripper/velocity/acceleration/torque limits
├── URDF, base, and TCP frame
├── joint names and joint_signs
└── moveL sampling parameters
        │
        ▼
robot_param/motor_param/7dof_*.yaml
├── serial type and baud rate
├── CANboard / CANport / serial_id
├── motor ID, type, and name
├── firmware position/torque limit switches
└── timeout, brake, and communication options
```

After changing the URDF or YAML, rerun left/right FK, limit, mirror, and collision validation. A parsable configuration does not prove correct physical transmission or motor direction.

### Tests and Hardware-Free Validation

V2 model regression:

```bash
conda activate panthera
python tools/test_openarm_v2_model.py
```

This test creates no low-level `Robot` and opens no serial port. It verifies the 14-DoF model, mirrored FK, Jacobians, gravity, Coriolis, mass-matrix, and dynamics shapes. Hardware arrival error, latency, braking, torque, and real collision behavior require separate acceptance testing.

### D435i Web Viewer

`tools/d435i_web_viewer` uses librealsense2 and OpenCV to capture `640×360 @ 30 FPS` RGB and depth, concatenate the views, and publish MJPEG through a loopback HTTP server.

```bash
./tools/start_d435i_web_viewer.sh
```

It listens on `127.0.0.1:8765`. Open it through VS Code port forwarding:

```text
http://127.0.0.1:8765
```

Log: `/home/nvidia/d435i_web_viewer.log`; PID file: `/tmp/d435i_web_viewer.pid`. Rebuilding requires librealsense2, OpenCV, and a C++ compiler.

### Development and Release

- `setup.py` and `pyproject.toml` describe the native-extension package;
- `CMakeLists.txt` finds Python, pybind11, the HighTorque SDK, LCM, and yaml-cpp;
- `src/bindings.cpp` is the native Python ABI entry point;
- `requirements.txt` lists high-level `scripts/Lib` dependencies, not the vendor wheel;
- do not commit `__pycache__`, build directories, temporary trajectories, runtime logs, or certificate private keys;
- before release, install the wheel on every target architecture/Python version and run import, offline-model, and supervised hardware tests.

## Troubleshooting

### `ModuleNotFoundError: matplotlib`

```bash
conda activate panthera
python -m pip install matplotlib
```

### Keyboard input does not respond

Run the example in a real interactive TTY. Do not launch it through a redirected or noninteractive background input. `pynput` is not required.

### `Mesh ../meshes/*.STL could not be found`

The collision script temporarily uses the URDF directory as the base for relative mesh paths. Verify that `OpenarmV2_URDF/meshes/` is complete.

### `/dev/ttyACM*` is missing

Check communication-board power, USB wiring, emergency-stop state, firmware mode, serial permissions, and competing serial processes.

## License

This project is licensed under the MIT License. See `LICENSE` at the repository root.
