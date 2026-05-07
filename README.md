# libpyfranka

Python real-time control bindings for **Franka FR3** via the official
[libfranka](https://github.com/frankaemika/libfranka) C++ SDK.

Sister project to [libpyflexiv](../libpyflexiv) — same architecture, different
robot.

## Why This Project Exists

Franka's `libfranka` requires a 1 kHz hard real-time loop. CPython cannot meet
that deadline (GIL, GC, interpreter overhead). This package puts the 1 kHz loop
in a C++ `SCHED_FIFO` thread that talks to libfranka's async control API
(`startXxxControl()` / `readOnce()` / `writeOnce()`), and exposes a streaming
target interface to Python via mutex-protected shared state. Python writes
targets at whatever rate it wants (10–200 Hz typical for VLA / policy);
the C++ thread interpolates to 1 kHz.

## Controllers

| Controller | Streaming target | libfranka API |
|---|---|---|
| `JointPositionControl` | 7D joint positions | `startJointPositionControl(kJointImpedance)` |
| `JointTorqueControl` | 7D torques | `startTorqueControl()` |
| `CartesianPoseControl` | 4×4 pose matrix | `startCartesianPoseControl(kCartesianImpedance)` |
| `CartesianImpedanceControl` | 4×4 pose + 6D K/D | `startTorqueControl()` (computes τ via Jacobian) |

## Build & Install

### Step 1: Build libfranka

```bash
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka && git checkout 0.21.2
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX=$HOME/franka_install ..
cmake --build . -j$(nproc)
cmake --install .
```

### Step 2: Build & install libpyfranka

```bash
git clone --recursive https://github.com/<user>/libpyfranka.git
cd libpyfranka
mkdir build && cd build
CMAKE_PREFIX_PATH=$HOME/franka_install cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(nproc)
cd .. && pip install -e .
```

### Step 3: Run (needs `SCHED_FIFO` privileges)

Either run with `sudo -E`, or set up PAM rt limits (preferred for production):

```
# /etc/security/limits.conf
@realtime    -    rtprio    99
@realtime    -    memlock   unlimited
```

## Quick Start

### Joint position streaming

```python
import time, numpy as np, franka_rt as fr

with fr.Robot("172.16.0.2") as robot:
    with robot.start_joint_position_control() as jc:
        q0 = jc.get_state().q
        for t in np.arange(0, 5, 0.01):
            target = q0.copy()
            target[3] += 0.05 * np.sin(2 * np.pi * 0.3 * t)
            jc.set_target_joints(target)
            time.sleep(0.01)
```

### Cartesian impedance

```python
with robot.start_cartesian_impedance_control() as cic:
    cic.set_stiffness([300, 300, 300, 30, 30, 30])
    pose = cic.get_state().O_T_EE.copy()
    pose[1, 3] += 0.05  # +5 cm in Y
    cic.set_target_pose(pose)
    time.sleep(2.0)
```

## API Reference

(See docstrings in `franka_rt/__init__.py` and the C++ header files under
`include/realtime_control/`.)

## Safety Features

- Per-cycle joint and Cartesian velocity / acceleration clamping
- NaN / Inf check on every command
- Adaptive jump detection (catches large discontinuities from upstream)
- libfranka's built-in `limitRate()` and `cartesianLowpassFilter()` applied
  before `writeOnce()`
- E-stop: `controller.trigger_estop()` cleanly aborts the RT loop

## Project Structure

```
include/realtime_control/    C++ public headers
src/                         pybind11 bindings + RT loop implementations
franka_rt/                   Python package
examples/                    Usage examples (one per controller + reset)
tests/cpp/                   GTest unit tests (no robot needed)
tests/python/                pytest sim + integration tests
libfranka/                   Vendored libfranka submodule
```

## Tests

```bash
# C++ unit tests (no robot)
cmake -DBUILD_TESTS=ON .. && cmake --build . && ctest

# Python sim tests (no robot)
pytest tests/python/test_interpolation_sim.py

# Integration tests (requires real FR3)
sudo -E pytest tests/python/test_rt_integration.py --robot-ip=172.16.0.2
```

## License

Apache-2.0 (matching libfranka).
