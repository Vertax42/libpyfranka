# libpyfranka

Python real-time control bindings for **Franka FR3** via the official
[libfranka](https://github.com/frankaemika/libfranka) C++ SDK.

The 1 kHz `robot.control()` loop runs in a C++ background thread; Python
streams sparse targets at any rate and a continuous online trajectory
generator (OTG) integrates them to a smooth 1 kHz pose/joint stream that
libfranka accepts without firing motion-discontinuity reflexes.

API names mirror the official
[`pylibfranka`](https://frankarobotics.github.io/libfranka/pylibfranka/latest/)
binding so a one-line `import` change is enough to switch back and forth.

## Why not just use pylibfranka

Both expose libfranka. The difference is **where the 1 kHz loop runs**:

| Path | 1 kHz loop runs in | Safe with cameras / GPU policies in the same process? |
|---|---|---|
| `pylibfranka` | Python main thread | No — GIL/GC/cuda-alloc jitter exceeds 1 ms → reflex |
| `libpyfranka` (this) | C++ `SCHED_FIFO` thread | Yes |

If you're driving the FR3 from a process that also runs camera capture,
tactile inference, or VLA policy code, you want the C++ thread. If your
Python process just talks to the robot, `pylibfranka` is simpler.

## Controllers

| Class | Streaming target | Internal libfranka controller |
|---|---|---|
| `JointPositionControl` | 7D joint positions | `kJointImpedance` |
| `CartesianControl(mode)` | 4×4 EE pose | `kCartesianImpedance` (default) or `kJointImpedance` |

Each controller exposes:

- `set_target_*(...)` — non-blocking push into an SPSC ring buffer
- `get_state()` — non-blocking read of the latest robot state (releases the GIL,
  RT side never blocks on the Python read)
- `move_to_*(target, duration_sec=0)` — non-blocking on-RT min-jerk segment;
  `duration_sec=0` auto-computes a safe duration; `is_moving()` to poll
- `trigger_estop()`, `stop()`, context-manager support

## Architecture

```
Python (any rate, 30–200 Hz typical)
   ↓  set_target_* → SPSC ring buffer (lock-free)
─────────────────────────────────────────────────────────
C++ RT thread (SCHED_FIFO):
   robot.control(callback, mode, limit_rate=true) {
     callback(state, period) {
       1. snapshot state under try_lock (RT never blocks on Python)
       2. drain ring buffer → latest target
       3. continuous OTG step:
            desired_acc = Kp·(target − pos) − Kd·vel
            acc += clamp(desired_acc − acc, ±Jmax·dt)
            vel += acc·dt;  vel = clamp(vel, ±Vmax)
            pos += vel·dt
       4. return new pose / joints to libfranka
     }
   }
─────────────────────────────────────────────────────────
libfranka: limit_rate=true filter → motors at 1 kHz
```

The OTG state (`vel`, `acc`) is **continuous across all 1 kHz cycles** — there
is no segment boundary, no re-init.  This is the design that survived a
12-iteration debug arc against `joint_motion_generator_*_discontinuity` and
`cartesian_motion_generator_*_discontinuity` errors that segment-based
interpolation kept producing.  See the comment block at the top of
`src/cartesian_control.cpp::rtLoop` for the full reasoning.

## Build & install

### 1. Build libfranka

```bash
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka && git checkout 0.21.x
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/franka_install
cmake --build build -j$(nproc) --target install
```

### 2. Build & install libpyfranka

```bash
conda activate xensehand
cd libpyfranka
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=$HOME/franka_install \
      -Dfmt_DIR=$CONDA_PREFIX/lib/cmake/fmt \
      -DPython3_EXECUTABLE=$(which python) \
      -DBUILD_TESTS=ON
cmake --build build -j$(nproc)
pip install -e .
```

Build and run from the same Python environment.  The extension filename embeds
the CPython ABI, e.g. `_franka_rt.cpython-310-...so`; importing it from Python
3.13 will fail even if the C++ build succeeded.

`fmt_DIR` is required when libfranka was built against the system fmt but you
run from a conda env.  Both libraries must agree on the `libfmt.so.X` ABI; the
build script validates this and fails early on mismatch.

### 3. RT privileges

Either run with `sudo -E`, or grant your user `SCHED_FIFO`:

```
# /etc/security/limits.conf
@realtime    -    rtprio    99
@realtime    -    memlock   unlimited
```

## Quick start

### Robot setup before control

Call NRT configuration methods **before** `start_*_control()`.  The constructor
already installs conservative defaults, but teleop deployments should make the
tool/load and stiffness explicit:

```python
import franka_rt as fr

robot = fr.Robot(
    "192.168.99.111",
    fr.RealtimeConfig.kIgnore,
    load_mass=0.35,                    # kg, 0.0 if no payload
    load_com=[0.0, 0.0, 0.06],          # payload COM in flange frame [m]
)

# Cartesian stiffness: [x, y, z, roll, pitch, yaw]
# translation in N/m, rotation in Nm/rad.
robot.set_cartesian_impedance([800, 800, 800, 80, 80, 80])

# Joint stiffness, one value per joint, Nm/rad.  libfranka exposes stiffness K
# only; damping is handled internally by Franka's impedance controller.
robot.set_joint_impedance([1500, 1500, 1500, 1200, 1000, 800, 600])
```

`set_cartesian_impedance`, `set_joint_impedance`, `set_collision_behavior`,
`set_load`, `set_EE`, and `set_K` are NRT commands.  Do not call them from
inside the 1 kHz control loop.

### Cartesian-pose streaming

```python
import time, numpy as np, franka_rt as fr

with fr.Robot("192.168.99.111", fr.RealtimeConfig.kIgnore) as robot:
    with robot.start_cartesian_pose_control(fr.ControllerMode.CartesianImpedance) as cc:
        T0 = cc.get_state()["O_T_EE"].copy()       # 4×4 numpy
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0:
            T = T0.copy()
            T[0, 3] += 0.05 * np.sin(2 * np.pi * 0.2 * (time.monotonic() - t0))
            cc.set_target_pose(T)
            time.sleep(1/30)                        # 30 Hz writer is fine
```

### Episode reset without leaving RT mode

`move_to_pose()` and `move_to_joints()` run a min-jerk trajectory inside the
C++ control thread and return immediately.  This matches the `libpyflexiv`
pattern: cache an episode start pose, skip teleop writes while `is_moving()`,
then resume streaming after the reset finishes.

```python
with fr.Robot("192.168.99.111", fr.RealtimeConfig.kIgnore) as robot:
    with robot.start_cartesian_pose_control(fr.ControllerMode.CartesianImpedance) as cc:
        initial_T = cc.get_state()["O_T_EE_c"].copy()

        # ... teleop streaming ...

        cc.move_to_pose(initial_T, duration_sec=3.0)
        while cc.is_moving():
            obs = cc.get_state()     # still safe during the RT reset
            time.sleep(0.02)

        # Resume set_target_pose(...) after the reset.
```

### Joint-position streaming

```python
with fr.Robot("192.168.99.111", fr.RealtimeConfig.kIgnore) as robot:
    with robot.start_joint_position_control() as jc:
        q0 = jc.get_state()["q"]
        target = list(q0)
        target[3] += 0.05                           # +0.05 rad on joint 3
        jc.move_to_joints(target, duration_sec=0)   # auto-pick safe duration
        while jc.is_moving():
            time.sleep(0.05)
```

### Reading state at any rate

`get_state()` releases the GIL and uses `try_lock()` on the RT side, so calling
it from a Python thread cannot stall the 1 kHz loop:

```python
state = cc.get_state()
state["q"]                    # measured joint pos
state["O_T_EE"]               # measured EE pose (4×4)
state["O_F_ext_hat_K"]        # external wrench, base frame
state["has_error"]            # bool
state["control_command_success_rate"]
```

Full state dict fields are listed in `src/franka_rt.cpp::getState`.

## Impedance Modes

`CartesianControl` can stream the same 4×4 target pose through either built-in
libfranka controller:

```python
robot.start_cartesian_pose_control(fr.ControllerMode.CartesianImpedance)
robot.start_cartesian_pose_control(fr.ControllerMode.JointImpedance)
```

Use `CartesianImpedance` for teleoperation and contact-rich tasks.  It tracks
the target in Cartesian space with compliance set by
`robot.set_cartesian_impedance([Kx, Ky, Kz, Kr, Kp, Kyaw])`.

Use `JointImpedance` when you want stiffer pose tracking through libfranka's IK
and joint impedance controller.  Its stiffness comes from
`robot.set_joint_impedance([K1, ..., K7])`.

libfranka does **not** expose user-settable damping arrays for these internal
controllers.  Our internal `kKp/kKd/kMaxVel/kMaxAcc/kMaxJerk` constants are
only target-smoothing parameters; they are not robot impedance gains.

## Two RT-design rules to remember

If you extend the controllers, two things kept biting us during development:

1. **Use `period.toSec()` directly as `dt`, never clamp.**  If you clamp `dt`
   when a 3 ms scheduler-jitter cycle hits, your OTG integrates as if 2 ms
   passed, but libfranka samples velocity over the real 3 ms — it sees a
   33 % velocity step at the sample boundary and fires the discontinuity
   check.  Honest `dt` keeps OTG and libfranka in sync.

2. **Use `try_lock()`, never `lock_guard`, on the RT side.**  A real lock
   means the 1 kHz callback can be stalled by a slow Python `get_state()`
   (GIL/GC).  The next callback then comes in 2–5 ms late, libfranka sees
   the gap, discontinuity reflex fires.  `try_lock` + skip-this-cycle is
   safe; missing one snapshot at 1 kHz is invisible to consumers.

## Tests

```bash
# C++ unit tests (no robot)
ctest --test-dir build --output-on-failure

# Python smoke tests (no robot)
python -m pytest tests/python/test_smoke.py

# Interactive on-robot tests (graduated phases, prompts before each motion)
python tests/python/test_cartesian_robot_interactive.py --ip 192.168.99.111
python tests/python/test_joint_robot_interactive.py     --ip 192.168.99.111

# Ground-truth official-libfranka A/B baseline (callback path, no franka_rt)
cmake -S tests/robot -B build-libfranka-diagnostics \
      -DCMAKE_PREFIX_PATH=$HOME/franka_install -Dfmt_DIR=$CONDA_PREFIX/lib/cmake/fmt
cmake --build build-libfranka-diagnostics
./build-libfranka-diagnostics/libfranka_cartesian_sine --ip 192.168.99.111
```

## Project layout

```
include/realtime_control/   C++ public headers (controllers + RT helpers)
src/                        pybind11 bindings + RT loop implementations
franka_rt/                  Python package
tests/cpp/                  GTest unit tests (no robot)
tests/python/               pytest smoke tests + interactive on-robot tests
tests/robot/                Official-libfranka baseline diagnostic
```

## Troubleshooting

### `ImportError: libfmt.so.*: cannot open shared object file`

Mismatched `fmt` ABI between libfranka and the active runtime env.  Inspect:

```bash
readelf -d $HOME/franka_install/lib/libfranka.so.0.21 | grep NEEDED
ldd franka_rt/_franka_rt*.so | grep fmt
```

Fix by passing `-Dfmt_DIR=…` to the libpyfranka configure step pointing at
the same `fmt` libfranka was linked against.  CMake validates this during
configure and fails early on mismatch.

### `cartesian_motion_generator_*_discontinuity` / `joint_motion_generator_*_discontinuity` reflex

If you see these on streaming tests, check the two RT-design rules above —
they are exactly the failure modes those rules guard against.  The interactive
test scripts log the last 5 cycle periods on failure, which usually reveals
a > 1 ms scheduler jitter spike.

## License

Apache-2.0 (matching libfranka).
