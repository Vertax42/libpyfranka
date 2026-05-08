"""franka_rt v0.1.0 — Python real-time control bindings for Franka FR3 via libfranka.

Architecture
------------
The 1 kHz `robot.control(callback)` loop runs in a C++ background RT thread;
libfranka owns the timing and `limit_rate=true` filtering pipeline, we only
fill in the callback.  Python writes sparse targets at any rate (30–200 Hz)
into an SPSC ring buffer; the C++ callback runs a continuous stateful online
trajectory generator (PD + jerk-limited integrator) that follows those
targets at 1 kHz with no segment-boundary discontinuities.

This decouples the RT loop from the Python GIL/GC, so it stays safe in
lerobot-style environments where the main Python loop also runs cameras,
tactile inference, and policy code.

API names mirror the official `pylibfranka` Python binding so a single-line
`import` change is enough to switch back and forth — see
https://frankarobotics.github.io/libfranka/pylibfranka/latest/.

Exports
-------
    Robot                — main connection to the FCI
    RealtimeConfig       — kEnforce / kIgnore (mirrors pylibfranka)
    ControllerMode       — JointImpedance / CartesianImpedance
    RobotMode            — Idle / Move / Reflex / UserStopped / ...
    JointPositionControl — 1 kHz joint-position streaming + moveToJoints
    CartesianControl     — 1 kHz Cartesian-pose streaming + moveToPose
    FrankaException, ControlException, CommandException, NetworkException
"""

from ._franka_rt import (
    Robot,
    RobotMode,
    RealtimeConfig,
    ControllerMode,
    JointPositionControl,
    CartesianControl,
    FrankaException,
    ControlException,
    CommandException,
    NetworkException,
)

__version__ = "0.1.0"

__all__ = [
    "Robot",
    "RobotMode",
    "RealtimeConfig",
    "ControllerMode",
    "JointPositionControl",
    "CartesianControl",
    "FrankaException",
    "ControlException",
    "CommandException",
    "NetworkException",
]
