"""Smoke tests that verify the module loads and the Python surface API exists.

Does NOT require a real robot — only validates the Python-side bindings.
"""

import importlib


def test_module_imports():
    fr = importlib.import_module("franka_rt")
    for name in [
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
    ]:
        assert hasattr(fr, name), f"franka_rt missing attribute {name}"


def test_controller_mode_enum_values():
    fr = importlib.import_module("franka_rt")
    assert hasattr(fr.ControllerMode, "JointImpedance")
    assert hasattr(fr.ControllerMode, "CartesianImpedance")


def test_robot_mode_enum_values():
    fr = importlib.import_module("franka_rt")
    for name in ["Idle", "Move", "Reflex", "UserStopped"]:
        assert hasattr(fr.RobotMode, name), f"RobotMode missing {name}"


def test_robot_constructor_invalid_address_raises():
    """Connecting to a bogus address must raise, not crash."""
    fr = importlib.import_module("franka_rt")
    raised = False
    try:
        _ = fr.Robot("0.0.0.0", fr.RealtimeConfig.kIgnore)
    except (fr.NetworkException, fr.FrankaException, Exception):
        raised = True
    assert raised, "expected an exception when connecting to 0.0.0.0"
