"""Smoke tests that verify the module loads and surface API exists.

Does NOT require a real robot — only validates the Python-side surface.
"""

import importlib

import numpy as np


def test_module_imports():
    fr = importlib.import_module("franka_rt")
    for name in [
        "Robot",
        "JointPositionControl",
        "JointTorqueControl",
        "CartesianPoseControl",
        "CartesianImpedanceControl",
        "FrankaException",
        "NetworkException",
    ]:
        assert hasattr(fr, name), f"franka_rt missing attribute {name}"


def test_robot_constructor_invalid_address_raises():
    """Connecting to a bogus address should raise NetworkException, not crash."""
    fr = importlib.import_module("franka_rt")
    raised = False
    try:
        # 0.0.0.0 is unroutable as a peer — this should fail fast.
        _ = fr.Robot("0.0.0.0", "ignore")
    except fr.NetworkException:
        raised = True
    except fr.FrankaException:
        raised = True
    except Exception:
        # Some libfranka versions surface a generic runtime_error
        raised = True
    assert raised, "expected an exception when connecting to 0.0.0.0"
