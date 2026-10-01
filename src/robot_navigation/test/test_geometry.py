"""Unit tests for the pure geometry helpers (no ROS runtime required)."""
import math

from robot_navigation.geometry import clamp, normalize_angle, yaw_from_quaternion


def test_normalize_angle_wraps_into_range():
    # Result is always within (-pi, pi]; +/-pi are the same angle, and this
    # implementation yields pi for +3pi and -pi for -3pi.
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-9)
    assert math.isclose(normalize_angle(-3 * math.pi), -math.pi, abs_tol=1e-9)
    assert math.isclose(normalize_angle(0.5), 0.5, abs_tol=1e-9)
    # Any input lands in the canonical range.
    for a in (10.0, -10.0, 100.0, -0.1, 7.3):
        assert -math.pi - 1e-9 <= normalize_angle(a) <= math.pi + 1e-9


def test_normalize_angle_idempotent_in_range():
    for a in (-1.0, 0.0, 1.0, math.pi / 2):
        assert math.isclose(normalize_angle(a), a, abs_tol=1e-9)


def test_clamp():
    assert clamp(5, 0, 1) == 1
    assert clamp(-5, 0, 1) == 0
    assert clamp(0.5, 0, 1) == 0.5


def test_yaw_from_quaternion_identity():
    assert math.isclose(yaw_from_quaternion(0, 0, 0, 1), 0.0, abs_tol=1e-9)


def test_yaw_from_quaternion_90_deg():
    # 90-degree rotation about Z: (x,y,z,w) = (0,0,sin45,cos45)
    s = math.sqrt(0.5)
    assert math.isclose(yaw_from_quaternion(0, 0, s, s), math.pi / 2, abs_tol=1e-6)
