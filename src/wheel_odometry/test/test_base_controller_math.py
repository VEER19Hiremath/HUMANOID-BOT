"""Pure-math tests for base_controller (no ROS needed): python3 -m pytest."""
import math

from wheel_odometry.base_controller import (
    MAX_WHEEL_MPS, ease, integrate, wheel_speeds)

B = 0.4318


def test_straight():
    assert wheel_speeds(0.06, 0.0, B) == (0.06, 0.06)


def test_speed_capped_keeps_ratio():
    left, right = wheel_speeds(0.2, 0.2, B)
    assert max(abs(left), abs(right)) <= MAX_WHEEL_MPS + 1e-9
    l2, r2 = wheel_speeds(0.10, 0.10, B)
    assert math.isclose(left / right, l2 / r2, rel_tol=1e-6)


def test_turn_in_place_becomes_forward_arc():
    left, right = wheel_speeds(0.0, 0.5, B)
    assert left > 0 and right > left        # left turn, both forward


def test_never_opposite_directions():
    for v in (-0.06, -0.02, 0.0, 0.02, 0.06):
        for w in (-1.0, -0.3, 0.0, 0.3, 1.0):
            left, right = wheel_speeds(v, w, B)
            assert left * right >= 0.0, (v, w, left, right)


def test_tightest_arc_inner_wheel_keeps_speed():
    left, right = wheel_speeds(0.06, 5.0, B)
    assert left / right > 0.33           # radius ~0.46 m, inner wheel ~36 %


def test_reverse_arc():
    left, right = wheel_speeds(-0.04, 0.05, B)
    assert left < 0 and right < 0


def test_ease_limits_acceleration():
    v = 0.0
    for _ in range(10):
        v = ease(v, 0.06, 0.02)
    assert 0.0 < v < 0.06                    # 0.2 s at 0.1 m/s^2 = 0.02 m/s
    assert math.isclose(v, 0.02, rel_tol=1e-6)


def test_ease_stops_faster_than_it_starts():
    v = 0.06
    v = ease(v, 0.0, 0.1)
    assert math.isclose(v, 0.03, rel_tol=1e-6)


def test_integrate_straight_and_arc():
    x, y, t = integrate(0, 0, 0, 1.0, 1.0, B)
    assert math.isclose(x, 1.0) and abs(y) < 1e-9 and t == 0
    # Quarter circle of radius 1 m to the left, in small steps.
    x = y = t = 0.0
    r, n = 1.0, 200
    arc = math.pi / 2 * r / n
    dl, dr = arc * (r - B / 2) / r, arc * (r + B / 2) / r
    for _ in range(n):
        x, y, t = integrate(x, y, t, dl, dr, B)
    assert math.isclose(t, math.pi / 2, rel_tol=1e-6)
    assert math.isclose(x, 1.0, abs_tol=1e-6) and math.isclose(y, 1.0, abs_tol=1e-6)
