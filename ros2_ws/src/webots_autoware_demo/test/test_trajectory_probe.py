from types import SimpleNamespace

from webots_autoware_demo.trajectory_probe import first_stop_x


def point(x, velocity):
    return SimpleNamespace(
        longitudinal_velocity_mps=velocity,
        pose=SimpleNamespace(position=SimpleNamespace(x=x)),
    )


def test_first_stop_x_returns_first_zero_velocity_point():
    trajectory = SimpleNamespace(
        points=[point(0.0, 0.25), point(0.6, 0.0), point(1.5, 0.0)]
    )
    assert first_stop_x(trajectory) == 0.6


def test_first_stop_x_returns_nan_without_stop():
    trajectory = SimpleNamespace(points=[point(0.0, 0.25), point(1.0, 0.1)])
    assert first_stop_x(trajectory) != first_stop_x(trajectory)
