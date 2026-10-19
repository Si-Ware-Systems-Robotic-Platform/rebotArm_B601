"""Outcome checks must not report a stalled/dropped ball as a completed mission."""
import math

from rebotarm_mujoco_dm.core.slider_checks import SlideMonitor, release_correction, release_tcp
from rebotarm_mujoco_dm.bridge.scene_geometry import rotate


def test_outlet_requires_travel_around_bend():
    monitor = SlideMonitor()
    monitor.update(1., (.12, -.1085, .173))
    monitor.update(2., (.16, .10, .03))
    assert not monitor.exited
    monitor.update(3., (-.10, .03, .08))
    monitor.update(4., (.16, .10, .03))
    assert monitor.exited


def test_stalled_ball_and_side_fall_are_failures():
    stalled = SlideMonitor()
    for t in range(7):
        stalled.update(float(t), (.10, -.10, .15))
    assert 'stalled' in stalled.failure
    assert not stalled.exited
    fallen = SlideMonitor()
    fallen.update(0., (.12, -.1085, .173))
    fallen.update(1., (-.22, -.10, .02))
    assert 'outside' in fallen.failure


def test_clock_reset_and_timeout_do_not_pass():
    monitor = SlideMonitor(timeout=2.)
    monitor.update(0., (.12, -.1, .17))
    monitor.update(3., (.10, -.1, .15))
    assert 'Timed out' in monitor.failure
    reset = SlideMonitor()
    reset.update(10., (.12, -.1, .17))
    reset.update(0., (.12, -.1, .17))
    assert 'reset' in reset.failure


def test_release_uses_rotated_measured_offset():
    s = math.sqrt(.5)
    offset = rotate([0., s, 0., s], [-.014, .002, .003])
    tcp = release_tcp((.33, .0685, .1579475), offset, .015)
    assert math.dist(tcp, (.327, .0665, .1589475)) < 1e-9


def test_release_correction_cancels_measured_miss():
    expected = (.334, .0685, .1825)
    # The 3.9 mm miss seen live: shift the TCP by the opposite of the error.
    shift = release_correction((.3365, .0712, .1835), expected, .003, .006)
    assert all(math.isclose(s, v, abs_tol=1e-12) for s, v in zip(shift, (-.0025, -.0027, -.001)))
    assert release_correction((.336, .0695, .1875), expected, .003, .006) is None
    assert release_correction((.334, .0685, .1895), expected, .003, .006) is not None
