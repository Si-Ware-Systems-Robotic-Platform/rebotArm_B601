"""Measured slider progress, shared by the ROS mission and offline physics check."""
import math


class SlideMonitor:
    """Track a ball around the curved CAD track and across its outlet in local metres."""

    def __init__(self, timeout=30.0, stall_time=5.0):
        self.timeout = timeout
        self.stall_time = stall_time
        self.samples = []
        self.start = None
        self.last_stamp = None
        self.visited_bend = False
        self.exited = False
        self.failure = None

    def update(self, stamp, local_position):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp:
                self.failure = "Simulation clock reset during slide observation"
            return
        if not all(math.isfinite(v) for v in local_position):
            self.failure = "Non-finite ball position"
            return
        self.last_stamp = stamp
        if self.start is None:
            self.start = stamp
        if self.exited or self.failure:
            return
        x, y, z = local_position
        self.visited_bend |= x < -0.03
        # All CAD outlets face +X, on the +Y side of the slider.
        if self.visited_bend and x >= 0.15 and 0.04 <= y <= 0.19 and 0.015 <= z <= 0.09:
            self.exited = True
            return
        if z < 0.005 or x < -0.21 or y < -0.20 or y > 0.22 or x > 0.20:
            self.failure = f"Ball left the track outside its outlet: {local_position}"
            return
        self.samples.append((stamp, tuple(local_position)))
        while len(self.samples) > 1 and self.samples[1][0] < stamp - self.stall_time:
            self.samples.pop(0)
        if stamp - self.samples[0][0] >= self.stall_time:
            anchor = self.samples[0][1]
            if max(math.dist(anchor, p) for _, p in self.samples) < 0.003:
                self.failure = f"Ball stalled on slider at {local_position}"
        if stamp - self.start >= self.timeout:
            self.failure = "Timed out before measured passage through the slider outlet"


def release_correction(actual, expected, tolerance, vertical_tolerance):
    """TCP shift cancelling the measured ball miss, or None once inside tolerance.

    Rail selection depends on X/Y; Z only changes the drop height above the groove."""
    error = [a - e for a, e in zip(actual, expected)]
    if math.hypot(error[0], error[1]) <= tolerance and abs(error[2]) <= vertical_tolerance:
        return None
    return tuple(-e for e in error)


def release_tcp(target_center, offset_world, clearance):
    """Keep a deliberate vertical release gap above the CAD ball-centre marker."""
    return tuple(target_center[i] - offset_world[i] + (clearance if i == 2 else 0)
                 for i in range(3))
