"""One Euro filter -- adaptive low-pass for jittery interactive signals.

Casiez, Roussel & Vogel, CHI 2012. The cutoff frequency rises with signal
speed: heavy smoothing when the signal is slow/still (kills jitter), light
smoothing when it moves fast (kills lag). Used by camera_reader.py to clean the
AprilTag pose before it reaches the robot.

Two knobs per channel:
  min_cutoff (Hz) -- lower => more smoothing of a still signal
  beta            -- higher => less lag when the signal moves fast
"""

from __future__ import annotations

import math

from quat import quat_normalize


def _alpha(dt: float, cutoff: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


class _LowPass:
    """Plain exponential low-pass with a settable per-call smoothing factor."""

    def __init__(self) -> None:
        self.y = None

    def reset(self) -> None:
        self.y = None

    def __call__(self, x: float, alpha: float) -> float:
        if self.y is None:
            self.y = x
        else:
            self.y = alpha * x + (1.0 - alpha) * self.y
        return self.y


class OneEuroFilter:
    """Scalar One Euro filter. Call once per sample with the real dt (seconds)."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.0,
                 dcutoff: float = 1.0) -> None:
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.dcutoff = float(dcutoff)
        self._x = _LowPass()
        self._dx = _LowPass()
        self._x_prev = None

    def reset(self) -> None:
        self._x.reset()
        self._dx.reset()
        self._x_prev = None

    def __call__(self, x: float, dt: float) -> float:
        x = float(x)
        if dt <= 0.0:
            dt = 1e-3
        dx = 0.0 if self._x_prev is None else (x - self._x_prev) / dt
        self._x_prev = x
        edx = self._dx(dx, _alpha(dt, self.dcutoff))
        cutoff = self.min_cutoff + self.beta * abs(edx)
        return self._x(x, _alpha(dt, cutoff))


class OneEuroPose:
    """One Euro filtering for a full pose: 3 position channels + a quaternion.

    Position and quaternion get separate (min_cutoff, beta) because their
    speeds are in different units (mm/s vs quaternion-units/s).
    """

    def __init__(self, pos_params: dict, rot_params: dict,
                 dcutoff: float = 1.0) -> None:
        pos_params = pos_params or {}
        rot_params = rot_params or {}
        pc = float(pos_params.get("min_cutoff", 1.0))
        pb = float(pos_params.get("beta", 0.05))
        rc = float(rot_params.get("min_cutoff", 1.5))
        rb = float(rot_params.get("beta", 0.20))
        self._pos = [OneEuroFilter(pc, pb, dcutoff) for _ in range(3)]
        self._quat = [OneEuroFilter(rc, rb, dcutoff) for _ in range(4)]
        self._q_prev = None        # previous raw quat, for hemisphere alignment

    def reset(self) -> None:
        for f in self._pos:
            f.reset()
        for f in self._quat:
            f.reset()
        self._q_prev = None

    def filter(self, pos, quat, dt: float):
        """Smooth (pos[x,y,z] mm, quat[w,x,y,z]); returns (list3, list4)."""
        out_pos = [self._pos[i](pos[i], dt) for i in range(3)]

        # Hemisphere-align the raw quat to the previous raw quat so the four
        # component signals stay continuous (q and -q are the same rotation).
        q = [float(v) for v in quat]
        if self._q_prev is not None:
            if sum(q[i] * self._q_prev[i] for i in range(4)) < 0.0:
                q = [-v for v in q]
        self._q_prev = q
        sq = [self._quat[i](q[i], dt) for i in range(4)]
        return out_pos, list(quat_normalize(sq))
