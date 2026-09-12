"""Preset reference for the deployment trials: a steady westbound approach leg.

Starts at the SPAWN (-329.5, 221), 130 m east of the canonical start
(-459.5, 221), drives due west (reference psi = pi, y constant) so the vessel
reaches x = -459.5 at y = 221, not 5.4 m north of it as a 3.10 rad reference
would give. The spawn yaw stays 3.10 rad (spawn_canonical_start.yaml): that
keeps the node's continuous yaw at +3.10 rather than -3.14, and the 0.04 rad
initial heading error to the pi reference is harmless. Ramps to cruise speed
over 10 s and holds it. Long enough (500 s) to cover the healthy logging
window (~170 s), the fault trigger, and the manifold timeout while the sidecar
computes, so the node never runs off the end of the file. Safety hold at
x = -520 (48 m east of the dock fronts at x = -568).

8 columns at 20 Hz: [x, y, psi, u, v, r, Tp, Ts]; Tp, Ts left at 0 (the MPC
decides, W_u is small), as the node's own station-keeping generator does.
"""
import numpy as np
import argparse

p = argparse.ArgumentParser()
p.add_argument("--out", default="approach_leg_0p8.txt")
p.add_argument("--speed", type=float, default=0.8)
p.add_argument("--duration", type=float, default=500.0)
p.add_argument("--x0", type=float, default=-329.5, help="spawn x")
p.add_argument("--y0", type=float, default=221.0)
p.add_argument("--psi", type=float, default=np.pi, help="reference heading (rad); pi = due west")
p.add_argument("--x-stop", type=float, default=-520.0,
               help="hold position once x reaches this (safety: 60 m east of the opening)")
a = p.parse_args()

dt = 0.05
t = np.arange(0.0, a.duration + dt, dt)
ramp = 10.0
u = np.where(t < ramp, a.speed * 0.5 * (1 - np.cos(np.pi * t / ramp)), a.speed)
s = np.concatenate([[0.0], np.cumsum(0.5 * (u[1:] + u[:-1]) * dt)])
x = a.x0 + s * np.cos(a.psi)
y = a.y0 + s * np.sin(a.psi)   # zero drift for psi = pi (sin(pi) ~ 1e-16)
# safety stop: hold the pose where x first reaches x_stop (never drive into the quay)
stop = np.flatnonzero(x <= a.x_stop)
if len(stop):
    k = stop[0]
    x[k:], y[k:], u[k:] = x[k], y[k], 0.0
traj = np.zeros((len(t), 8))
traj[:, 0], traj[:, 1], traj[:, 2], traj[:, 3] = x, y, a.psi, u
np.savetxt(a.out, traj, fmt="%.6f")
k459 = np.argmax(x <= -459.5)
print(f"{a.out}: {len(t)} rows, canonical start x=-459.5 reached at t={t[k459]:.0f} s (y={y[k459]:.2f}), "
      f"holds at ({x[-1]:.1f}, {y[-1]:.1f}) from t={t[stop[0]] if len(stop) else a.duration:.0f} s")
