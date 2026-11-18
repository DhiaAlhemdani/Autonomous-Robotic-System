"""Constant-velocity prediction (theory 09 §3.2, the baseline that must always run)."""
import numpy as np


def predict_constant_velocity(pos, vel, horizon, dt, sigma0, sigma_rate):
    """Predict an agent's future positions and 1-sigma uncertainty.

    Returns:
        pred:  (horizon, 2) predicted positions at steps k = 1..horizon
        sigma: (horizon,)   1-sigma position uncertainty at each step

    The uncertainty grows linearly with prediction time. This is a modelling choice
    (ASSUMPTION in config), not a calibrated estimate.
    """
    pos = np.asarray(pos, dtype=float)
    vel = np.asarray(vel, dtype=float)
    k = np.arange(1, horizon + 1)
    t = k * dt
    pred = pos[None, :] + vel[None, :] * t[:, None]
    sigma = sigma0 + sigma_rate * t
    return pred, sigma
