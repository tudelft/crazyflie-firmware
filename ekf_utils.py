"""
Generic Extended Kalman Filter (EKF) with numerical Jacobians.

Supports discrete or continuous dynamics (RK4 discretisation).
Cleaned-up version of the user-provided EKF class.
"""

from __future__ import annotations

import copy
import warnings

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Utility functions
# ---------------------------------------------------------------------------

def wrapToPi(rad):
    """Wrap angle(s) to [-π, π)."""
    rad = np.asarray(rad, dtype=float)
    return (rad + np.pi) % (2 * np.pi) - np.pi


def wrapTo2Pi(rad):
    """Wrap angle(s) to [0, 2π)."""
    rad = np.asarray(rad, dtype=float)
    return rad % (2 * np.pi)


def angle_difference(target, current):
    """Signed minimal angular difference (radians), in [-π, π)."""
    return wrapToPi(target - current)


def jacobian_numerical(f, x, u, epsilon=1e-6):
    """
    Central-difference numerical Jacobian  ∂f/∂x  evaluated at (x, u).
    """
    x = np.asarray(x, dtype=np.float64)
    u = np.asarray(u, dtype=np.float64)

    n = len(x)
    f0 = np.atleast_1d(f(x, u))
    m = len(f0)

    J = np.zeros((m, n))
    for i in range(n):
        x_plus  = x.copy();  x_plus[i]  += epsilon
        x_minus = x.copy();  x_minus[i] -= epsilon
        J[:, i] = (np.atleast_1d(f(x_plus, u))
                    - np.atleast_1d(f(x_minus, u))) / (2.0 * epsilon)
    return J


def rk4_discretize(f, x, u, dt):
    """RK4 integration:  x_{k+1} = x_k + (dt/6)(k1 + 2k2 + 2k3 + k4)."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


# ---------------------------------------------------------------------------
# EKF class
# ---------------------------------------------------------------------------

class EKF:
    """
    Extended Kalman Filter with Joseph-form covariance update.

    Parameters
    ----------
    f : callable
        State transition  xdot = f(x, u)  (continuous) or  x_{k+1} = f(x_k, u_k)  (discrete).
    h : callable
        Measurement function  z = h(x, u).
    x0, u0 : array-like
        Initial state and input vectors.
    P0 : ndarray
        Initial state error covariance.
    Q, R : ndarray
        Process / measurement noise covariance.
    circular_measurements : iterable of bool, optional
        Which measurement indices are angular (wrapped to [-π, π)).
    dynamics_type : {'discrete', 'continuous'}
        Whether *f* returns  x_{k+1}  or  dx/dt.
    discretization_timestep : float, optional
        Default dt for RK4 (required when dynamics_type='continuous').
    """

    def __init__(self, f, h, x0, u0, P0, Q, R,
                 circular_measurements=None,
                 dynamics_type='discrete',
                 discretization_timestep=None):

        self.f = f
        self.h = h
        self.dynamics_type = dynamics_type

        if dynamics_type == 'continuous':
            if discretization_timestep is None:
                raise ValueError("discretization_timestep is required for continuous dynamics")
            self.discretization_timestep = discretization_timestep
        else:
            self.discretization_timestep = None

        # Sizes
        self.x0 = np.atleast_1d(np.array(x0, dtype=float))
        self.u0 = np.atleast_1d(np.array(u0, dtype=float))

        # Validate by running f and h once
        self.x0 = np.atleast_1d(self.f_discrete(self.x0, self.u0))
        self.z0 = np.atleast_1d(self.h(self.x0, self.u0))

        self.n = self.x0.shape[0]   # states
        self.p = self.z0.shape[0]   # measurements
        self.c = self.u0.shape[0]   # controls

        # Circular measurement flags
        if circular_measurements is None:
            self.circular_measurements = tuple(np.zeros(self.p, dtype=bool))
        else:
            self.circular_measurements = tuple(circular_measurements)

        # Matrices
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.F = None
        self.H = None
        self.S = None
        self.K = None

        # History
        self.history = {
            'X': [self.x0.copy()],
            'U': [self.u0.copy()],
            'Z': [self.z0.copy()],
            'P': [self.P.copy()],
            'P_diags': [np.diag(self.P)],
        }

        # Current state
        self.x = self.x0.copy()
        self.u = self.u0.copy()
        self.z = self.z0.copy()
        self.k = 0

    # ------------------------------------------------------------------
    # Discretisation helper
    # ------------------------------------------------------------------

    def f_discrete(self, x, u):
        if self.dynamics_type == 'continuous':
            return rk4_discretize(self.f, x, u, self.discretization_timestep)
        else:
            return np.atleast_1d(self.f(x, u))

    # ------------------------------------------------------------------
    # Predict / Update
    # ------------------------------------------------------------------

    def _predict(self, u, Q=None, discretization_timestep=None):
        if discretization_timestep is not None and self.dynamics_type == 'continuous':
            self.discretization_timestep = discretization_timestep

        self.u = np.atleast_1d(np.array(u, dtype=float))
        if Q is not None:
            self.Q = np.array(Q, dtype=float)

        self.x = self.f_discrete(self.x, self.u)
        self.F = jacobian_numerical(self.f_discrete, self.x, self.u)
        self.P = self.F @ self.P @ self.F.T + self.Q

    def _update(self, z, R=None):
        self.z = np.atleast_1d(np.array(z, dtype=float))
        if R is not None:
            self.R = np.array(R, dtype=float)

        z_pred = np.atleast_1d(self.h(self.x, self.u))

        # Innovation (with angular wrapping where needed)
        y = np.zeros(self.p)
        for j in range(self.p):
            if self.circular_measurements[j]:
                y[j] = float(wrapToPi(self.z[j] - z_pred[j]))
            else:
                y[j] = self.z[j] - z_pred[j]

        self.H = jacobian_numerical(self.h, self.x, self.u)
        self.S = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(self.S)

        self.x = self.x + self.K @ y

        # Joseph form
        I = np.eye(self.n)
        ImKH = I - self.K @ self.H
        self.P = ImKH @ self.P @ ImKH.T + self.K @ self.R @ self.K.T

        # Record history
        self.history['X'].append(self.x.copy())
        self.history['U'].append(self.u.copy())
        self.history['Z'].append(self.z.copy())
        self.history['P'].append(self.P.copy())
        self.history['P_diags'].append(np.diag(self.P))

        self.k += 1

    def forward_update(self, z, u, Q=None, R=None, discretization_timestep=None):
        """Combined predict + update."""
        self._predict(u, Q=Q, discretization_timestep=discretization_timestep)
        self._update(z, R=R)
