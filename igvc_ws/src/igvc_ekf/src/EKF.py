import numpy as np

from math import radians, degrees
import time

from sympy import sin, cos, asin, atan2, sqrt, Matrix, Symbol, Identity
from sympy.abc import theta, phi, lamda, psi, p, x, y, v, a, t, r, l

class EKF:

    def __init__(self, x_o, P_o):
        # Initialize state and covariance
        self.x_k = x_o
        self.P_k = P_o

        # Initialize motion model
        self.motion = MotionModel()

    def run_filter(self, sensor_data):
        self.x_k, self.P_k = self.motion.run_model(self.x_k, self.P_k)



class MotionModel:
    """
    SCR IGVC Motion Model
    """

    # Constants
    L = 0.381           # Wheelbase length (meters)
    wheel_rad = 0.127   # Wheel radius (meters)
    R = 6378137         # Earth radius (meters)

    def __init__(self):
        x = Symbol("x")

        # State variables
        self.state_vars = ["phi", "lamda", "theta", "x", "y", "psi", "v", "p", "l", "r", "a"]

        # Differential drive approximation
        x_dot = 0.5 * self.wheel_rad * (l + r) * cos(psi)
        y_dot = 0.5 * self.wheel_rad * (l + r) * sin(psi)
        psi_dot = (self.wheel_rad / self.L) * (r - l)
        x_k = x + x_dot * t
        y_k = y + y_dot * t
        psi_k = psi + psi_dot * t
        theta_k = theta + psi_dot * t

        # Velocity calculation
        v_k = 0.5 * self.wheel_rad * (l + r) #sqrt(x_dot**2 + y_dot**2)

        # GPS calcs
        lat = phi + (v * t) * cos(theta) / self.R
        lon = lamda + (v * t) * sin(theta) / (self.R * cos(phi))

        f = Matrix(
            [lat,
             lon,
             theta_k,
             x_k,
             y_k,
             psi_k,
             v_k,
             psi_dot,
             l,
             r,
             a
            ])

        X = Matrix([phi, lamda, theta, x, y, psi, v, p, l, r, a])

        # IGVC Fk matrix
        self.Fk = f.jacobian(X)

        self.Q = Identity(11, 11) * 0.2

        self.csv_data = "lat, lon\n"

    def run_model(self, last_xk, last_Pk, dt = 0.05):
        last_xk_mat = Matrix(last_xk[:11])

        # Input dictionary setup
        input_dict = {self.state_vars[i]: last_xk[i] for i in range(0, len(self.state_vars))}
        input_dict['t'] = dt

        self.state_estimate = self.Fk.subs(input_dict) * last_xk_mat

        self.csv_data += str(degrees(self.state_estimate [0])) + "," + str(degrees(self.state_estimate [1])) + "\n"

        self.P = self.Fk * last_Pk * self.Fk.T + self.Q

        return (self.state_estimate, self.P)

    def export_data(self):
        csv_file = open("export.csv", "w")
        csv_file.write(self.csv_data)
        csv_file.close()