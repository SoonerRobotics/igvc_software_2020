"""
Motion model code and simulation for kalman filter model
"""
from math import radians, degrees
import time

from sympy import sin, cos, asin, atan2, sqrt, Matrix, Symbol, eye
from sympy.abc import theta, phi, lamda, psi, p, x, y, v, a, t, r, l

from matplotlib import pyplot as plt

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
        self.state_vars = ["phi", "lamda", "theta", "x", "y", "psi", "v",  "a"]
        self.ctrl_vars = ["l", "r"]

        # Differential drive approximation
        # Velocity calculations
        v_k = 0.5 * self.wheel_rad * (l + r)
        x_dot = v_k * cos(psi)
        y_dot = v_k * sin(psi)
        psi_dot = (self.wheel_rad / self.L) * (l - r)

        # Position calculations
        x_k = x + x_dot * t
        y_k = y + y_dot * t
        psi_k = psi + psi_dot * t
        theta_k = theta + psi_dot * t

        # GPS calcs
        lat = phi + (v * t) * cos(theta) / self.R
        lon = lamda + (v * t) * sin(theta) / (self.R * cos(phi))

        f = Matrix([
            lat,
            lon,
            theta_k,
            x_k,
            y_k,
            psi_k,
            v_k,
            a
            ])

        X = Matrix([phi, lamda, theta, x, y, psi, v, a])

        # IGVC Fk matrix
        self.model = f.jacobian(X)
        self.cov_matrix = eye(8) * 0.5
        self.Q = eye(8) * 0.2

        self.csv_data = "lat, lon\n"

    def run_filter(self, last_xk, ctrl, dt):
        last_xk_mat = Matrix(last_xk[:11])

        # Input dictionary setup
        input_dict = {self.state_vars[i]: last_xk[i] for i in range(0, len(self.state_vars))}
        input_dict['t'] = dt
        ctrl_dict = {self.ctrl_vars[i]: ctrl[i] for i in range(0, len(self.ctrl_vars))}

        F = self.model.subs(input_dict).subs(ctrl_dict)
        new_state = F * last_xk_mat
        self.cov_matrix = (F * self.cov_matrix * F.T) + self.Q

        self.csv_data += str(degrees(new_state[0])) + "," + str(degrees(new_state[1])) + "\n"

        return new_state

    def get_model(self):
        return self.model

    def get_cov(self):
        return self.cov_matrix

    def print_gps(self):
        print(self.gps_lat, ", ", self.gps_lon)

    def export_data(self):
        csv_file = open("export.csv", "w")
        csv_file.write(self.csv_data)
        csv_file.close()



if __name__ == "__main__":
    motion = MotionModel()

    #radians(270.95)
    hdg = radians(180)
    psi_k = radians(0)
    # Drive forward simulation
    last_xk = [
            radians(35.210467),
            radians(-97.441811),
            hdg,
            0,
            0,
            psi_k,
            1,
            0
        ]
    ctrl_signal = [
        6.3,
        6.3
    ]

    dt = 1 / 20
    sim_time = 4
    sim_steps = round(sim_time / dt)

    points = []

    start_time = time.time()
    for _ in range(0, sim_steps):
        last_xk = motion.run_filter(last_xk, ctrl_signal, dt)
        points.append((last_xk[3], last_xk[4]))

    end_time = time.time()

    plt.scatter(*zip(*points))
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("This doesn't work")
    plt.show()

    output_dict = {motion.state_vars[i]: last_xk[i] for i in range(0, len(motion.state_vars))}
    print(output_dict)
    print("elapsed time:", end_time-start_time)
    print("cov:")
    print(motion.get_cov())

    motion.export_data()
