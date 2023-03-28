
import numpy as np

def get_dynamic_model_from_type(dynamic_model_type: str, sigmas: list):
    if dynamic_model_type == "constant_velocity_no_attitude":
        dynamic_model = ConstantVelocityModel
    else:
        print(f"Dynamic model type not supported: {dynamic_model_type}")
        raise NotImplementedError

    return dynamic_model(sigmas)

class ConstantVelocityModel():

    n = 6 # 3 pos, 3 vel

    def __init__(self, sigmas):
        self._Q = np.diag(sigmas)**2

    def f(self, x: np.ndarray, u: np.ndarray, dt: float):
        """
        Calculate zero-noise transition from current x

        x = [helipad position in drone body frame (x,y,z),
             helipad velocity in drone body frame (v_x, v_y, v_z)]

        u = not used
        """

        x_next = np.zeros_like(x, dtype=float)

        # Position prediction
        x_next[0] = x[0] - dt*x[3]
        x_next[1] = x[1] - dt*x[4]
        x_next[2] = x[2] - dt*x[5]

        # Velocity prediction
        x_next[3] = x[3]
        x_next[4] = x[4]
        x_next[5] = x[5]

        return x_next.copy()

    def F(self, x, u, dt):
        """
        Calculate transition function Jacobian
        """

        F_pos = np.array([
            [1, 0, 0, -dt, 0, 0],
            [0, 1, 0, 0, -dt, 0],
            [0, 0, 1, 0, 0, -dt]
        ])

        F_vel = np.array([
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        F = np.vstack((F_pos, F_vel))

        return F.copy()

    def Q(self, x, dt):
        return self._Q.copy()