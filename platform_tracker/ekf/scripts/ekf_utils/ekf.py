
import scipy.linalg as la
import numpy as np
from measurement_models.measurement_models import MeasurementModel

class EKFState():

    def __init__(self, mean, cov):

        self.mean = mean
        self.cov = cov

class EKF():

    def __init__(self, dynamic_model, measurement_models: dict):
        self.dynamic_model = dynamic_model
        self.measurement_models = measurement_models

    def predict(self, ekfstate: EKFState, u: np.ndarray, dt: float):
        """Predict the EKF state dt seconds ahead."""
        x = ekfstate.mean
        P = ekfstate.cov

        F = self.dynamic_model.F(x, u, dt)
        Q = self.dynamic_model.Q(x, dt)

        x_pred = self.dynamic_model.f(x, u, dt)

        P_pred = F @ P @ F.T + Q

        assert np.all(np.isfinite(P_pred)) and np.all(
            np.isfinite(x_pred)
        ), "Non-finite EKF prediction."

        state_pred = EKFState(x_pred, P_pred)

        return state_pred

    def innovation_mean(self, z, ekfstate, measurement_model: MeasurementModel):
        """Calculate the innovation mean for ekfstate at z."""

        x = ekfstate.mean

        zbar = measurement_model.h(x)

        v = z - zbar

        return v

    def innovation_cov(self, ekfstate, measurement_model: MeasurementModel):
        """Calculate the innovation covariance for ekfstate at z."""

        x = ekfstate.mean
        P = ekfstate.cov

        H = measurement_model.H()
        R = measurement_model.R()
        S = H @ P @ H.T + R

        return S

    def innovation(self, z, ekfstate, measurement_model):
        """Calculate the innovation for ekfstate at z in sensor_state."""

        v = self.innovation_mean(z, ekfstate, measurement_model)
        S = self.innovation_cov(ekfstate, measurement_model)

        return v, S

    def update(self, z: np.ndarray, ekfstate: EKFState, measurement_type: str):
        """Update ekfstate with z in sensor_state"""

        measurement_model = self.measurement_models[measurement_type]

        x = ekfstate.mean
        P = ekfstate.cov

        v, S = self.innovation(z, ekfstate, measurement_model)

        H = measurement_model.H()
        W = P @ la.solve(S, H).T

        x_upd = x + W @ v
        # P_upd = P - W @ H @ P

        # this version of getting P_upd is more numerically stable
        I = np.eye(*P.shape)
        R = measurement_model.R()
        P_upd = (I - W @ H) @ P @ (I - W @ H).T + W @ R @ W.T

        ekfstate_upd = EKFState(x_upd, P_upd)

        return ekfstate_upd

    def step(self, z, ekfstate, dt):
        """Predict ekfstate dt units ahead and then update this prediction with z in sensor_state."""

        ekfstate_pred = self.predict(ekfstate, dt)
        ekfstate_upd = self.update(z, ekfstate_pred)
        return ekfstate_upd

    def NIS(self, z, ekfstate):
        """Calculate the normalized innovation squared for ekfstate at z in sensor_state"""

        v, S = self.innovation(z, ekfstate)

        cholS = la.cholesky(S, lower=True)

        invcholS_v = la.solve_triangular(cholS, v, lower=True)

        NIS = (invcholS_v ** 2).sum()

        # alternative:
        # NIS = v @ la.solve(S, v)
        return NIS

    def NEES(self, ekfstate, x_true: np.ndarray) -> float:
        """Calculate the normalized etimation error squared from ekfstate to x_true."""

        x, P = ekfstate

        x_diff = x - x_true  # Optional step
        NEES = x_diff.T @ la.inv(P) @ x_diff
        return NEES
