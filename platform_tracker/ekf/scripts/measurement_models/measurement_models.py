
from pyexpat import model
import numpy as np

def get_measurement_models_from_types(measurement_model_types: list, measurement_models_config: dict):

    measurement_models = {}

    for model_type in measurement_model_types:
        if model_type == "dnn_cv_position":
            measurement_model = DnnCvPosition
        elif model_type == "dnn_cv_position_xy":
            measurement_model = DnnCvPositionXY
        elif model_type == "drone_velocity":
            measurement_model = DroneVelocity
        elif model_type == "apriltags_position":
            measurement_model = AprilTagsPosition
        elif model_type == "altitude_measurement":
            measurement_model = AltitudeMeasurement
        elif model_type == "gnss_measurement":
            measurement_model = GnssMeasurement
        else:
            raise NotImplementedError

        measurement_models[model_type] = measurement_model(measurement_models_config[model_type]["sigmas"])

    return measurement_models


class MeasurementModel():

    def __init__(self, sigmas):
        """
        Generic measurement model class.
        """

        self._R = np.diag(sigmas)**2
        self._H = None

    def h(self, x):
        """
        This function must be implemented for each instance of the class.
        """
        raise NotImplementedError

    def H(self):
        return self._H

    def R(self):
        return self._R


class GnssMeasurement(MeasurementModel):
    def __init__(self, sigmas):
        """
        Measurement model for GNSS measurements coming from the internal EKF of the Anafi.

        Frame of reference: GNSS measurements given in body frame with origin at init position of drone.
        z = [x,y,z]

        """
        super().__init__(sigmas)

        self._H = np.hstack((np.eye(3), np.zeros((3,3))))

    def h(self, x):
        return x[:3]

class AltitudeMeasurement(MeasurementModel):
    def __init__(self, sigmas):
        """
        Measurement model for altitide measurements coming from the internal EKF of the Anafi

        Frame of reference: Altitide over ground measurement from the Anafi drone in body frame
        z = [z]

        """
        super().__init__(sigmas)

        self._H = np.hstack((np.array([0,0,1]).reshape(1,3), np.zeros((1,3))))

    def h(self, x):
        return x[2]

class DnnCvPosition(MeasurementModel):

    def __init__(self, sigmas):
        """
        Measurement model for position measurements coming from the DNN CV module

        Frame of reference: All positions correspond to the helipad position
        relative to the drone body frame
        z = [x, y, z]

        """
        super().__init__(sigmas)

        self._H = np.hstack((np.eye(3), np.zeros((3,3))))

    def h(self, x):
        return x[:3]

class DnnCvPositionXY(MeasurementModel):

    def __init__(self, sigmas):
        """
        Measurement model for XY position measurements only coming from the DNN CV module

        Frame of reference: All positions correspond to the helipad position
        relative to the drone body frame
        z = [x, y]

        """
        super().__init__(sigmas)

        self._H = np.hstack((np.eye(2), np.zeros((2,4))))

    def h(self, x):
        return x[:2]

class AprilTagsPosition(MeasurementModel):

    def __init__(self, sigmas):
        """
        Measurement model for position measurements coming from the AprilTag module

        Frame of reference: All positions correspond to the helipad position
        relative to the drone body frame
        z = [x, y, z]

        """
        super().__init__(sigmas)

        self._H = np.hstack((np.eye(3), np.zeros((3,3))))

    def h(self, x):
        return x[:3]

class DroneVelocity(MeasurementModel):
    def __init__(self, sigmas):
        """
        Measurement model for drone velocity measurements coming from
        drone internal EKF.

        Frame of reference: All positions correspond to the helipad position
        relative to the drone body frame
        z = [v_x, v_y, v_z]

        """
        super().__init__(sigmas)

        self._H = np.hstack((np.zeros((3,3)), np.eye(3)))

    def h(self, x):
        return x[3:]
