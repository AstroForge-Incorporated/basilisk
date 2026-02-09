from abc import ABC, abstractmethod
from enum import Enum

import numpy as np

from Basilisk.ExternalModules import accelerometer

G = 9.81


class AccelFactory(ABC):
    """
    Abstract class for creating accelerometer objects
    """

    def create(
        self, sensor_pos_B: np.ndarray, accel_frame_B: np.ndarray, dt: float
    ) -> accelerometer.Accelerometer:
        """
        Creates accelerometer.
        @params:
        - sensor_pos_B: arr[3]: Sensor position with respect to the body,
            in meters in body frame coordinates
        - accel_frame_B: arr[3][3]: Accelerometer frame definition with respect
            to the body frame
        - dt_s: Accelerometer tick rate (s)
        """
        error_params = self.create_error_params()
        accel = accelerometer.Accelerometer(
            error_params, sensor_pos_B, accel_frame_B, dt
        )
        return accel

    def create_custom_error_params(
        self,
        error_model_params: accelerometer.ErrorModelParams,
        sensor_pos_B: np.ndarray,
        accel_frame_B: np.ndarray,
        dt: float,
    ) -> accelerometer.Accelerometer:
        """
        Creates accelerometer.
        @params:
        - sensor_pos_B: arr[3]: Sensor position with respect to the body,
            in meters in body frame coordinates
        - accel_frame_B: arr[3][3]: Accelerometer frame definition with respect
            to the body frame
        - dt_s: Accelerometer tick rate (s)
        """
        accel = accelerometer.Accelerometer(
            error_model_params, sensor_pos_B, accel_frame_B, dt
        )
        return accel

    @abstractmethod
    def create_error_params(self) -> accelerometer.ErrorModelParams:
        pass


class ADXL356Factory(AccelFactory):
    """
    Creates Accelerometer with parameters tuned to match Analog Devices ADXL356 accelerometer
    https://www.analog.com/media/en/technical-documentation/data-sheets/adxl356-ep.pdf
    """

    def create_noise_params(self) -> accelerometer.NoiseParams:
        noise_params = accelerometer.NoiseParams()
        noise_params.bias = self.create_bias_params()
        noise_params.noise_density = np.ones(3) * 80e-6 * G
        return noise_params

    def create_bias_params(self) -> accelerometer.BiasParams:
        bias_params = accelerometer.BiasParams()
        bias_params.init_val = np.zeros(3)
        bias_params.stability = np.ones(3) * 8 * G * 1e-6
        bias_params.stability_dt = 100.0
        return bias_params

    def create_scale_factor_params(self) -> accelerometer.ScaleFactorParams:
        scale_factor_params = accelerometer.ScaleFactorParams()
        scale_factor_params.init_val = np.ones(3) * 80.0
        scale_factor_params.repeatability = (
            np.array([0.1 / 100, 0.1 / 100, 0.3 / 100]) * 1.5
        )
        scale_factor_params.repeatability_dt = 10.0 * 365.0 * 24.0 * 3600.0
        return scale_factor_params

    def create_quantization_params(self) -> accelerometer.QuantizationParams:
        quant_params = accelerometer.QuantizationParams()
        quant_params.full_scale_range = 10 * G
        quant_params.num_bits = 20
        return quant_params

    def create_error_params(self) -> accelerometer.ErrorModelParams:
        error_params = accelerometer.ErrorModelParams()
        error_params.noise = self.create_noise_params()
        error_params.quantization = self.create_quantization_params()
        error_params.scale_factor = self.create_scale_factor_params()
        error_params.deadband_thresh = 0 * G * 1e-6
        return error_params


class PerfectAccelFactory(AccelFactory):
    """
    Creates "perfect" accelerometer with no noise, no bias drift, no scale factor drift, and
    low quantization noise
    """

    def create_noise_params(self) -> accelerometer.NoiseParams:
        noise_params = accelerometer.NoiseParams()
        noise_params.bias = self.create_bias_params()
        noise_params.noise_density = np.zeros(3)  ## No noise
        return noise_params

    def create_bias_params(self) -> accelerometer.BiasParams:
        bias_params = accelerometer.BiasParams()
        bias_params.init_val = np.zeros(3)
        bias_params.stability = np.zeros(3)  ## 0 stabilty --> 0 bias drift
        bias_params.stability_dt = 100.0  ## This param doesn' matter. Setting to nonzero to avoid divide by 0 error
        return bias_params

    def create_scale_factor_params(self) -> accelerometer.ScaleFactorParams:
        scale_factor_params = accelerometer.ScaleFactorParams()
        scale_factor_params.init_val = np.ones(3)
        scale_factor_params.repeatability = np.zeros(
            3
        )  ## 0 repeatability --> 0 scale factor drift
        scale_factor_params.repeatability_dt = 100.0  ## This param doesn' matter. Setting to nonzero to avoid divide by 0 error
        return scale_factor_params

    def create_quantization_params(self) -> accelerometer.QuantizationParams:
        quant_params = accelerometer.QuantizationParams()
        quant_params.full_scale_range = 10 * G
        quant_params.num_bits = (
            2_000  ## Setting to large number, to allow for very fine discretization
        )
        return quant_params

    def create_error_params(self) -> accelerometer.ErrorModelParams:
        error_params = accelerometer.ErrorModelParams()
        error_params.noise = self.create_noise_params()
        error_params.quantization = self.create_quantization_params()
        error_params.scale_factor = self.create_scale_factor_params()
        error_params.deadband_thresh = 0 * G * 1e-6
        return error_params


class Accelerometers(Enum):
    ADXL356 = ADXL356Factory
    Perfect = PerfectAccelFactory
