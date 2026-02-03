from typing import List

import numpy as np
import matplotlib.pyplot as plt
import scipy
import allantools

from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging

from Basilisk.ExternalModules import accelerometer
from ExternalModules.accelerometer import accel_config

SEED = 3


class TesterFactory(accel_config.AccelFactory):
    """
    Creates test accelerometer with simple parameters for easy testing
    """

    def create_noise_params(self) -> accelerometer.NoiseParams:
        noise_params = accelerometer.NoiseParams()
        noise_params.bias = self.create_bias_params()
        noise_params.noise_density = np.ones(3) * 15e-6 * accel_config.G
        return noise_params

    def create_bias_params(self) -> accelerometer.BiasParams:
        bias_params = accelerometer.BiasParams()
        bias_params.init_val = np.ones(3)
        bias_params.stability = np.ones(3) * 4 * accel_config.G * 1e-6
        bias_params.stability_dt = 100.0
        return bias_params

    def create_quantization_params(self) -> accelerometer.QuantizationParams:
        quant_params = accelerometer.QuantizationParams()
        quant_params.full_scale_range = 5.0
        quant_params.num_bits = 2
        return quant_params

    def create_scale_factor_params(self) -> accelerometer.ScaleFactorParams:
        scale_factor_params = accelerometer.ScaleFactorParams()
        scale_factor_params.init_val = np.ones(3) * 500_000 * accel_config.G
        scale_factor_params.repeatability = np.ones(3) * 600.0 / 1e6
        scale_factor_params.repeatability_dt = 365.0 * 24.0 * 3600.0
        return scale_factor_params

    def create_error_params(self) -> accelerometer.ErrorModelParams:
        error_params = accelerometer.ErrorModelParams()
        error_params.noise = self.create_noise_params()
        error_params.quantization = self.create_quantization_params()
        error_params.scale_factor = self.create_scale_factor_params()
        error_params.deadband_thresh = 200.0e-6
        return error_params


def create_tester_sensor() -> accelerometer.Accelerometer:
    sensor_pos_B = np.array([0.0, 0.0, 1.0])
    accel_frame_B = np.eye(3)
    dt = 1.0
    factory = TesterFactory()
    sensor = factory.create(sensor_pos_B, accel_frame_B, dt)
    sensor.ModelTag = "Accelerometer"

    return sensor


def test_true_accel():
    sensor = create_tester_sensor()
    thrust_force_B = np.array([[1.0], [1.0], [1.0]])
    mass_sc = 100.0
    omega_BN_B = np.array([[2.0], [0.0], [0.0]])
    omega_dot_BN_B = np.array([[0.0], [1.0], [0.0]])
    true_accel = sensor.CalculateTrueAccel(
        thrust_force_B, omega_BN_B, omega_dot_BN_B, mass_sc
    )

    ## Expect omega_BN_B x omega_BN_B x sensor_pos_B --> omega_BN_B x (0.0, -2.0, 0.0)
    ## --> (0.0, 0.0, -4.0)
    ## Expect omega_dot_BN_B x sensor_pos_B --> (1.0, 0.0, 0.0)
    ## Expect thrust_force_B / mass_sc --> (0.01, 0.01, 0.01)
    ## Net: (1.01, 0.01, -3.99)
    assert true_accel == [[1.01], [0.01], [-3.99]]


def test_apply_bias():
    """
    Test accelerometer has an initial bias of [1, 1, 1]
    """
    sensor = create_tester_sensor()
    accel = np.array([[-3.0], [3], [1]])
    out = sensor.ApplyBias(accel)
    assert out == [[-2.0], [4.0], [2.0]]


def test_apply_quantization():
    """
    Test accelerometer has 2 bits (2^2 = 4 chunks) and a range of ±5 m/s^2
    (2.5 m/s^2 per chunk).
    """
    sensor = create_tester_sensor()
    accel = np.array([[-3.0], [3], [1]])
    out = sensor.ApplyQuantization(accel)
    assert out == [[-2.5], [2.5], [0.0]]

    # test cases right on a quantization interval
    accel = np.array([[2.5], [-2.5], [2.5]])
    out = sensor.ApplyQuantization(accel)
    assert out == [[2.5], [-2.5], [2.5]]

    # test all 0's
    accel = np.array([[0.0], [0.0], [0.0]])
    out = sensor.ApplyQuantization(accel)
    assert out == [[0.0], [0.0], [0.0]]


def test_apply_limit():
    """
    Test accelerometer has a range of ±5 m/s^2
    """
    sensor = create_tester_sensor()
    accel = np.array([[-10], [10], [4]])
    out = sensor.ApplyLimit(accel)
    assert out == [[-5], [5], [4]]


def run_velocity_random_walk(factory: accel_config.AccelFactory):
    """
    We choose a low integration time here; velocity random walk
    is derived from the integration of white noise. With high integration times,
    we begin to also add the bias walk.
    """
    dt = 0.1
    sensor_pos_B = np.array([0.0, 0.0, 1.0])
    accel_frame_B = np.eye(3)
    simulation_time = 1  # 1s integration time

    N_RUNS = 1000
    final_vels = []
    thrust = [0.2, 0.2, 0.2]

    ## Initialize outside the loop to avoid resetting the random generator
    sensor = factory.create(sensor_pos_B, accel_frame_B, dt)
    sensor.SetSeed(SEED)
    for _ in range(N_RUNS):
        sensor.Reset(0)
        sensor.thrustOutputInMsgs.clear()  ## Remove thruster messages; in run_sim, we add more
        measurements = run_sim(sensor, simulation_time, thrust, dt)
        integrated_vel = np.sum(measurements, axis=0) * dt
        final_vels.append(integrated_vel)

    final_vels_arr = np.array(final_vels)
    std = np.std(final_vels_arr, axis=0)
    return std


def test_adxl356_allan_variance(show_plots: bool = False):
    """
    Tests ADXL356 allan variance
    """
    factory = accel_config.ADXL356Factory()
    velocity_random_walk = np.array([65e-6, 65e-6, 65e-6])
    bias_instability = np.array([8, 8, 8])
    run_allan_variance(factory, velocity_random_walk, bias_instability, show_plots)


def test_adxl356_vel_random_walk():
    """
    EP datasheet says random walk is 45 mu m / s/ sqrt(hr)
    -- this is likely low; main datasheet (the ADXL-356/357 datasheet
    lists a much higher number of 38mm/s/sqrt(hr)).
    We benchmark 38mm/s/sqrt(hr); expect ~6e-4
    """
    factory = accel_config.ADXL356Factory()
    std = run_velocity_random_walk(factory)
    assert np.allclose(
        std, np.array([7.5e-4, 7.5e-4, 7.5e-4]), atol=2e-4
    ), "Failed ADXL356 random walk test:, found std: {}".format(std)


def test_adxl356_scale_factor_drift():
    """
    Compare ADXL scale factor drift over 1 year against
    datasheet repeatability target, scaled down to 1 year in
    https://www.notion.so/astroforge/Accelerometer-Models-519c6e294f0a4341b46c53d747865b3f?pvs=4#caddd07e6d6c40d8b0ac93577a8bfd1e

    Note: Had to bump scale factor repeatability setting internally
    to hit unit test target.
    """
    factory = accel_config.ADXL356Factory()
    test_length_s = 60 * 60 * 24 * 365  ## 1 year in seconds
    scale_factor_drift = run_scale_factor_drift(factory, test_length_s)
    assert np.all(
        scale_factor_drift > np.array([300, 300, 600])
    ), "Failed ADXL356 scale factor drift test:, found std: {}".format(
        scale_factor_drift
    )


def test_deadband_thresh():
    """
    This function tests the deadband threshold
    """
    sensor = create_tester_sensor()

    # Test deadband thresh
    accel = np.array([[201.0e-6], [200.0e-6], [200.0e-6]])
    out = sensor.ApplyDeadband(accel)
    assert np.allclose(np.squeeze(out), np.array([201e-6, 0.0, 0.0]), atol=1e-4)

    # Test negative deadband thresh
    accel = np.array([[-201.0e-6], [-200.0e-6], [-200.0e-6]])
    out = sensor.ApplyDeadband(accel)
    assert np.allclose(np.squeeze(out), np.array([-201e-6, 0.0, 0.0]), atol=1e-4)


def run_allan_variance(
    factory: accel_config.AccelFactory,
    velocity_random_walk: np.ndarray,
    bias_instability: np.ndarray,
    show_plots: bool,
):
    """
    Function will save allan variance to a plot in the _UnitTest folder;
    check the plot against the sensor datasheet.
    """

    ## Create sensor
    dt = 0.01
    sensor_pos_B = np.array([0.0, 0.0, 1.0])
    accel_frame_B = np.eye(3)
    sensor = factory.create(sensor_pos_B, accel_frame_B, dt)
    simulation_time = 12000
    thrust = [0.2, 0.2, 0.2]
    measurements = run_sim(sensor, simulation_time, thrust, dt)
    measurements = np.array(measurements) / accel_config.G * 1e6  ## Convert to microgee
    (t2, ad_x, _, _) = allantools.oadev(
        measurements[:, 0], rate=1 / dt, data_type="freq"
    )
    (t2, ad_y, _, _) = allantools.oadev(
        measurements[:, 1], rate=1 / dt, data_type="freq"
    )
    (t2, ad_z, _, _) = allantools.oadev(
        measurements[:, 2], rate=1 / dt, data_type="freq"
    )

    # run home grown allan variance
    x = np.cumsum(measurements[:, 0]) * dt
    y = np.cumsum(measurements[:, 1]) * dt
    z = np.cumsum(measurements[:, 2]) * dt
    (tausx, avg_valuesx) = allan_deviation(x, 1 / dt)
    (tausy, avg_valuesy) = allan_deviation(y, 1 / dt)
    (tausz, avg_valuesz) = allan_deviation(z, 1 / dt)

    # univariant spline object makes data smooth and easier to differentiate
    spl_avg_valuesx = scipy.interpolate.UnivariateSpline(tausx, avg_valuesx, s=10)
    spl_avg_valuesy = scipy.interpolate.UnivariateSpline(tausy, avg_valuesy, s=10)
    spl_avg_valuesz = scipy.interpolate.UnivariateSpline(tausz, avg_valuesz, s=10)

    # curve fit to find noise data
    # calculate velocity random walk from deviation data
    # Find the index where the slope of the log-scaled Allan deviation is equal
    # to the slope specified
    VRMx = calculate_noise_at_slope(spl_avg_valuesx, tausx, -0.5, 1.0)
    VRMy = calculate_noise_at_slope(spl_avg_valuesy, tausy, -0.5, 1.0)
    VRMz = calculate_noise_at_slope(spl_avg_valuesz, tausz, -0.5, 1.0)

    # bias instability
    BIASx = calculate_bias_instability_from_allan_deviation(spl_avg_valuesx, tausx, 0.0)
    BIASy = calculate_bias_instability_from_allan_deviation(spl_avg_valuesy, tausy, 0.0)
    BIASz = calculate_bias_instability_from_allan_deviation(spl_avg_valuesz, tausz, 0.0)

    # calculate the VRM line
    lineVRMx = VRMx / np.sqrt(tausx)
    lineVRMy = VRMy / np.sqrt(tausy)
    lineVRMz = VRMz / np.sqrt(tausz)

    # calculate the bias instability line
    scfb = np.sqrt(2 * np.log(2) / np.pi)
    lineBIASx = BIASx * scfb * np.ones(len(tausx))
    lineBIASy = BIASy * scfb * np.ones(len(tausy))
    lineBIASz = BIASz * scfb * np.ones(len(tausz))

    # plot the results
    if show_plots:
        _, ax = plt.subplots(1, 3, figsize=(15, 10))
        ax[0].loglog(t2, ad_x, label=r"$\sigma$")
        ax[0].semilogy(tausx, avg_valuesx, linestyle="--", label=r"$\sigma_{custom}$")
        ax[0].loglog(tausx, lineVRMx, linestyle="--", label=r"$\sigma_{VRM}$")
        ax[0].loglog(tausx, lineBIASx, linestyle="--", label=r"$\sigma_{BI}$")

        ax[0].legend()
        ax[0].grid(which="both")
        ax[0].set_xlabel(r"$\tau$ (sec)")
        ax[0].set_ylabel(r"X Allan Deviation ($\mu$G)")
        ax[0].set_title(
            "VRM: "
            + str("{0:.4g}".format(micro_g_to_mps(VRMx)))
            + r" $\frac{\frac{m}{s}}{\sqrt{hr}}$"
            + "\nBias Inst: "
            + str("{0:.4g}".format(BIASx))
            + r" $\mu$G"
        )

        ax[1].loglog(t2, ad_y, label=r"$\sigma$")
        ax[1].semilogy(tausy, avg_valuesy, linestyle="--", label=r"$\sigma_{custom}$")
        ax[1].loglog(tausy, lineVRMy, linestyle="--", label=r"$\sigma_{VRM}$")
        ax[1].loglog(tausy, lineBIASy, linestyle="--", label=r"$\sigma_{BI}$")
        ax[1].legend()
        ax[1].grid(which="both")
        ax[1].set_xlabel(r"$\tau$ (sec)")
        ax[1].set_ylabel(r"Y Allan Deviation ($\mu$G)")
        ax[1].set_title(
            "VRM: "
            + str("{0:.4g}".format(micro_g_to_mps(VRMy)))
            + r" $\frac{\frac{m}{s}}{\sqrt{hr}}$"
            + "\nBias Inst: "
            + str("{0:.4g}".format(BIASy))
            + r" $\mu$G"
        )

        ax[2].loglog(t2, ad_z, label=r"$\sigma$")
        ax[2].semilogy(tausz, avg_valuesz, linestyle="--", label=r"$\sigma_{custom}$")
        ax[2].loglog(tausz, lineVRMz, linestyle="--", label=r"$\sigma_{VRM}$")
        ax[2].loglog(tausz, lineBIASz, linestyle="--", label=r"$\sigma_{BI}$")
        ax[2].legend()
        ax[2].grid(which="both")
        ax[2].set_xlabel(r"$\tau$ (sec)")
        ax[2].set_ylabel(r"Z Allan Deviation ($\mu$G)")
        ax[2].set_title(
            "VRM: "
            + str("{0:.4g}".format(micro_g_to_mps(VRMz)))
            + r" $\frac{\frac{m}{s}}{\sqrt{hr}}$"
            + "\nBias Inst: "
            + str("{0:.4g}".format(BIASz))
            + r" $\mu$G"
        )

        plt.suptitle("Allan Variance: {}".format(factory.__class__.__name__))
        plt.savefig("allan_variance__{}.png".format(factory.__class__.__name__))
        plt.show()

    # test against spec sheet
    assert np.all(np.array([VRMx, VRMy, VRMz]) > velocity_random_walk)
    assert np.all(np.array([BIASx, BIASy, BIASz]) > bias_instability)


def run_sim(
    sensor: accelerometer.Accelerometer,
    simulation_time: int,
    thrust: List[float],
    dt: float,
):
    testTaskName = "unitTestTask"
    testProcessName = "unitTestProcess"
    testTaskRate = macros.sec2nano(dt)
    sim_time = macros.sec2nano(simulation_time)

    # Create a simulation container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess(testProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(testTaskName, testTaskRate))
    unitTestSim.AddModelToTask(testTaskName, sensor)

    # Create messages
    sc_mass_props_payload = messaging.SCMassPropsMsgPayload()
    sc_mass_props_payload.massSC = 100
    sc_mass_in_msg = messaging.SCMassPropsMsg().write(sc_mass_props_payload)

    thr_in_msgs = []
    for _ in range(5):
        thr_msg_payload = messaging.THROutputMsgPayload()
        thr_msg_payload.thrustForce_B = thrust
        msg = messaging.THROutputMsg().write(thr_msg_payload)
        msg.this.disown()
        thr_in_msgs.append(msg)

    sc_states_payload = messaging.SCStatesMsgPayload()
    sc_states_payload.omega_BN_B = [0.0, 0.0, 0.0]
    sc_states_payload.omegaDot_BN_B = [0.0, 0.0, 0.0]
    sc_states_in_msg = messaging.SCStatesMsg().write(sc_states_payload)

    # Subscribe to messages
    sensor.scMassPropsInMsg.subscribeTo(sc_mass_in_msg)
    for i in range(5):
        sensor.AddThruster(thr_in_msgs[i])
    sensor.scStatesInMsg.subscribeTo(sc_states_in_msg)

    # Add recorder for message out
    dataLog = sensor.sensorOutMsg.recorder()
    unitTestSim.AddModelToTask(testTaskName, dataLog)

    # Initialize and run simulation one step at a time
    unitTestSim.InitializeSimulation()
    # Configure run time and execute simulation
    unitTestSim.ConfigureStopTime(sim_time)
    unitTestSim.ExecuteSimulation()

    return dataLog.accel_AN_A


def allan_deviation(data: np.ndarray, fs: float, maxNumM: int = 100):
    """Compute the Allan deviation (sigma) of time-series data.

    Algorithm obtained from Mathworks:
    https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html

    Args
    ----
        data: 1D data array
        fs: Data sample frequency in Hz
        maxNumM: Number of output points

    Returns
    -------
        (taus, allanDev): Tuple of results
        taus (numpy.ndarray): Array of tau values
        allanDev (numpy.ndarray): Array of computed Allan deviations
    """
    ts = 1.0 / fs
    N = len(data)
    Mmax = 2 ** np.floor(np.log2(N / 2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=maxNumM)
    M = np.ceil(M)  # Round up to integer
    M = np.unique(M)  # Remove duplicates
    taus = M * ts  # Compute 'cluster durations' tau

    # Compute Allan variance
    allan_var = np.zeros(len(M))
    for i, mi in enumerate(M):
        twoMi = int(2 * mi)
        mi = int(mi)
        allan_var[i] = np.sum(
            (data[twoMi:N] - (2.0 * data[mi : N - mi]) + data[0 : N - twoMi]) ** 2
        )

    allan_var /= (2.0 * taus**2) * (N - (2.0 * M))
    return (taus, np.sqrt(allan_var))  # Return deviation (dev = sqrt(var))


# create a function the calculates the velocity random walk from allan deviation data
def calculate_noise_at_slope(adev, tau: np.ndarray, slope: float, start: float):
    """
    This function calculates the velocity random walk from Allan deviation data.
    :param adev: Allan deviation data
    :param tau: tau values
    :param slope: slope of the line to find the velocity random walk
    :param start: start value of the line to find the velocity random walk
    :return: velocity random walk rad/s*sqrt(hz)
    """

    logtau = np.log10(tau)
    logadev = np.log10(adev(tau))
    dlogadev = adev.derivative()(tau)
    idx = np.argmin(abs(dlogadev - slope))

    # Find the y-intercept of the line.
    b = logadev[idx] - slope * logtau[idx]

    # Determine the angle random walk coefficient from the line.
    logN = slope * np.log(start) + b
    N = 10**logN

    return N


def calculate_bias_instability_from_allan_deviation(
    adev, tau: np.ndarray, slope: float
):
    """
    This function calculates the bias instability from Allan deviation data.
    :param adev: Allan deviation data
    :param tau: tau values
    :return: bias instability (rad/s)
    """

    # log of the x and y axis
    logtau = np.log10(tau)
    logadev = np.log10(adev(tau))

    # find derivatives of spline  adev
    first_derivative = adev.derivative(1)(tau)

    # find the index of the first positive value of the slop of the spline fit
    idx = np.argwhere(first_derivative >= 0)[0][0]

    # Find the y-intercept of the line.
    b = logadev[idx] - slope * logtau[idx]
    scfB = np.sqrt(2 * np.log(2) / np.pi)
    logB = b - np.log10(scfB)
    B = 10**logB

    return B


def micro_g_to_mps(micro_g):
    """
    This function converts micro G to m/s/sqrt(hr)
    :param microG: micro G
    :return: m/s/sqrt(hr)
    """
    return micro_g * 9.80665e-6 * 60.0


def run_scale_factor_drift(
    factory: accel_config.AccelFactory, test_length_s: int
) -> np.ndarray:
    """
    Test scale factor uncertainty
    """
    N_RUNS = 10
    dt = 100
    N_TICKS = int(test_length_s / dt)
    drift_ppms = []

    sensor_pos_B = np.array([0.0, 0.0, 1.0])
    accel_frame_B = np.eye(3)

    ## Initialize outside the loop to avoid resetting the random generator
    sensor = factory.create(sensor_pos_B, accel_frame_B, dt)
    sensor.SetSeed(SEED)
    for _ in range(N_RUNS):
        sensor.Reset(0)
        orig_scale_factor = np.array(sensor.GetScaleFactor()).squeeze()
        scale_factors = []
        for _ in range(N_TICKS):
            sensor.WalkScaleFactor(dt)
            new_scale_factor = sensor.GetScaleFactor()
            scale_factors.append(new_scale_factor)
        scale_factors_arr = np.array(scale_factors).squeeze()
        drift = orig_scale_factor - np.mean(scale_factors_arr[-100:], axis=0)
        drift_ppm = np.abs(drift) / orig_scale_factor * 1e6
        drift_ppms.append(drift_ppm)

    drift_ppms_arr: np.ndarray = np.array(drift_ppms)
    avg_drift_ppm = np.mean(drift_ppms_arr, axis=0)
    return avg_drift_ppm


def run_tests(show_plots: bool = False):
    """
    Runs all the tests
    """
    test_apply_quantization()
    test_apply_limit()
    test_apply_bias()
    test_true_accel()
    test_deadband_thresh()

    test_adxl356_allan_variance(show_plots)
    test_adxl356_vel_random_walk()
    test_adxl356_scale_factor_drift()


if __name__ == "__main__":
    run_tests(show_plots=False)
