import pytest
import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import macros

from af_plotting.plotting import *
from scenarios.common.basilisk_simulation_core import BasiliskSimulationCore
from af_gnc_utilities.general import array_normalized
from af_gnc_utilities.rigid_body_kinematics import (
    chain_quaternions,
    quaternion_between_vectors,
    quaternion_from_principal_rotation,
    scalar_last_to_scalar_first_q,
)
from af_gnc_utilities.vectors import (
    angle_between_vectors,
    vector_normalized,
)
from scenarios.common.utilities.time_and_spice_utilities import (
    KernelManager,
    get_sun_position_vector_from_astropy,
)

TOLERANCE = 1e-9
VECTOR_TOLERANCE = 1e-3
STANDARD_GRAVITY = 9.80665
ANGLE_TOL_DEG = 0.05
ANGLE_TOL = np.deg2rad(ANGLE_TOL_DEG)
ANGLE_BETWEEN_VECTORS_IN_DEG = True


def get_valid_configuration():
    configuration = {
        "sim_config": {
            "dynamics_task_update_time_step": 0.1,
            "sensor_task_update_time_step": 0.1,
            "SetProgressBar": False,
            "bypass_torque_actuators": False,
            "simulation_start_time": "2024 November 15, 00:00:00.000 UTC",
            "useSphericalHarmonics": False,
            "celestial_bodies": ["sun", "earth"],
            "downsample": True,
            "recorder_sampling_time": 2.0,  # [s]
            "rng_seed": 0,
        },
        "spacecraft": {
            "name": "test_spacecraft",
            "body_inertia": [900, 0, 0, 0, 800, 0, 0, 0, 600],
            "q_i2b_0_nom": [0.0, 0.0, 0.0, 1.0],
            "ang_err_b": [0.0, 0.0, 0.0],
            "omega_BN_B_0": [0.0, 0.0, 0.0],
            "body_mass": 750.0,
            "orbital_state": {
                "method": "fixed",
                "initial_state": "cartesian",
                "r_N": [0.0, 0.0, 0.0],  # [m] position vector in the inertial frame
                "v_N": [0.0, 0.0, 0.0],  # [m/s] velocity vector in the inertial frame
            },
        },
        "fine_sun_sensors": [
            {
                "fov_deg": 45.0,  # half-angle field of view in deg
                "std_dev": 0.0001,
                "q_cfnom2b": [0.5, 0.5, 0.5, 0.5],
                "ang_err_cf": [0.0, 0.0, 0.0],
            }
        ],
    }

    return configuration


@pytest.mark.parametrize("secondary_rotation_axis", ["+x", "+y"])
@pytest.mark.parametrize("secondary_rotation_angle_deg", [0, 30, 44.9, 45.1])
def test_fss(secondary_rotation_axis, secondary_rotation_angle_deg, plot=False):
    kernel_manager = KernelManager()
    kernel_manager.get_kernels()

    configuration = get_valid_configuration()
    sim_duration_minutes = 30.0

    axis_map = {
        "+x": np.atleast_2d([1, 0, 0]).T,
        "+y": np.atleast_2d([0, 1, 0]).T,
    }
    secondary_rotation_axis_cf = axis_map[secondary_rotation_axis]

    if (
        np.abs(secondary_rotation_angle_deg)
        <= configuration["fine_sun_sensors"][0]["fov_deg"]
    ):
        sun_inside_fov = True
    else:
        sun_inside_fov = False

    configuration["spacecraft"]["orbital_state"]["r_N"] = [
        -orbitalMotion.AU * 1000.0,
        +orbitalMotion.AU * 1000.0,
        +orbitalMotion.AU * 1000.0,
    ]

    ##################

    sun_position_vector_from_astropy_inertial_frame_0 = (
        get_sun_position_vector_from_astropy(
            configuration["sim_config"]["simulation_start_time"]
        )
    )

    r_sc_inertial_frame = np.array(configuration["spacecraft"]["orbital_state"]["r_N"])

    r_sc_to_sun_inertial_frame_0 = (
        sun_position_vector_from_astropy_inertial_frame_0 - r_sc_inertial_frame
    )
    r_sc_to_sun_inertial_frame_unit_0 = vector_normalized(r_sc_to_sun_inertial_frame_0)

    fss_boresight_cf = np.atleast_2d([0, 0, 1]).T

    dcm_cf2b_fss = RigidBodyKinematics.EP2C(
        scalar_last_to_scalar_first_q(configuration["fine_sun_sensors"][0]["q_cfnom2b"])
    )
    fss_boresight_body = dcm_cf2b_fss @ fss_boresight_cf

    dcm_i2b = RigidBodyKinematics.EP2C(
        scalar_last_to_scalar_first_q(configuration["spacecraft"]["q_i2b_0_nom"])
    )
    fss_boresight_inertial = dcm_i2b.T @ fss_boresight_body

    # computing the quaternion to rotate sun sensor boresight to the sun vector
    q_align_fss_boresight_to_sun = quaternion_between_vectors(
        fss_boresight_inertial, r_sc_to_sun_inertial_frame_unit_0
    )

    # after sun sensor boresight is aligned with the sun vector, then rotate the spacecraft along the sun sensor x-axis or y-axis to test the FOV
    secondary_rotation_axis_body = dcm_cf2b_fss @ secondary_rotation_axis_cf
    q_secondary_rotation = quaternion_from_principal_rotation(
        secondary_rotation_axis_body, np.deg2rad(secondary_rotation_angle_deg)
    )

    # chain rotations together
    q_combined = chain_quaternions(q_align_fss_boresight_to_sun, q_secondary_rotation)

    configuration["spacecraft"]["q_i2b_0_nom"] = q_combined

    ##########################

    simulation = BasiliskSimulationCore(configuration)

    simulation.setup_simulation_core()
    simulation.create_simulation_process()
    simulation.create_dynamics_task()
    simulation.create_sensor_task()

    simulation.create_spacecraft()
    simulation.activate_spacecraft_rotation()
    simulation.activate_spacecraft_translation()

    simulation.setup_orbital_environment()

    simulation.add_sun_sensor()

    simulation.initialize_simulation()

    simulation.run_simulation(macros.min2nano(sim_duration_minutes))

    ##########################

    kernel_manager.cleanup_kernels()

    sun_position_vector_from_basilisk = simulation.dataSpiceLog[
        simulation.sun_index
    ].PositionVector
    sun_position_vector_from_basilisk_unit = array_normalized(
        sun_position_vector_from_basilisk
    )

    r_sc_to_sun_inertial_frame = sun_position_vector_from_basilisk - r_sc_inertial_frame
    r_sc_to_sun_inertial_frame_unit = array_normalized(r_sc_to_sun_inertial_frame)

    sHat_sensed_inertial_frame = []
    angle_between_sensed_and_truth = []
    for idx, (sHat_sensed_cf, sHat_true_cf) in enumerate(
        zip(
            simulation.fssDataOutMsg_log_list[0].sHat_sensed,
            simulation.fssDataOutMsg_log_list[0].sHat_true,
        )
    ):
        if sun_inside_fov:
            dcm_i2b = RigidBodyKinematics.MRP2C(
                simulation.spacecraft_data_logging_module.sigma_BN[idx]
            )

            sHat_sensed_body = dcm_cf2b_fss @ np.atleast_2d(sHat_sensed_cf).T
            sHat_true_body = dcm_cf2b_fss @ np.atleast_2d(sHat_true_cf).T

            sHat_sensed_inertial_frame.append(dcm_i2b.T @ sHat_sensed_body)

            angle_between_sensed_and_truth.append(
                angle_between_vectors(
                    sHat_sensed_inertial_frame[-1],
                    r_sc_to_sun_inertial_frame_unit[idx],
                    DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
                )
            )

            dcm_combined = RigidBodyKinematics.EP2C(
                scalar_last_to_scalar_first_q(q_combined)
            )
            final_boresight_inertial = dcm_combined.T @ fss_boresight_body

            assert (
                np.abs(
                    angle_between_vectors(
                        final_boresight_inertial,
                        r_sc_to_sun_inertial_frame_unit_0,
                        DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
                    )
                    - secondary_rotation_angle_deg
                )
                < ANGLE_TOL_DEG
            ), "FSS boresight, and therefore spacecraft was not aligned properly to begin with!"

            assert (
                angle_between_sensed_and_truth[-1] < ANGLE_TOL_DEG
            ), "Angle between corrupted spacecraft to sun vector as measured by sun sensor is not close to the truth vector from Basilisk!"
            assert (
                angle_between_vectors(
                    dcm_i2b.T @ sHat_true_body,
                    r_sc_to_sun_inertial_frame_unit[idx],
                    DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
                )
                < ANGLE_TOL_DEG
            ), "Angle between uncorrupted spacecraft to sun vector as measured by sun sensor is not close to the truth vector from Basilisk!"
            assert (
                angle_between_vectors(
                    sHat_sensed_body,
                    sHat_true_body,
                    DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
                )
                < ANGLE_TOL_DEG
            ), "Uncorrupted and corrupted sun sensor measurement are not close to each other!"
        else:
            assert np.allclose(sHat_true_cf, 0, atol=TOLERANCE)
            assert np.allclose(sHat_sensed_cf, 0, atol=TOLERANCE)

    # running this test because we are using astropy to get the sun position vector before running the simulation
    # Based on Example 5-1 on p. 280 of Vallado, "Fundamentals of Astrodynamics and Applications, Fourth Edition: https://astroforge.box.com/s/nr7030gkgbuiefbdcqus7viejo0s1w0z
    vallado_time = "2006 April 2, 00:00:00.000 UTC"

    sun_position_vector_from_vallado = (
        np.array([146259922.0, 28595947.0, 12397430.0]) * 1e3
    )
    assert (
        angle_between_vectors(
            get_sun_position_vector_from_astropy(vallado_time),
            sun_position_vector_from_vallado,
            DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
        )
        < ANGLE_TOL_DEG
    ), "Sun position vector from Vallado is not close to vector from Astropy!"
    assert (
        angle_between_vectors(
            sun_position_vector_from_basilisk_unit[0],
            sun_position_vector_from_astropy_inertial_frame_0,
            DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
        )
        < ANGLE_TOL_DEG
    ), "Initial Sun position vector from Astropy is not close to vector from Basilisk!"

    if sun_inside_fov:
        assert np.allclose(
            angle_between_vectors(
                simulation.fssDataOutMsg_log_list[0].sHat_sensed[0],
                fss_boresight_cf,
                DEGREES=ANGLE_BETWEEN_VECTORS_IN_DEG,
            ),
            np.abs(secondary_rotation_angle_deg),
            atol=ANGLE_TOL_DEG,
        ), "FSS boresight is not aligned with sun at the start of the simulation!"

        sHat_sensed = simulation.fssDataOutMsg_log_list[0].sHat_sensed[0]
        # Check if the secondary rotation angle is not close to 0
        if not np.isclose(secondary_rotation_angle_deg, 0, atol=TOLERANCE):
            # if the second rotation is about the x axis, then there should not be an x axis component in the measurement
            if np.array_equal(secondary_rotation_axis, np.array([1, 0, 0])):
                assert np.isclose(
                    sHat_sensed[0], 0, atol=VECTOR_TOLERANCE
                ), "X axis should be close to 0 but is not."
                assert not np.isclose(
                    sHat_sensed[1], 0, atol=VECTOR_TOLERANCE
                ), "Y axis should not be close to 0 but is."
                assert not np.isclose(
                    sHat_sensed[2], 0, atol=VECTOR_TOLERANCE
                ), "Z axis should not be close to 0 but is."
            # if the second rotation is about the y axis, then there should not be a y axis component in the measurement
            elif np.array_equal(secondary_rotation_axis, np.array([0, 1, 0])):
                assert np.isclose(
                    sHat_sensed[1], 0, atol=VECTOR_TOLERANCE
                ), "Y axis should be close to 0 but is not."
                assert not np.isclose(
                    sHat_sensed[0], 0, atol=VECTOR_TOLERANCE
                ), "X axis should not be close to 0 but is."
                assert not np.isclose(
                    sHat_sensed[2], 0, atol=VECTOR_TOLERANCE
                ), "Z axis should not be close to 0 but is."

    ##########################

    if plot:
        time = macros.NANO2SEC * simulation.spacecraft_data_logging_module.times()

        plot_general(
            time,
            sun_position_vector_from_basilisk_unit,
            "Sun Position Unit Vector from Basilisk",
        )

        plot_data_comparison(
            time,
            simulation.fssDataOutMsg_log_list[0].sHat_sensed,
            "Corrupted FSS measurement",
            simulation.fssDataOutMsg_log_list[0].sHat_true,
            "Uncorrupted FSS measurement",
            "Spacecraft to Sun Unit Vector, Body Frame",
            ylim=[-1.1, 1.1],
        )

        if sun_inside_fov:
            plot_data_comparison(
                time,
                np.array(sHat_sensed_inertial_frame),
                "Corrupted FSS measurement",
                r_sc_to_sun_inertial_frame_unit,
                "Basilisk Truth",
                "Spacecraft to Sun Unit Vector",
                ylim=[-1.1, 1.1],
            )
            plot_general(
                time,
                np.array(angle_between_sensed_and_truth),
                "Angle between FSS measurement and Basilisk truth [deg]",
            )

        plot_general(
            time,
            simulation.spacecraft_data_logging_module.sigma_BN,
            r"Spacecraft Attitude, $\sigma_{i2b}$",
        )

        plot_general(
            time,
            np.rad2deg(simulation.spacecraft_data_logging_module.omega_BN_B),
            r"Spacecraft Angular Velocity, $\omega_{b}$ [deg/s]",
        )
        plot_general(
            time,
            simulation.spacecraft_data_logging_module.r_BN_N,
            "Spacecraft Position in Inertial Frame [m]",
        )
        plot_general(
            time,
            simulation.spacecraft_data_logging_module.v_BN_N,
            "Spacecraft Velocity in Inertial Frame [m/s]",
        )
        plot_general(
            time,
            simulation.spacecraft_data_logging_module.nonConservativeAccelpntB_B,
            "Spacecraft Acceleration in Inertial Frame [m/s^2]",
        )

    ##############################


if __name__ == "__main__":
    plot_output = True

    test_fss(
        secondary_rotation_axis="+x", secondary_rotation_angle_deg=30, plot=plot_output
    )

    if plot_output:
        plt.show()
