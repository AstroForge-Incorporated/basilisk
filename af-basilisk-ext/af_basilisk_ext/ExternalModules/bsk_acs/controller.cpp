#include "controller.h"
#include <stdexcept>
#include <iostream>

#include <string>

#include "architecture/utilities/linearAlgebra.h"

#include "../_GeneralModuleFiles/eigenSupport.h"

/// @brief Initializes attitude controller
/// @param config_path : Path to config.toml file containing PID parameters
/// @param extra_jitter : Decreases controller derivative gain to increase jitter. For use in testing Perception.
AttitudeController::AttitudeController(const std::string& config_path, bool extra_jitter) {
    // Read in config
    auto config = cpptoml::parse_file(config_path);
    auto params = config->get_table("pid-params");

    auto pid_p = params->get_as<double>("proportional_gain");
    cpptoml::option<double> pid_d;
    if (extra_jitter) {
        pid_d = params->get_as<double>("derivative_gain_extra_jitter");
    } else {
        pid_d = params->get_as<double>("derivative_gain");
    }

    if (pid_p && pid_d) {
        this->pid_p = *pid_p;
        this->pid_d = *pid_d;
    } else {
        throw std::runtime_error("Config could not find proportional_gain or derivative_gain.");
    }
}

/// @brief Controller tick function. Publishes 1) Requested body torque message, to 
/// be used in simulations where RWs are not enabled and instead torques are applied directly to the body
/// and 2) Requested motor torque message, to be used in simulations where RWs are enabled 
/// torques are requested from the RW motors. This motor torque is the *negative* of the requested body torque.
/// @param current_sim_nanos: Current simulation time, in ns
void AttitudeController::UpdateState(uint64_t current_sim_nanos) {
    // Read Att Guide message
    AttGuidMsgPayload attGuideInBuffer;
    attGuideInBuffer = this->guideInMsg();

    Eigen::Array3d sigma_BR = cArray2EigenArray3d(attGuideInBuffer.sigma_BR);
    Eigen::Array3d omega_BR_B = cArray2EigenArray3d(attGuideInBuffer.omega_BR_B);

    // Compute control
    Eigen::Array3d control = sigma_BR * this->pid_p + omega_BR_B * this->pid_d;
    Eigen::Array3d neg_control = -control;

    // Write out message
    CmdTorqueBodyMsgPayload cmdTorqueOutMsgBuffer;
    v3Copy(neg_control.data(), cmdTorqueOutMsgBuffer.torqueRequestBody);
    this->cmdTorqueOutMsg.write(&cmdTorqueOutMsgBuffer, moduleID, current_sim_nanos);

    ArrayMotorTorqueMsgPayload cmdRWTorqueOutMsgBuffer;
    v3Copy(control.data(), cmdRWTorqueOutMsgBuffer.motorTorque);
    this->cmdRWTorqueOutMsg.write(&cmdRWTorqueOutMsgBuffer, moduleID, current_sim_nanos);
}
