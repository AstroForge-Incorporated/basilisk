#include "accelerometer.h"
#include "../_GeneralModuleFiles/eigenSupport.h"
#include "../_GeneralModuleFiles/random.h"

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cmath>
#include <iostream>
#include <stdio.h>

/**
 * Constructs accelerometer.
 * @param error_model_params Error model parameters
 * @param accel_frame_B Accelerometer frame definition with respect to the
 * spacecraft body frame
 * @param dt_s Accelerometer polling timestep (s)
 */
Accelerometer::Accelerometer(ErrorModelParams error_model_params,
                             Eigen::Vector3d sensor_pos_B,
                             Eigen::Matrix3d accel_frame_B, double dt_s) {
  // Unpack error model params struct
  this->error_model_params = error_model_params;
  this->bias = error_model_params.noise.bias.init_val;
  this->scale_factor = error_model_params.scale_factor.init_val;

  this->dt_s = dt_s;
  this->sensor_pos_B = sensor_pos_B;
  this->accel_frame_B = accel_frame_B;

  // Set random generator with seed
  std::random_device seed;
  std::mt19937 generator(seed());
  this->generator = generator;
}

/**
 * Class destructor. KABOOM.
 */
Accelerometer::~Accelerometer() {}

/**
 * Resets accelerometer bias and scale factor to their initial values
 */
void Accelerometer::Reset(uint64_t current_sim_nanos) {
  this->bias = this->error_model_params.noise.bias.init_val;
  this->scale_factor = this->error_model_params.scale_factor.init_val;
}

void Accelerometer::AddThruster(Message<THROutputMsgPayload> *msg) {
  /* add the message reader to the vector of input messages */
  this->thrustOutputInMsgs.push_back(msg->addSubscriber());
}

/**
 * Ticks accelerometer forward one step
 * @param current_sim_nanos Current sim time (ns)
 */
void Accelerometer::UpdateState(uint64_t current_sim_nanos) {
  // Read SC mass message
  SCMassPropsMsgPayload scMassPropsInBuffer;
  scMassPropsInBuffer = this->scMassPropsInMsg();

  // Read SC thrust messages
  Eigen::Array3d thrustForce_B = Eigen::Array3d::Zero();

  for (ReadFunctor<THROutputMsgPayload> msg : this->thrustOutputInMsgs) {
    THROutputMsgPayload buff = msg();
    thrustForce_B += cArray2EigenArray3d(buff.thrustForce_B);
  }

  // Read SC angular velocity and accel messages
  SCStatesMsgPayload scStatesInBuffer;
  scStatesInBuffer = this->scStatesInMsg();
  Eigen::Vector3d omega_BN_B =
      cArray2EigenVector3d(scStatesInBuffer.omega_BN_B);
  Eigen::Vector3d omegaDot_BN_B =
      cArray2EigenVector3d(scStatesInBuffer.omegaDot_BN_B);

  // Update bias and sensitivity
  this->WalkBias(this->dt_s);
  this->WalkScaleFactor(this->dt_s);

  // Calculate net acceleration in body frame
  Eigen::Vector3d accel_AN_B = this->CalculateTrueAccel(
      thrustForce_B, omega_BN_B, omegaDot_BN_B, scMassPropsInBuffer.massSC);

  Eigen::Array3d accel_AN_A = (this->accel_frame_B * accel_AN_B).array();

  // Apply deadband
  Eigen::Array3d deadband_accel_AN_A = this->ApplyDeadband(accel_AN_A);

  if (deadband_accel_AN_A.matrix().squaredNorm() == 0.0) {
    this->WriteOutputMessages(deadband_accel_AN_A, current_sim_nanos);
  } else {
    // Apply scale factor
    Eigen::Array3d scaled_accel_AN_A =
        this->ApplyScaleFactor(deadband_accel_AN_A);

    // Apply white noise
    Eigen::Array3d noise_accel_AN_A =
        this->ApplyWhiteNoise(scaled_accel_AN_A, this->dt_s);

    // Apply bias
    Eigen::Array3d biased_accel_AN_A = this->ApplyBias(noise_accel_AN_A);

    // Apply quantization
    Eigen::Array3d quantized_accel_AN_A =
        this->ApplyQuantization(biased_accel_AN_A);

    // Apply limit
    Eigen::Array3d limited_accel_AN_A = this->ApplyLimit(quantized_accel_AN_A);

    this->WriteOutputMessages(limited_accel_AN_A, current_sim_nanos);
  }
}

/**
 * Calculates true accelerometer acceleration, with respect to the inertial
 * frame, in body frame components
 * @param thrust_force_B Thrust force in the body frame (N)
 * @param omega_BN_B Body angular velocity with respect to the inertial frame,
 * in body frame components (rad/s)
 * @param omega_dot_BN_B Body angular acceleration with respect to the inertial
 * frame, in body frame components (rad/s^2)
 * @param mass_sc Mass of the spacecraft
 * @return True accelerometer acceleration, with respect to the inertial frame,
 * in body frame components (m/s^2)
 *
 * @details $a_{accelerometer} = a_{thrust} + omega_dot x r_{sensor} + omega x
 * omega x r_{sensor}$
 */
Eigen::Vector3d Accelerometer::CalculateTrueAccel(
    const Eigen::Array3d &thrust_force_B, const Eigen::Vector3d &omega_BN_B,
    const Eigen::Vector3d &omega_dot_BN_B, double mass_sc) {
  // Calculate net thrust accel in body frame
  Eigen::Array3d thrust_accel_BN_B = thrust_force_B / mass_sc;

  // Calculate total acceleration in the body frame
  Eigen::Vector3d accel_AN_B =
      thrust_accel_BN_B.matrix() +
      (omega_dot_BN_B.cross(this->sensor_pos_B) +
       omega_BN_B.cross(omega_BN_B.cross(this->sensor_pos_B)));

  return accel_AN_B;
}

/**
 * Applies deadband to input acceleration
 * @param accel Acceleration of the accelerometer with respect to the inertial
 * frame, in accelerometer frame components (m/s^2)
 * @return Deadbanded acceleration with respect to inertial frame, in
 * accelerometer frame components (m/s^2)
 */
Eigen::Array3d Accelerometer::ApplyDeadband(const Eigen::Array3d &accel_AN_A) {
  Eigen::Array3d deadbanded_accel_AN_A;
  for (int i = 0; i < accel_AN_A.rows(); ++i) {
    deadbanded_accel_AN_A[i] =
        (abs(accel_AN_A[i]) > this->error_model_params.deadband_thresh)
            ? accel_AN_A[i]
            : 0.0;
  }
  return deadbanded_accel_AN_A;
}

/**
 * Applies scale factor uncertainty to input acceleration. Divides
 * acceleration by current scale factor to get voltage,
 * and multiplies by assumed (initial) scale factor
 * to get sensed acceleration.
 * @param accel Input acceleration (m/s^2)
 * @return Sensed acceleration with respect to inertial frame, in accelerometer
 * frame components (m/s^2)
 */
Eigen::Array3d Accelerometer::ApplyScaleFactor(const Eigen::Array3d &accel) {
  Eigen::Array3d true_voltage = accel / this->scale_factor;
  Eigen::Array3d sensed_accel =
      true_voltage * this->error_model_params.scale_factor.init_val;
  return sensed_accel;
}

/**
 * Corrupts input acceleration with white noise
 * @param accel Input acceleration (m/s^2)
 * @param dt_s: Accel polling timestep (s)
 * @return Corrupted acceleration (m/s^2)
 */
Eigen::Array3d Accelerometer::ApplyWhiteNoise(const Eigen::Array3d &accel,
                                              double dt_s) {
  double hz = 1.0 / dt_s;
  Eigen::Array3d std = this->error_model_params.noise.noise_density * sqrt(hz);
  Eigen::Array3d corrupted_accel = Random::normal(accel, std, this->generator);
  return corrupted_accel;
}

/**
 * Corrupts input acceleration with bias
 * @param accel Input acceleration (m/s^2)
 * @return Corrupted acceleration (m/s^2)
 */
Eigen::Array3d Accelerometer::ApplyBias(const Eigen::Array3d &accel) {
  Eigen::Array3d biased_accel = accel + this->bias;
  return biased_accel;
}

/**
 * Applies sensor quantization to input acceleration
 * @param accel Input acceleration (m/s^2)
 * @return Quantized acceleration (m/s^2)
 */
Eigen::Array3d Accelerometer::ApplyQuantization(const Eigen::Array3d &accel) {
  // Calculate acceleration range represented by each bit
  double &full_scale_range =
      this->error_model_params.quantization.full_scale_range;

  uint8_t &num_bits = this->error_model_params.quantization.num_bits;
  uint64_t num_chunks = 1 << num_bits; // 2^num_bits
  double range_per_chunk = (full_scale_range * 2) /
                           num_chunks; // Multiply by two to get negative range

  Eigen::Array3i reading_chunks = (accel / range_per_chunk).cast<int>();
  Eigen::Array3d quantized_accel =
      reading_chunks.cast<double>() * range_per_chunk;
  return quantized_accel;
}

/**
 * Applies sensor full scale range limit to input acceleration
 * @param accel Input acceleration (m/s^2)
 * @return Limited acceleration (m/s^2)
 */
Eigen::Array3d Accelerometer::ApplyLimit(const Eigen::Array3d &accel) {
  double &full_scale_range =
      this->error_model_params.quantization.full_scale_range;
  Eigen::Array3d limited_accel =
      accel.min(full_scale_range).max(-full_scale_range);
  return limited_accel;
}

/**
 * Applies random walk to bias by input timestep
 * @param dt_s Timestep (s)
 */
void Accelerometer::SetSeed(unsigned int seed) { this->generator.seed(seed); }

/**
 * Applies random walk to bias by input timestep
 * @param dt_s Timestep (s)
 */
void Accelerometer::WalkBias(double dt_s) {
  BiasParams &bias_params = this->error_model_params.noise.bias;
  Eigen::Array3d stability =
      bias_params.stability * sqrt(dt_s / bias_params.stability_dt);
  this->bias = Random::normal(this->bias, stability, this->generator);
}

/**
 * Applies random walk to scale factor by input timestep
 * @param dt_s Timestep (s)
 */
void Accelerometer::WalkScaleFactor(double dt_s) {
  Eigen::Array3d &scale_factor_repeatability =
      this->error_model_params.scale_factor.repeatability;
  double scale_factor_repeat_dt =
      this->error_model_params.scale_factor.repeatability_dt;
  Eigen::Array3d repeatability =
      scale_factor_repeatability * sqrt(dt_s / scale_factor_repeat_dt);
  Eigen::Array3d mean = Eigen::Array3d::Zero();
  this->scale_factor =
      this->scale_factor *
      (1.0 + Random::normal(mean, repeatability, this->generator));
}

/**
 * Returns accelerometer current scale factor
 * @return Scale factor
 */
Eigen::Array3d Accelerometer::GetScaleFactor() { return this->scale_factor; }

/**
 * Writes out module messages
 * @param accel_BN_A Sensor output; measured acceleration of the accelerometer
 * with respect to the inertial frame, in accelerometer frame components
 * @param current_sim_nanos Current sim time (ns)
 */
void Accelerometer::WriteOutputMessages(Eigen::Array3d &accel_AN_A,
                                        uint64_t current_sim_nanos) {
  // Write out message
  AccelMsgPayload accelOutMsgBuffer;
  v3Copy(accel_AN_A.data(), accelOutMsgBuffer.accel_AN_A);
  this->sensorOutMsg.write(&accelOutMsgBuffer, moduleID, current_sim_nanos);
}
