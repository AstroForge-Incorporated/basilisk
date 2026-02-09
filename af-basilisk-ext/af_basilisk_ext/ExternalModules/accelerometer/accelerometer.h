#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <random>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/avsEigenSupport.h"

#include "../../msgPayloadDefC/AccelMsgPayload.h"
#include "architecture/msgPayloadDefC/SCMassPropsMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"

struct BiasParams {
  Eigen::Array3d init_val;
  Eigen::Array3d stability;
  double stability_dt;
};

struct NoiseParams {
  BiasParams bias;
  Eigen::Array3d noise_density;
};

struct ScaleFactorParams {
  Eigen::Array3d init_val;
  Eigen::Array3d repeatability;
  double repeatability_dt;
};

struct QuantizationParams {
  double full_scale_range; // in m/s^2, maximum value accel
                           // can record (we assume it record
                           // ± this value)
  uint8_t num_bits;
};

struct ErrorModelParams {
  NoiseParams noise;
  ScaleFactorParams scale_factor;
  QuantizationParams quantization;
  double deadband_thresh;
};

class Accelerometer : public SysModel {
public:
  Accelerometer(ErrorModelParams error_model_params,
                Eigen::Vector3d sensor_pos_B, Eigen::Matrix3d accel_frame_B,
                double dt); // To implement later: Accel
                            // rotation relative to body
  ~Accelerometer();

  void Reset(uint64_t current_sim_nanos);
  void UpdateState(uint64_t current_sim_nanos);
  void AddThruster(Message<THROutputMsgPayload> *msg);

public:
  // Input messages
  ReadFunctor<SCMassPropsMsgPayload> scMassPropsInMsg;
  std::vector<ReadFunctor<THROutputMsgPayload>> thrustOutputInMsgs;
  ReadFunctor<SCStatesMsgPayload> scStatesInMsg;

  // Output messages
  Message<AccelMsgPayload> sensorOutMsg;

public: // We define these as public functions (even though they should really
        // be private) for ease of testing.
  // Accel math operations
  Eigen::Vector3d CalculateTrueAccel(const Eigen::Array3d &thrust_force_B,
                                     const Eigen::Vector3d &omega_BN_B,
                                     const Eigen::Vector3d &omega_dot_BN_B,
                                     double mass_sc);
  Eigen::Array3d ApplyDeadband(const Eigen::Array3d &accel);
  Eigen::Array3d ApplyScaleFactor(const Eigen::Array3d &accel);
  Eigen::Array3d ApplyWhiteNoise(const Eigen::Array3d &accel, double dt);
  Eigen::Array3d ApplyBias(const Eigen::Array3d &accel);
  Eigen::Array3d ApplyQuantization(const Eigen::Array3d &accel);
  Eigen::Array3d ApplyLimit(const Eigen::Array3d &accel);

  Eigen::Array3d GetScaleFactor();
  void SetSeed(unsigned int seed);
  void WalkBias(double dt);
  void WalkScaleFactor(double dt);

  // Module operations
  void WriteOutputMessages(Eigen::Array3d &accel_BN_A,
                           uint64_t current_sim_nanos);

protected:
  // Sensor parameters
  ErrorModelParams error_model_params;
  Eigen::Vector3d sensor_pos_B;
  Eigen::Matrix3d accel_frame_B;
  double dt_s;

  // Random generator
  std::mt19937 generator;

  // Initial bias and scale factor
  Eigen::Array3d bias;
  Eigen::Array3d scale_factor;
};

#endif
