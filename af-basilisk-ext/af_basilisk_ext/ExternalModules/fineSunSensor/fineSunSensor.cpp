/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "fineSunSensor.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include <math.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <inttypes.h>

//! Initialize a bunch of defaults in the constructor.
FineSunSensor::FineSunSensor()
{
    this->r_B.fill(0.0);
    this->nHat_B.fill(0.0);
    this->fov = 1.0471975512;
    this->dcm_cf2b.setIdentity(3, 3);
    this->sHat_M_true.fill(0.0);
    this->sHat_M_sensed.fill(0.0);
    this->senNoiseBound = 1.0;

    // initialize the uniform distributions with constant bounds
    this->dist_minus1_to_1 = UniformDist(-1.0, 1.0);
    this->dist_0_to_2pi = UniformDist(0.0, 2.0 * M_PI);

    return;
}

//! There is nothing to do in the default destructor
FineSunSensor::~FineSunSensor()
{
    return;
}

/*! This method is used to reset the module.
 @param CurrentSimNanos The current simulation time from the architecture
 @return void */
void FineSunSensor::Reset(uint64_t CurrentSimNanos)
{
    //! - If either messages is not valid, send a warning message
    if (!this->sunInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "FineSunSensor: Failed to link a sun sensor input message");
    }
    if (!this->stateInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "FineSunSensor: Failed to link a spacecraft state input message");
    }
    if (!this->RNGSeed_initialization_flag)
    {
        bskLogger.bskLog(BSK_ERROR, "FineSunSensor: RNGSeed has not been initialized. Please set RNGSeed using the method: setRNGSeed(uint32_t seed).");
    }
    if (!this->senNoiseBound_initialization_flag)
    {
        bskLogger.bskLog(BSK_ERROR, "FineSunSensor: senNoiseBound has not been initialized. Please set setSensorNoiseBound using the method: setSensorNoiseBound(double senNoiseBound).");
    }
}

void FineSunSensor::readInputMessages()
{
    //! - Zero ephemeris information
    this->sunData = this->sunInMsg.zeroMsgPayload;
    this->stateCurrent = this->stateInMsg.zeroMsgPayload;

    //! - If we have a valid sun ID, read Sun ephemeris message
    if (this->sunInMsg.isLinked())
    {
        this->sunData = this->sunInMsg();
    }
    //! - If we have a valid state ID, read vehicle state ephemeris message
    if (this->stateInMsg.isLinked())
    {
        this->stateCurrent = this->stateInMsg();
    }
}

/*! This method computes the sun-vector heading information in the vehicle
 body frame.*/
void FineSunSensor::computeSunData()
{
    Eigen::Vector3d Sc2Sun_Inrtl;
    Eigen::Vector3d sHat_N;
    Eigen::Matrix3d dcm_BN;

    Eigen::Vector3d r_BN_N_eigen;
    Eigen::Vector3d sunPos;
    Eigen::MRPd sigma_BN_eigen;

    //! - Get the position from spacecraft to Sun

    //! - Read Message data to eigen
    r_BN_N_eigen = cArray2EigenVector3d(this->stateCurrent.r_BN_N);
    sunPos = cArray2EigenVector3d(this->sunData.PositionVector);
    sigma_BN_eigen = cArray2EigenMRPd(this->stateCurrent.sigma_BN);

    //! - Find sun heading unit vector
    Sc2Sun_Inrtl = sunPos - r_BN_N_eigen;
    sHat_N = Sc2Sun_Inrtl / Sc2Sun_Inrtl.norm();

    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    dcm_BN = sigma_BN_eigen.toRotationMatrix().transpose(); // transpose() required because toRotationMatrix() returns the transpose so we must transpose back.
    this->sHat_B = dcm_BN * sHat_N;
}

/*! This method computes the true sensed values for the sensor */
void FineSunSensor::computeTrueOutput()
{
    // Rotation of sun vector body frame (B) to sensor frame (S) to misaligned sensor frame (M)
    this->sHat_M_true = this->dcm_cf2b.transpose() * this->sHat_B;
    this->sHat_M_true = this->sHat_M_true / this->sHat_M_true.norm();

    // Check if the sun vector is within the sensor's field of view along the +Z axis and within the 2 rectangular FOV angles
    if (this->sHat_M_true[2] <= 0 ||
        abs(atan2(this->sHat_M_true[0], this->sHat_M_true[2])) > this->fov ||
        abs(atan2(this->sHat_M_true[1], this->sHat_M_true[2])) > this->fov)
    {
        // If the sun vector is not within the FOV, set it to zero
        this->sHat_M_true.fill(0.0);
    }

    // If sun heading is within sensor field of view, compute signal
    this->nHat_B = this->dcm_cf2b.col(2);
    double signal = this->nHat_B.dot(this->sHat_B);
    this->trueValue = signal >= cos(this->fov) ? signal : 0.0;
}

/*! This method takes the true observed vector converts
 it over to an errored value.  It applies noise to the truth. */
void FineSunSensor::applySensorErrors()
{
    double sHat_M_true_atan_xz = atan2(this->sHat_M_true[0], this->sHat_M_true[2]);
    double sHat_M_true_atan_yz = atan2(this->sHat_M_true[1], this->sHat_M_true[2]);

    //! - If the standard deviation is not positive, do not add noise
    if (this->senNoiseBound <= 0.0 ||
        this->sHat_M_true[2] <= 0 ||
        abs(sHat_M_true_atan_xz) > this->fov ||
        abs(sHat_M_true_atan_yz) > this->fov)
    {
        this->sHat_M_sensed = this->sHat_M_true;
    }
    else
    {
        // Generate random values from the uniform distributions
        double u = this->dist_minus1_to_1(this->rng);
        double theta = this->dist_0_to_2pi(this->rng);
        double magnitudeOfError = this->dist_0_to_senNoiseBound(this->rng);

        // this constructs a unit vector which is uniformly distributed over the surface of a unit sphere
        // See equation 6-8 on https://mathworld.wolfram.com/SpherePointPicking.html
        double temp = sqrt(1 - pow(u, 2.));
        double x = temp * cos(theta);
        double y = temp * sin(theta);
        double z = u;
        Eigen::Vector3d directionOfError_unitVector(x, y, z);

        Eigen::Vector3d perpendicular_vec = this->sHat_M_true.cross(directionOfError_unitVector);

        double dcm_noise[3][3];
        double prv_noise[3];
        Eigen::Vector3d prv_noise_eigen = perpendicular_vec * magnitudeOfError;
        eigenVector3d2CArray(prv_noise_eigen, prv_noise);
        PRV2C(prv_noise, dcm_noise);

        this->sHat_M_sensed = c2DArray2EigenMatrix3d(dcm_noise) * this->sHat_M_true;
    }
}

/*! This method writes the output message.  The output message contains the
 current output of the FSS converted over to some discrete "counts" to
 emulate ADC conversion of S/C.
 @param Clock The current simulation time*/
void FineSunSensor::writeOutputMessages(uint64_t Clock)
{
    if (this->fssDataOutMsg.isLinked())
    {
        FSSRawDataMsgPayload localMessage;
        //! - Zero the output message
        localMessage = this->fssDataOutMsg.zeroMsgPayload;
        //! - Set the outgoing data
        eigenVector3d2CArray(this->sHat_B, localMessage.sHat_true_body_frame);
        eigenVector3d2CArray(this->sHat_M_true, localMessage.sHat_true);
        eigenVector3d2CArray(this->sHat_M_sensed, localMessage.sHat_sensed);
        //! - Write the outgoing message to the architecture
        this->fssDataOutMsg.write(&localMessage, this->moduleID, Clock);
    }

    // create FSS configuration log message
    if (this->fssConfigLogOutMsg.isLinked())
    {
        FSSConfigLogMsgPayload configMsg;
        configMsg = this->fssConfigLogOutMsg.zeroMsgPayload;
        configMsg.fov = this->fov;
        configMsg.solar_irradiance = this->trueValue;
        if (this->FSSGroupID >= 0)
        {
            configMsg.FSSGroupID = this->FSSGroupID;
        }
        eigenVector3d2CArray(this->r_B, configMsg.r_B);
        configMsg.senNoiseBound = this->senNoiseBound;
        eigenMatrix3d2CArray(this->dcm_cf2b, configMsg.dcm_cf2b);

        this->fssConfigLogOutMsg.write(&configMsg, this->moduleID, Clock);
    }
}

/*! This method is called at a specified rate by the architecture.  It makes the
 calls to compute the current sun information and write the output message for
 the rest of the model.
 @param CurrentSimNanos The current simulation time from the architecture*/
void FineSunSensor::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the inputs
    this->readInputMessages();
    //! - Get sun vector
    this->computeSunData();
    //! - compute true cosine
    this->computeTrueOutput();
    //! - Apply any set errors
    this->applySensorErrors();
    //! - Write output data
    this->writeOutputMessages(CurrentSimNanos);
}

void FineSunSensor::setRNGSeed(uint32_t seed)
{
    this->RNGSeed_initialization_flag = true;

    this->RNGSeed = seed;

    // initialize the random number generator with a fixed seed for repeatable results
    // this is the same fixed seed that Basilisk uses for the Gauss-Markov module
    this->rng = std::mt19937(this->RNGSeed);
}

void FineSunSensor::setSensorNoiseBound(double senNoiseBound)
{
    this->senNoiseBound_initialization_flag = true;

    this->senNoiseBound = senNoiseBound;

    // Initialize the distribution with the new standard deviation
    this->dist_0_to_senNoiseBound = UniformDist(0.0, this->senNoiseBound);
}
