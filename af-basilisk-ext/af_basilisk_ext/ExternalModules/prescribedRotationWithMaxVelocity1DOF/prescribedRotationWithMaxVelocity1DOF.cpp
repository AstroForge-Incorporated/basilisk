/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "prescribedRotationWithMaxVelocity1DOF.h"
#include <cmath>
#include <iostream>

//! This method self initializes the C-wrapped output messages.
void PrescribedRotationWithMaxVelocity1DOF::SelfInit()
{
    HingedRigidBodyMsg_C_init(&this->spinningBodyOutMsgC);
    PrescribedRotationMsg_C_init(&this->prescribedRotationOutMsgC);
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.

    @param callTime [ns] Time the method is called
*/
void PrescribedRotationWithMaxVelocity1DOF::Reset(uint64_t callTime)
{
    if (!this->spinningBodyInMsg.isLinked())
    {
        _bskLog(this->bskLogger, BSK_ERROR, "prescribedRotationWithMaxVelocity1DOF.spinningBodyInMsg wasn't connected.");
    }
    if (this->rotHat_M.norm() != 1.0)
    {
        _bskLog(this->bskLogger, BSK_ERROR, "prescribedRotationWithMaxVelocity1DOF: RotHat_M must be a unit vector.");
    }
    if (this->thetaDotMax <= 0)
    {
        _bskLog(this->bskLogger, BSK_ERROR, "prescribedRotationWithMaxVelocity1DOF: thetaDotMax must be greater than zero.");
    }
    if (this->thetaDDotMax <= 0)
    {
        _bskLog(this->bskLogger, BSK_ERROR, "prescribedRotationWithMaxVelocity1DOF: thetaDDotMax must be greater than zero.");
    }

    // Set the initial convergence to true to enter the required loop in Update() method on the first pass
    this->convergence = true;
}

/*! This method profiles the spinning body rotation and updates the prescribed rotational states as a function of time.
    The spinning body rotational states are then written to the output message.

    @param callTime [ns] Time the method is called
*/
void PrescribedRotationWithMaxVelocity1DOF::UpdateState(uint64_t callTime)
{
    // Read the input message
    HingedRigidBodyMsgPayload spinningBodyIn = HingedRigidBodyMsgPayload();
    if (this->spinningBodyInMsg.isWritten())
    {
        spinningBodyIn = this->spinningBodyInMsg();
    }

    // Compute absolute difference between new target and previous reference, checks if change exceeds threshold
    bool target_change_exceeds_threshold = fabs(spinningBodyIn.theta - this->thetaRef) > this->newTargetAngularThreshold;

    // This loop is entered initially and if both (a) and (b) hold:
    // -> (a) new message, and (b) either convergence achieved OR significant change in target
    // The parameters used to profile the spinning body rotation are updated in this statement
    if ((this->spinningBodyInMsg.timeWritten() <= callTime) && (this->convergence || target_change_exceeds_threshold))
    {
        // Update the initial time as the current simulation time
        this->tInit = callTime * NANO2SEC;

        // Update the initial angle and rate
        this->thetaInit = this->theta;
        this->thetaDotInit = this->thetaDot;

        // Store the reference angle
        this->thetaRef = spinningBodyIn.theta;

        // Set the parameters required to profile the rotation
        if (this->thetaInit != this->thetaRef)
        {
            this->computeRotationParameters();
        }
        else
        {
            this->t_f = this->tInit;
        }

        // Set the convergence to false until the rotation is complete
        this->convergence = false;
    }

    // Compute the scalar rotational states at the current simulation time
    this->computeCurrentState(callTime * NANO2SEC);

    // Write the module output messages
    this->writeOutputMessages(callTime);
}

/*! Computes the time required to transition between two angular rates under constant acceleration
    @param theta_dot_initial [rad/s] Initial angular rate
    @param theta_dot_final [rad/s] Final angular rate
    @return double [seconds] Time required to transition between the two rates
*/
double PrescribedRotationWithMaxVelocity1DOF::computeTimeForTransitionBetweenRates(double theta_dot_initial, double theta_dot_final) const
{
    return fabs(theta_dot_final - theta_dot_initial) / this->thetaDDotMax;
}

/*! Computes angular displacement over a time interval assuming constant acceleration
    @param delta_t [s] Time interval over which to compute angular displacement
    @return double [rad] Angular displacement over the time interval
*/
double PrescribedRotationWithMaxVelocity1DOF::computeAngularDisplacementWithConstAccel(double delta_t) const
{
    return 0.5 * this->rotationDirection * this->thetaDDotMax * pow(delta_t, 2);
}

//! This intermediate method groups the calculation of rotation parameters into a single method.
void PrescribedRotationWithMaxVelocity1DOF::computeRotationParameters()
{
    this->deltaThetaTotal = this->thetaRef - this->thetaInit;
    this->rotationDirection = (this->deltaThetaTotal >= 0.0) ? 1.0 : -1.0;

    // Calculates the time required to accelerate from the initial angular rate to the maximum configured rate, then decelerate to the target final rate
    double dt_b1_temp = this->computeTimeForTransitionBetweenRates(this->thetaDotInit, this->rotationDirection * this->thetaDotMax);
    double dt_b2_temp = this->computeTimeForTransitionBetweenRates(this->rotationDirection * this->thetaDotMax, this->thetaDotRef);

    // Angular displacement during acceleration/deceleration phases, assuming we go to the maximum configured angular rate
    double deltaTheta_b1_temp = this->computeAngularDisplacementWithConstAccel(dt_b1_temp);
    double deltaTheta_b2_temp = this->computeAngularDisplacementWithConstAccel(dt_b2_temp);

    // is the total angular displacement required to accelerate to and decelerate
    // from the maximum angular velocity less than the target total angular displacement?
    // If yes then we use a bang-coast-bang profile, otherwise we use a bang-bang profile
    if (abs(deltaTheta_b1_temp + deltaTheta_b2_temp) < abs(this->deltaThetaTotal))
    {
        // Storing these parameters as we determine we can reach the maximum configured angular rate
        this->dt_b1 = dt_b1_temp;
        this->dt_b2 = dt_b2_temp;
        this->deltaTheta_b1 = deltaTheta_b1_temp;
        this->deltaTheta_b2 = deltaTheta_b2_temp;

        this->useCoastSegment = true;
        this->computeBangCoastBangParameters();
    }
    else
    {
        this->useCoastSegment = false;

        this->computeBangBangParameters();
    }
}

/*! Computes the peak angular rate that can be achieved given initial and final rates,
    total angle to rotate, and max acceleration. Assumes no coast phase.

    This is derived by summing the angular displacement during two constant-acceleration phases
    (acceleration from thetaDotInit to thetaDot_peak and deceleration from thetaDot_peak to thetaDotRef)
    and solving the resulting equation for thetaDot_peak.

    @param deltaThetaTotal [rad] Total angular displacement to rotate
    @param thetaDotInit [rad/s] Initial angular rate
    @param thetaDotRef [rad/s] Final angular rate
    @param thetaDDotMax [rad/s^2] Maximum angular acceleration
    @return double [rad/s] Peak angular rate that can be achieved during the maneuver
*/
double PrescribedRotationWithMaxVelocity1DOF::computeReachablePeakAngularRate(double deltaThetaTotal, double thetaDotInit, double thetaDotRef, double thetaDDotMax) const
{
    return sqrt(thetaDDotMax * fabs(deltaThetaTotal) + 0.5 * (pow(thetaDotInit, 2) + pow(thetaDotRef, 2)));
}

//! This method computes the required parameters for the rotation with a non-smoothed bang-bang acceleration profile.
void PrescribedRotationWithMaxVelocity1DOF::computeBangBangParameters()
{
    // Compute the reachable peak angular rate (less than thetaDotMax)
    this->thetaDot_peak = this->computeReachablePeakAngularRate(this->deltaThetaTotal, this->thetaDotInit, this->thetaDotRef, this->thetaDDotMax);

    // Time to accelerate/decelerate to/from thetaDot_peak (reachable peak angular rate)
    this->dt_b1 = this->computeTimeForTransitionBetweenRates(this->thetaDotInit, this->rotationDirection * thetaDot_peak);
    this->dt_b2 = this->computeTimeForTransitionBetweenRates(this->rotationDirection * thetaDot_peak, this->thetaDotRef);

    // Angular displacement during the first bang segment
    this->deltaTheta_b1 = this->computeAngularDisplacementWithConstAccel(this->dt_b1);

    // Time and angular position at the end of the first bang segment
    this->t_b1_final = this->tInit + this->dt_b1;
    this->theta_b1_final = this->thetaInit + this->deltaTheta_b1;

    // Time and angular position at the start of the second bang segment
    this->t_b2_init = this->t_b1_final;
    this->theta_b2_init = this->theta_b1_final;

    // Determine the time when the rotation is complete t_f
    this->t_f = this->tInit + this->dt_b1 + this->dt_b2;
}

//! This method computes the required parameters for the rotation with a non-smoothed bang-coast-bang acceleration profile.
void PrescribedRotationWithMaxVelocity1DOF::computeBangCoastBangParameters()
{
    // Compute the reachable peak angular rate
    this->thetaDot_peak = this->thetaDotMax;

    // Time and angular position at the end of the first bang segment
    this->t_b1_final = this->tInit + this->dt_b1;
    this->theta_b1_final = this->thetaInit + this->deltaTheta_b1;

    // Time duration and angular displacement during the coast period
    double deltaTheta_coast = this->deltaThetaTotal - this->deltaTheta_b1 - this->deltaTheta_b2;
    double dt_coast = fabs(deltaTheta_coast / this->thetaDotMax);

    // Time and angular position at the start of the second bang segment
    this->t_b2_init = this->tInit + this->dt_b1 + dt_coast;
    this->theta_b2_init = this->thetaInit + this->deltaTheta_b1 + deltaTheta_coast;

    // Determine the time when the rotation is complete t_f
    this->t_f = this->tInit + this->dt_b1 + dt_coast + this->dt_b2;
}

//! This intermediate method groups the calculation of the current rotational states into a single method.
void PrescribedRotationWithMaxVelocity1DOF::computeCurrentState(double t)
{
    if (this->isInFirstBangSegment(t))
    {
        this->computeFirstBangSegment(t);
    }
    else if (this->useCoastSegment && this->isInCoastSegment(t))
    {
        this->computeCoastSegment(t);
    }
    else if (this->isInSecondBangSegment(t))
    {
        this->computeSecondBangSegment(t);
    }
    else
    {
        this->computeRotationComplete();
    }
}

/*! This method determines if the current time is within the first bang segment.
    @param t [s] Current simulation time
    @return bool
*/
bool PrescribedRotationWithMaxVelocity1DOF::isInFirstBangSegment(double t) const
{
    // is current time before end of first bang segment AND is total maneuver duration non-zero?
    return (t <= this->t_b1_final && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the second bang segment.
    @param t [s] Current simulation time
    @return bool
*/
bool PrescribedRotationWithMaxVelocity1DOF::isInSecondBangSegment(double t) const
{
    // is current time after start of second bang segment AND before end of maneuver AND is total maneuver duration non-zero?
    return (t > this->t_b2_init && t <= this->t_f && this->t_f - this->tInit != 0.0);
}

/*! This method determines if the current time is within the coast segment.
    @param t [s] Current simulation time
    @return bool
*/
bool PrescribedRotationWithMaxVelocity1DOF::isInCoastSegment(double t) const
{
    // is current time after first bang ends and before coast ends and is total maneuver duration non-zero?
    return (t > this->t_b1_final && t <= this->t_b2_init && this->t_f - this->tInit != 0.0);
}

/*! Computes angular position and rate at time `t` assuming constant acceleration.
    Derived from standard kinematic equations for constant acceleration.
    @param t Current time
    @param thetaDDot Constant angular acceleration
    @param t0 Initial time
    @param theta0 Initial angular position
    @param thetaDot0 Initial angular velocity
*/
void PrescribedRotationWithMaxVelocity1DOF::computeAngularKinematics(
    double t, double thetaDDot, double t_0, double theta_0, double thetaDot_0)
{
    double dt = t - t_0;
    this->thetaDDot = thetaDDot;
    this->thetaDot = thetaDDot * dt + thetaDot_0;
    this->theta = 0.5 * thetaDDot * dt * dt + thetaDot_0 * dt + theta_0;
}

/*! This method computes the scalar rotational states for the first bang segment.
    @param t [s] Current simulation time
*/
void PrescribedRotationWithMaxVelocity1DOF::computeFirstBangSegment(double t)
{
    double t0 = this->tInit;
    double theta0 = this->thetaInit;
    double thetaDot0 = this->thetaDotInit;

    double thetaDDot = this->rotationDirection * this->thetaDDotMax;

    computeAngularKinematics(t, thetaDDot, t0, theta0, thetaDot0);
}

/*! This method computes the scalar rotational states for the second bang segment.
    @param t [s] Current simulation time
*/
void PrescribedRotationWithMaxVelocity1DOF::computeSecondBangSegment(double t)
{
    double t0 = this->t_b2_init;
    double theta0 = this->theta_b2_init;
    double thetaDot0 = this->rotationDirection * this->thetaDot_peak;

    double thetaDDot = -this->rotationDirection * this->thetaDDotMax;

    computeAngularKinematics(t, thetaDDot, t0, theta0, thetaDot0);
}

/*! This method computes the coast segment scalar rotational states
    @param t [s] Current simulation time
*/
void PrescribedRotationWithMaxVelocity1DOF::computeCoastSegment(double t)
{
    double t0 = this->t_b1_final;
    double theta0 = this->theta_b1_final;
    double thetaDot0 = this->rotationDirection * this->thetaDotMax;

    double thetaDDot = 0.0;

    computeAngularKinematics(t, thetaDDot, t0, theta0, thetaDot0);
}

//! This method computes the scalar rotational states when the rotation is complete.
void PrescribedRotationWithMaxVelocity1DOF::computeRotationComplete()
{
    this->thetaDDot = 0.0;
    this->thetaDot = this->thetaDotRef;
    this->theta = this->thetaRef;
    this->convergence = true;
}

//! This method writes the module output messages and computes the output message data.
void PrescribedRotationWithMaxVelocity1DOF::writeOutputMessages(uint64_t callTime)
{
    // Create the output buffer messages
    HingedRigidBodyMsgPayload spinningBodyOut;
    PrescribedRotationMsgPayload prescribedRotationOut;

    // Zero the output messages
    spinningBodyOut = HingedRigidBodyMsgPayload();
    prescribedRotationOut = PrescribedRotationMsgPayload();

    // Compute the angular velocity of frame P wrt frame M in P frame components
    Eigen::Vector3d omega_PM_P = this->thetaDot * this->rotHat_M; // [rad/s]

    // Compute the B frame time derivative of omega_PM_P in P frame components
    Eigen::Vector3d omegaPrime_PM_P = this->thetaDDot * this->rotHat_M; // [rad/s^2]

    // Compute the MRP attitude of spinning body frame P with respect to frame M
    Eigen::Vector3d sigma_PM = this->computeSigma_PM();

    // Copy the module variables to the output buffer messages
    spinningBodyOut.theta = this->theta;
    spinningBodyOut.thetaDot = this->thetaDot;
    eigenVector3d2CArray(omega_PM_P, prescribedRotationOut.omega_PM_P);
    eigenVector3d2CArray(omegaPrime_PM_P, prescribedRotationOut.omegaPrime_PM_P);
    eigenVector3d2CArray(sigma_PM, prescribedRotationOut.sigma_PM);

    // Write the output messages
    this->spinningBodyOutMsg.write(&spinningBodyOut, moduleID, callTime);
    this->prescribedRotationOutMsg.write(&prescribedRotationOut, moduleID, callTime);
    HingedRigidBodyMsg_C_write(&spinningBodyOut, &spinningBodyOutMsgC, this->moduleID, callTime);
    PrescribedRotationMsg_C_write(&prescribedRotationOut, &prescribedRotationOutMsgC, this->moduleID, callTime);
}

//! This method computes the current spinning body MRP attitude relative to the mount frame: sigma_PM
Eigen::Vector3d PrescribedRotationWithMaxVelocity1DOF::computeSigma_PM()
{
    // Determine dcm_PP0 for the current spinning body attitude relative to the initial attitude
    double dcm_PP0[3][3];
    double prv_PP0_array[3];
    double theta_PP0 = this->theta - this->thetaInit;
    Eigen::Vector3d prv_PP0 = theta_PP0 * this->rotHat_M;
    eigenVector3d2CArray(prv_PP0, prv_PP0_array);
    PRV2C(prv_PP0_array, dcm_PP0);

    // Determine dcm_P0M for the initial spinning body attitude relative to the mount frame
    double dcm_P0M[3][3];
    double prv_P0M_array[3];
    Eigen::Vector3d prv_P0M = this->thetaInit * this->rotHat_M;
    eigenVector3d2CArray(prv_P0M, prv_P0M_array);
    PRV2C(prv_P0M_array, dcm_P0M);

    // Determine dcm_PM for the current spinning body attitude relative to the mount frame
    double dcm_PM[3][3];
    m33MultM33(dcm_PP0, dcm_P0M, dcm_PM);

    // Compute the MRP sigma_PM representing the current spinning body attitude relative to the mount frame
    double sigma_PM_array[3];
    C2MRP(dcm_PM, sigma_PM_array);
    return cArray2EigenVector3d(sigma_PM_array);
}

/*! Setter method for the spinning body rotation axis.
    @param rotHat_M Spinning body rotation axis (unit vector)
*/
void PrescribedRotationWithMaxVelocity1DOF::setRotHat_M(const Eigen::Vector3d &rotHat_M)
{
    this->rotHat_M = rotHat_M;
}

/*! Setter method for the max angular rate.
    @param thetaDotMax [rad/s] Max angular rate
*/
void PrescribedRotationWithMaxVelocity1DOF::setThetaDotMax(const double thetaDotMax)
{
    this->thetaDotMax = thetaDotMax;
}

/*! Setter method for the bang segment scalar angular acceleration.
    @param thetaDDotMax [rad/s^2] Bang segment scalar angular acceleration
*/
void PrescribedRotationWithMaxVelocity1DOF::setThetaDDotMax(const double thetaDDotMax)
{
    this->thetaDDotMax = thetaDDotMax;
}

/*! Setter method for the initial spinning body angle.
    @param thetaInit [rad] Initial spinning body angle
*/
void PrescribedRotationWithMaxVelocity1DOF::setThetaInit(const double thetaInit)
{
    this->thetaInit = thetaInit;
    this->theta = thetaInit;
    this->thetaRef = thetaInit;
}

/*! Setter method for the initial spinning body rate.
    @param thetaInit [rad] Initial spinning body rate
*/
void PrescribedRotationWithMaxVelocity1DOF::setThetaDotInit(const double thetaDotInit)
{
    this->thetaDotInit = thetaDotInit;
    this->thetaDot = thetaDotInit;
}

/*! Getter method for the spinning body rotation axis.
    @return const Eigen::Vector3d
*/
const Eigen::Vector3d &PrescribedRotationWithMaxVelocity1DOF::getRotHat_M() const
{
    return this->rotHat_M;
}

/*! Getter method for the max angular rate.
    @return double
*/
double PrescribedRotationWithMaxVelocity1DOF::getThetaDotMax() const
{
    return this->thetaDotMax;
}

/*! Getter method for the ramp segment scalar angular acceleration.
    @return double
*/
double PrescribedRotationWithMaxVelocity1DOF::getThetaDDotMax() const
{
    return this->thetaDDotMax;
}

/*! Getter method for the initial spinning body angle.
    @return double
*/
double PrescribedRotationWithMaxVelocity1DOF::getThetaInit() const
{
    return this->thetaInit;
}

/*! Setter method for the angular threshold to calculate a new target
    @param newTargetAngularThreshold [rad]
*/
void PrescribedRotationWithMaxVelocity1DOF::setNewTargetAngularThreshold(const double newTargetAngularThreshold)
{
    this->newTargetAngularThreshold = newTargetAngularThreshold;
}
