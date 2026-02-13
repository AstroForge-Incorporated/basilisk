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

#ifndef _PRESCRIBEDROTATIONWITHMAXVELOCITY1DOF_
#define _PRESCRIBEDROTATIONWITHMAXVELOCITY1DOF_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Prescribed 1 DOF Rotation Profiler Class */
class PrescribedRotationWithMaxVelocity1DOF : public SysModel
{
public:
    PrescribedRotationWithMaxVelocity1DOF() = default;  //!< Constructor
    ~PrescribedRotationWithMaxVelocity1DOF() = default; //!< Destructor

    void SelfInit() override;                                                  //!< Member function to initialize the C-wrapped output message
    void Reset(uint64_t CurrentSimNanos) override;                             //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                       //!< Update member function
    void setRotHat_M(const Eigen::Vector3d &rotHat_M);                         //!< Setter for the spinning body rotation axis
    void setThetaDotMax(const double thetaDotMax);                             //!< Setter for the max angular rate
    void setThetaDDotMax(const double thetaDDotMax);                           //!< Setter for the bang segment scalar angular acceleration
    void setThetaInit(const double thetaInit);                                 //!< Setter for the initial spinning body angle
    void setThetaDotInit(const double thetaDotInit);                           //!< Setter for the initial spinning body rate
    void setNewTargetAngularThreshold(const double newTargetAngularThreshold); //!< Setter for the minimum angular difference required to trigger a new rotation target update
    const Eigen::Vector3d &getRotHat_M() const;                                //!< Getter for the spinning body rotation axis
    double getThetaDotMax() const;                                             //!< Getter for the max angular rate
    double getThetaDDotMax() const;                                            //!< Getter for the bang segment scalar angular acceleration
    double getThetaInit() const;                                               //!< Getter for the initial spinning body angle

    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyInMsg;       //!< Input msg for the spinning body reference angle and angle rate
    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;          //!< Output msg for the spinning body angle and angle rate
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg; //!< Output msg for the spinning body prescribed rotational states
    HingedRigidBodyMsg_C spinningBodyOutMsgC = {};                  //!< C-wrapped output msg for the spinning body angle and angle rate
    PrescribedRotationMsg_C prescribedRotationOutMsgC = {};         //!< C-wrapped output msg for the spinning body prescribed rotational states

    BSKLogger *bskLogger; //!< BSK Logging

private:
    /* Methods for computing the required rotational parameters */
    void computeRotationParameters();                                                                                                   //!< Intermediate method to group the calculation of rotation parameters into a single method
    void computeBangBangParameters();                                                                                                   //!< Method for computing the required parameters for the non-smoothed bang-bang profiler option
    void computeBangCoastBangParameters();                                                                                              //!< Method for computing the required parameters for the non-smoothed bang-coast-bang profiler option
    double computeTimeForTransitionBetweenRates(double theta_dot_initial, double theta_dot_final) const;                                //!< Method for computing time to transition between angular rates with constant acceleration
    double computeAngularDisplacementWithConstAccel(double delta_t) const;                                                              //!< Method for computing angular displacement over time with constant acceleration
    double computeReachablePeakAngularRate(double deltaThetaTotal, double thetaDotInit, double thetaDotRef, double thetaDDotMax) const; //!< Method for computing the reachable peak angular rate during the maneuver
    void computeAngularKinematics(double t, double thetaDDot, double t_0, double theta_0, double thetaDot_0);                           //!< Computing the angular kinematics of the body

    /* Methods for computing the current rotational states */
    void computeCurrentState(double time);         //!< Intermediate method used to group the calculation of the current rotational states into a single method
    bool isInFirstBangSegment(double time) const;  //!< Method for determining if the current time is within the first bang segment
    bool isInSecondBangSegment(double time) const; //!< Method for determining if the current time is within the second bang segment
    bool isInCoastSegment(double time) const;      //!< Method for determining if the current time is within the coast segment
    void computeFirstBangSegment(double time);     //!< Method for computing the first bang segment scalar rotational states
    void computeSecondBangSegment(double time);    //!< Method for computing the second bang segment scalar rotational states
    void computeCoastSegment(double time);         //!< Method for computing the coast segment scalar rotational states
    void computeRotationComplete();                //!< Method for computing the scalar rotational states when the rotation is complete

    void writeOutputMessages(uint64_t CurrentSimNanos); //!< Method for writing the module output messages and computing the output message data
    Eigen::Vector3d computeSigma_PM();                  //!< Method for computing the current spinning body MRP attitude relative to the mount frame: sigma_PM

    /* User-configurable variables (set in Python by setter functions) */
    double thetaDotMax;               //!< [rad/s] Maximum angular rate
    double thetaDDotMax;              //!< [rad/s^2] Maximum angular acceleration of spinning body used in the bang segments
    double newTargetAngularThreshold; //!< [rad] Minimum angular difference required to trigger a new rotation target update
    Eigen::Vector3d rotHat_M;         //!< Spinning body rotation axis in M frame components

    /* User-configurable variables INITIALLY (set in Python by setter functions), then get overwritten */
    double thetaInit;    //!< [rad] Initial spinning body angle from frame M to frame P about rotHat_M
    double thetaDotInit; //!< [rad/s] Initial spinning body rate from frame M to frame P about rotHat_M

    /* Scalar rotational states (set in Python by setter functions) */
    double theta;    //!< [rad] Current angle. Initial value set by thetaInit in Python
    double thetaDot; //!< [rad/s] Current angle rate. Initial value set by thetaDotInit in Python
    double thetaRef; //!< [rad] Spinning body reference angle from frame M to frame P about rotHat_M. Initial value set by thetaInit in Python

    /* Scalar rotational states */
    double thetaDDot = 0.0;       //!< [rad/s^2] Current angular acceleration
    double thetaDotRef = 0.0;     //!< [rad] Spinning body reference rate from frame M to frame P about rotHat_M
    double deltaTheta_b1 = 0.0;   //!< [rad] Angular displacement during first bang segment
    double deltaTheta_b2 = 0.0;   //!< [rad] Angular displacement during second bang segment
    double deltaThetaTotal = 0.0; //!< [rad] Total angular displacement for planned motion
    double theta_b1_final = 0.0;  //!< [rad] Angle at the end of the first bang segment
    double theta_b2_init = 0.0;   //!< [rad] Angle at the start of the second bang segment
    double thetaDot_peak = 0.0;   //!< [rad/s] Reachable peak angular rate during maneuver, could be less than or equal to thetaDotMax

    /* Temporal parameters */
    double tInit = 0.0;      //!< [s] Simulation time at the beginning of the rotation
    double t_f = 0.0;        //!< [s] Simulation time when the rotation is complete
    double dt_b1 = 0.0;      //!< [s] Duration of first bang segment
    double dt_b2 = 0.0;      //!< [s] Duration of second bang segment
    double t_b1_final = 0.0; //!< [s] Simulation time at the end of the first bang segment
    double t_b2_init = 0.0;  //!< [s] Simulation time at the start of the second bang segment

    bool convergence = true;        //!< Boolean variable is true when the rotation is complete
    double rotationDirection = 1.0; //!< Direction of rotation (+1 or -1)
    bool useCoastSegment = false;   //!< [bool] True if coast segment is used in profile
};

#endif
