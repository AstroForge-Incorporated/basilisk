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

#ifndef FINE_SUN_SENSOR_H
#define FINE_SUN_SENSOR_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "../../msgPayloadDefCpp/FSSConfigLogMsgPayload.h"
#include "../../msgPayloadDefC/FSSRawDataMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include <random>

/*! @brief fine sun sensor class */
class FineSunSensor : public SysModel
{
public:
    FineSunSensor();
    ~FineSunSensor();

    void Reset(uint64_t CurrentClock);              //!< Method for reseting the module
    void UpdateState(uint64_t CurrentSimNanos);     //!< @brief method to update state for runtime
    void readInputMessages();                       //!< @brief method to read the input messages
    void computeSunData();                          //!< @brief method to get the sun vector information
    void computeTrueOutput();                       //!< @brief method to compute the true sun-fraction of FSS
    void applySensorErrors();                       //!< @brief method to set the actual output of the sensor with noise
    void writeOutputMessages(uint64_t Clock);       //!< @brief method to write the output message to the system
    void setRNGSeed(uint32_t seed);                 //!< @brief method to set the random number generator seed
    void setSensorNoiseBound(double senNoiseBound); //!< @brief method to set the sensor noise bound

public:
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;   //!< [-] input message for sun data
    ReadFunctor<SCStatesMsgPayload> stateInMsg;         //!< [-] input message for spacecraft state
    Message<FSSRawDataMsgPayload> fssDataOutMsg;        //!< [-] output message for FSS output data
    Message<FSSConfigLogMsgPayload> fssConfigLogOutMsg; //!< [-] output message for FSS configuration log data

    Eigen::Vector3d r_B;           //!< [m] position vector in body frame
    Eigen::Vector3d nHat_B;        //!< [-] fss unit direction vector in body frame components
    Eigen::Vector3d sHat_B;        //!< [-] unit vector to sun in B
    double trueValue;              //!< [-] solar irradiance measurement without perturbations
    double fov;                    //!< [-] rad, square field of view half-angle. The FSS boresight is aligned with the +Z axis of the sensor frame. The FOV extends symmetrically from the boresight along the X and Y axes of the sensor frame.
    double senNoiseBound;          //!< Sensor noise standard deviation vector
    int FSSGroupID = -1;           //!< [-] (optional) FSS group id identifier, -1 means it is not set and default is used
    BSKLogger bskLogger;           //!< -- BSK Logging
    Eigen::Vector3d sHat_M_true;   //!< [-] true unit vector to sun in misaligned sensor frame M
    Eigen::Vector3d sHat_M_sensed; //!< [-] sensed (corrupted) unit vector to sun in misaligned sensor frame M
    Eigen::Matrix3d dcm_cf2b;      //!< [-] rotation matrix of component frame cf to body frame B

private:
    bool RNGSeed_initialization_flag = false;
    bool senNoiseBound_initialization_flag = false;

    using UniformDist = std::uniform_real_distribution<double>; // type alias as this is very wordy
    UniformDist dist_minus1_to_1;
    UniformDist dist_0_to_2pi;
    UniformDist dist_0_to_senNoiseBound;
    std::mt19937 rng; //!< Mersenne Twister random number generator

    SpicePlanetStateMsgPayload sunData; //!< [-] sun data
    SCStatesMsgPayload stateCurrent;    //!< [-] Current SSBI-relative state
};

#endif
