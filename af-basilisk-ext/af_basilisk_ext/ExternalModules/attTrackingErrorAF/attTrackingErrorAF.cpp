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

#include <iostream>
#include <cmath>
#include <string.h>
#include "attTrackingErrorAF.h"
#include "fswAlgorithms/fswUtilities/fswDefinitions.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"

void AttitudeTrackingError::Reset(uint64_t current_sim_nanos) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: attTrackingError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "Error: attTrackingError.attNavInMsg wasn't connected.");
    }
    return;
}

void AttitudeTrackingError::UpdateState(uint64_t current_sim_nanos)
{
    // Only run if both input messages have been written to
    if (this->attRefInMsg.isWritten() && this->attNavInMsg.isWritten()) {
        /*! - Read the input messages */
        AttRefMsgPayload ref;
        ref = this->attRefInMsg();
        if (this->containsNaN(ref.sigma_RN)) {
            std::cout << "Ref has NaN";
        }
        // std::cout << "Ref: [" << ref.sigma_RN[0] << ", " << ref.sigma_RN[1] << ", " << ref.sigma_RN[2] << "] \n";
        NavAttMsgPayload nav;  
        nav = this->attNavInMsg();
        // std::cout << "Nav: [" << nav.sigma_BN[0] << ", " << nav.sigma_BN[1] << ", " << nav.sigma_BN[2] << "] \n";
        if (this->containsNaN(nav.sigma_BN)) {
            std::cout << "Nav has NaN";
        }

        /*! - Compute attitude error */
        AttGuidMsgPayload attGuidOut;
        this->computeAttitudeError(this->sigma_R0R, nav, ref, &attGuidOut);
        // std::cout << "Guide: [" << attGuidOut.sigma_BR[0] << ", " << attGuidOut.sigma_BR[1] << ", " << attGuidOut.sigma_BR[2] << "] \n";
        if (this->containsNaN(attGuidOut.sigma_BR)) {
            std::cout << "Guide has NaN";
        }

        /*! - Write out attitude error message */
        this->attGuidOutMsg.write(&attGuidOut, this->moduleID, current_sim_nanos);
    }
}

void AttitudeTrackingError::computeAttitudeError(double sigma_R0R[3], NavAttMsgPayload nav, 
                                                    AttRefMsgPayload ref, AttGuidMsgPayload *attGuidOut){
    double      sigma_RR0[3];               /* MRP from the original reference frame R0 to the corrected reference frame R */
    double      sigma_RN[3];                /* MRP from inertial to updated reference frame */
    double      dcm_BN[3][3];               /* DCM from inertial to body frame */

    /*! - compute the initial reference frame orientation that takes the corrected body frame into account */
    v3Scale(-1.0, sigma_R0R, sigma_RR0);
    addMRP(ref.sigma_RN, sigma_RR0, sigma_RN);

    subMRP(nav.sigma_BN, sigma_RN, attGuidOut->sigma_BR);               /*! - compute attitude error */

    MRP2C(nav.sigma_BN, dcm_BN);                                /* [BN] */
    m33MultV3(dcm_BN, ref.omega_RN_N, attGuidOut->omega_RN_B);              /*! - compute reference omega in body frame components */

    v3Subtract(nav.omega_BN_B, attGuidOut->omega_RN_B, attGuidOut->omega_BR_B);     /*! - delta_omega = omega_B - [BR].omega.r */

    m33MultV3(dcm_BN, ref.domega_RN_N, attGuidOut->domega_RN_B);            /*! - compute reference d(omega)/dt in body frame components */
}

bool AttitudeTrackingError::containsNaN(const double array[3]) {
   for (int i = 0; i < 3; i++) {
        if (std::isnan(array[i])) {
            return true;
        }
    }
    return false;
}
