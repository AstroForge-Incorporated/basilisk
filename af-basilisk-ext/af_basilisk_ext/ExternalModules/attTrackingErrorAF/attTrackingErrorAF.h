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

#ifndef _AF_ATT_TRACKING_ERROR_
#define _AF_ATT_TRACKING_ERROR_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/AttRefMsgPayload.h"

#include <stdint.h>


class AttitudeTrackingError : public SysModel {
    public:
        AttitudeTrackingError() {};
        ~AttitudeTrackingError() {};

        void Reset(uint64_t current_sim_nanos);
        void UpdateState(uint64_t current_sim_nanos);
    public:
        // Input messages
        ReadFunctor<NavAttMsgPayload> attNavInMsg;
        ReadFunctor<AttRefMsgPayload> attRefInMsg;

        // Output messages
        Message<AttGuidMsgPayload> attGuidOutMsg;

    private:
        void computeAttitudeError(double sigma_R0R[3], NavAttMsgPayload nav, 
                                AttRefMsgPayload ref, AttGuidMsgPayload *attGuidOut);

        bool containsNaN(const double array[3]);

    private:
        // Note: This is where the AstroForge fix came in – unlike in the Basilisk version of this code, we initialize this double.
        // Not initializing the double results in us reading garbage data.
        double sigma_R0R[3] = {0.0, 0.0, 0.0}; //!< MRP from corrected reference frame to original reference frame R0. This is the same as [BcB] going from primary body frame B to the corrected body frame Bc
        BSKLogger bskLogger; //!< -- BSK Logging
};

#endif
