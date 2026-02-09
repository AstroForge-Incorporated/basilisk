#ifndef DUMMY_ATT_DET_H
#define DUMMY_ATT_DET_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"

/// @brief Dummy Attitude Determination module, designed to take true Spacecraft attitude published
/// by the Spacecraft module, and publish corresponding Nav attitude messages
class DummyAttitudeDet: public SysModel {
    public:
        DummyAttitudeDet() {};
        ~DummyAttitudeDet() {};
        void Reset(uint64_t current_sim_nanos) {};
        void UpdateState(uint64_t current_sim_nanos);

    public:
        // Input messages
        ReadFunctor<SCStatesMsgPayload> spacecraftStatesInMsg;
        // Output messages
        Message<NavAttMsgPayload> navAttOutMsg;
};

#endif
