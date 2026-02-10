#ifndef ATT_CONTROLLER_H
#define ATT_CONTROLLER_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AttGuidMsgPayload.h"
#include "architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"

#include <string>
#include "include/cpptoml.h"

class AttitudeController: public SysModel {
    public:
        AttitudeController(const std::string& config_path, bool extra_jitter = false);
        ~AttitudeController() {};
        void Reset(uint64_t current_sim_nanos) {};
        void UpdateState(uint64_t current_sim_nanos);

    public:
        // Input messages
        ReadFunctor<AttGuidMsgPayload> guideInMsg;
        // Output messages
        Message<CmdTorqueBodyMsgPayload> cmdTorqueOutMsg;
        Message<ArrayMotorTorqueMsgPayload> cmdRWTorqueOutMsg;
    private:
        double pid_p;
        double pid_d;
};

#endif
