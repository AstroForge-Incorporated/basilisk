#include <stdint.h>

#ifndef _MAN_EXECUTOR_STATE_MESSAGE_H
#define _MAN_EXECUTOR_STATE_MESSAGE_H

enum ManeuverStatus {
    MANEUVER_NOT_STARTED,
    MANEUVER_EXECUTING,
    SLEW_COMPLETE,
    DV_COMPLETE,
};

/*! @brief Structure used to define Maneuver Executor state */
typedef struct {
    enum ManeuverStatus maneuver_status;
}ManExecutorStateMsgPayload;

#endif
