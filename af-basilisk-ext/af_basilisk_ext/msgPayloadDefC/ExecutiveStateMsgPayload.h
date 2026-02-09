#include <stdint.h>

#ifndef _EXECUTIVE_STATE_MESSAGE_H
#define _EXECUTIVE_STATE_MESSAGE_H


enum ExecState {
    EXECUTIVE_STANDBY,
    EXECUTIVE_TARGET_POINTING,
    EXECUTIVE_TARGET_IMG_STATE_EST,
    EXECUTIVE_HOMING,
    EXECUTIVE_TERMINAL,
    EXECUTIVE_POST_PROXIMITY,
};

/*! @brief Structure used to define Executive state, for debugging */
typedef struct {
    enum ExecState state;
}ExecutiveStateMsgPayload;

#endif
