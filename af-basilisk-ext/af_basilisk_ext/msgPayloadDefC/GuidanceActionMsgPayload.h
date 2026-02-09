#include <stdint.h>

#ifndef _GUIDANCE_ACTION_MESSAGE_H
#define _GUIDANCE_ACTION_MESSAGE_H

enum Action {
    GUIDANCE_DO_TARGET_POINTING,
    GUIDANCE_DO_HOMING,
    GUIDANCE_DO_TERMINAL,
    GUIDANCE_NONE, // Dummy NONE action, since BSK always sends a message at every tick. Use NULL
    // if no action is requested (this does not get sent to the FSW)
};

/*! @brief Structure used to define guidance action requests */
typedef struct {
    enum Action action; // Guidance action request
}GuidanceActionMsgPayload;

#endif
