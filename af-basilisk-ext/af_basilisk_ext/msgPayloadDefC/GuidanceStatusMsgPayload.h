#include <stdint.h>

#ifndef _GUIDANCE_RESP_MESSAGE_H
#define _GUIDANCE_RESP_MESSAGE_H

enum Status {
    GUIDANCE_MANEUVER_REQUESTED,
    GUIDANCE_NO_MANEUVER_REQUESTED,
};

/*! @brief Structure used to define guidance status */
typedef struct {
    enum Status status; // Guidance status
}GuidanceStatusMsgPayload;

#endif
