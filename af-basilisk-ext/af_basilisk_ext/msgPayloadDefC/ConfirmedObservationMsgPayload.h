#include <stdint.h>

#ifndef _CONFIRMED_OBSERVATION_MESSAGE_H
#define _CONFIRMED_OBSERVATION_MESSAGE_H

typedef struct {
    double los_c[3];
    int64_t capture_time_ns;
}ConfirmedObservationMsgPayload;

#endif
