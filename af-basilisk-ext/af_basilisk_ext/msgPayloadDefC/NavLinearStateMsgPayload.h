#include <stdint.h>

#ifndef _NAV_LINEAR_STATE_MESSAGE_H
#define _NAV_LINEAR_STATE_MESSAGE_H

typedef struct {
    double los_sun_n[3];
    double ast_body_pos_n[3];
    double ast_body_vel_n[3];
    int8_t ast_locked;
    int64_t current_time_ns;
}NavLinearStateMsgPayload;

#endif
