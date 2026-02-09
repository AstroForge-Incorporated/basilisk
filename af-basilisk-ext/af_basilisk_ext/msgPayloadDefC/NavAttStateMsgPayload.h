#include <stdint.h>

#ifndef _NAV_ATT_STATE_MESSAGE_H
#define _NAV_ATT_STATE_MESSAGE_H

typedef struct {
    double body_att_n[4];
    double body_ang_vel_n_n[3];
    int8_t tumbling; // 1 if tumbling, else 0
}NavAttStateMsgPayload;

#endif
