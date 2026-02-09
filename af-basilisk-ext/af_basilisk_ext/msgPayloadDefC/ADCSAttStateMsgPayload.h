#include <stdint.h>

#ifndef _ADCS_ATT_STATE_MESSAGE_H
#define _ADCS_ATT_STATE_MESSAGE_H

typedef struct {
    double body_att_n[4];
    double body_ang_vel_n_n[3];
    double gyro_bias_b[3];
}ADCSAttStateMsgPayload;

#endif
