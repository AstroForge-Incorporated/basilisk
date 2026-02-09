#include <stdint.h>

#ifndef _SIMLIB_STATUS_MESSAGE_H
#define _SIMLIB_STATUS_MESSAGE_H

typedef struct {
    int8_t err; // 1 if STATUS != 0; else 0
}SimLibStatusMsgPayload;

#endif
