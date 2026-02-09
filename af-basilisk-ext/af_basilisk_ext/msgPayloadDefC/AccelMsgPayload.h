#include <stdint.h>

#ifndef _ACCEL_MESSAGE_H
#define _ACCEL_MESSAGE_H

/*! @brief Structure used to define accelerometer data */
typedef struct {
    double accel_AN_A[3]; // Acceleration of accelerometer relative to inertial frame, expressed in accelerometer frame components
}AccelMsgPayload;

#endif
