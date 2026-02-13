## A few ground rules for custom messages:
- Custom messages MUST follow the naming convention: `NAMEHEREMsgPayload.h`
- Custom messages MUST follow the structure: 

```c
#include <stdint.h>

#ifndef _NAV_LINEAR_STATE_MESSAGE_H
#define _NAV_LINEAR_STATE_MESSAGE_H

typedef struct {
    double los_sun_n[3];
    double ast_body_pos_n[3];
    double ast_body_vel_n[3];
    bool ast_locked;
}NavLinearStateMsgPayload;

#endif
```
