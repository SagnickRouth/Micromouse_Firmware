#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdbool.h>

void speed_control_init(void);
void speed_control_update(float left_target_mmps, float right_target_mmps);
void speed_control_stop(void);
bool speed_control_is_active(void);

#endif
