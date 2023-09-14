#ifndef __MODESWITCHTASK_H__
#define __MODESWITCHTASK_H__
#include "main.h"

void mode_switch_task(void);
static void get_gimbal_mode(void);
static void get_chassis_mode(void);
uint8_t gimbal_is_controllable(void);
uint8_t chassis_is_controllable(void);
#endif

