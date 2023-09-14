#ifndef __BARRIERS_TASK_H__
#define __BARRIERS_TASK_H__
#include "main.h"

#define barriers_ecd_angle 11350

typedef enum
{
	BARRIERS_INIT         = 0,
	BARRIERS_RETURN       = 1,
  BARRIERS_UP           = 2,
	BARRIERS_DOWN         = 3,
  BARRIERS_KEY_UP       = 4,
	BARRIERS_KEY_DOWN     = 5,	
	BARRIERS_STOP         = 6,

} barriers_mode_e;

extern barriers_mode_e barriers_mode;
extern u8 barriers_zero_flag;
extern float Remove_Barrier_set_angle[2];
extern float Remove_Barrier_start_angle[2];
extern float Remove_Barrier_end_angle[2];


void barriers_task(void);
void return_zero(void);
void barriers_moving_calc(void);
void barriers_param_init(void);
#endif