#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "main.h"

#define CHASSIS_SPEED_ATTENUATION   (1.0f)
#define PREPARE_TIME_TICK_MS 4000      //prapare time in ms
#define POKE_PREPARE_TIME_TICK_MS  1000

typedef struct
{
  int32_t Cnt_Power_Judge_Recieved;
  int32_t Time_10ms;
  int32_t Cnt_Power_Judge_Recieved_Pre;
  u8 			Flag_Judge_Control;
  float 	K_Output;
} Power_Control_Struct;

#define POWER_CONTROL_DEFAULT \
{	0,\
	0,\
	0,\
	0,\
}\

extern RampGen_t GMPitchRamp;
extern RampGen_t GMYawRamp;
extern RampGen_t GMPokeRamp;
extern Power_Control_Struct Power_Control;
extern uint32_t time_tick_1ms;
extern int now_count;
extern int last_count;
extern int now_current;
extern int last_current;
void Control_Task(void);
void ControtLoopTaskInit(void);


#endif


