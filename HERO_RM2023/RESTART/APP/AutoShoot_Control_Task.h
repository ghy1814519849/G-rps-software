#ifndef _AUTO_SHOOT_CONTROL_TASK_H_
#define _AUTO_SHOOT_CONTROL_TASK_H_
#include "main.h"

typedef struct
{
  float x;
  float y;
  float dis;
  uint8_t flag;
  float pitch_speed;
	float yaw_speed;
  uint8_t shot_flag;                    //Éä»÷±êÖ¾Î»
} location;

typedef enum
{
  unkown = 0,
  blue = 1,
  red  = 2,
} robot_color_e;

extern robot_color_e robot_color ;
extern location new_location;

void targetOffsetDataDeal(uint8_t  len, u8 *buf);
void process_general_message(unsigned char* address, unsigned int length);
#endif

