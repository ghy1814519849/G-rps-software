#ifndef __AUTO_ANGLE_TASK_H_
#define __AUTO_ANGLE_TASK_H_
#include "main.h"

//#define gravity            9.7915
//#define image_muzzle_angle 15//trans angle to radian  ANGLE_TO_RAD
//#define projectile_speed   16

extern float shoot_angle;
extern float beta;
extern double image_muzzle_angle;
extern double projectile_speed;
extern float shot_radar_dis;
extern u8 shot_radar_flag;               //µı…‰≤‚æ‡±Í÷æŒª
extern u8 auto_angle_flag;
extern u32 auto_angle_key_flag;
extern u8 angle_shot_flag;
extern float avg_shot_speed;
extern u32 shot_speed_num;
extern u8 auto_angle_shot_flag;
extern u8 b_12_speed_flag;
extern float angle;
void chip_task(void);
void auto_angle_control(void);
float avg(float new_value);

extern pid_t auto_angle;
extern pid_t auto_angle_speed;
extern float angle_cal;

#endif

