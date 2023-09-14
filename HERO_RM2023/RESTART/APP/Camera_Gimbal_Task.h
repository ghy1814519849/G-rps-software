#ifndef __CAMERA_GIMBAL_TASK_H
#define __CAMERA_GIMBAL_TASK_H

#include "main.h"

typedef enum
{
	RELAX   		=   0,
	INIT    		=   1,
	NORMAL			=		2,
	AUTO_ANGLE	=		3,
}camera_gimbal_mode_t;



extern camera_gimbal_mode_t camera_gimbal_mode;
extern int aim_scope_flag;
extern int aim_scope_change_flag;
extern int aim_scope_time;
extern float scope_angle;
extern int aim_init_flag;
extern float cam_angle;
extern float lcam_angle;
extern float lcam_angle_err;

extern void aim_scope_init(void);
extern void aim_scope_control(void);
extern void camera_gimbal_control(void);
extern void cam_gim_normal(void);
extern void cam_gim_init(void);
extern void cam_gim_stop(void);
extern void cam_gim_param_init(void);
extern void cam_gim_auto_angle(void);
extern pid_t aim_scope_speed;
#endif
