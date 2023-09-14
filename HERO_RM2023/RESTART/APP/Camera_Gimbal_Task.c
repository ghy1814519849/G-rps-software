#include "main.h"

pid_t camera_gim_angle = {0};
pid_t camera_gim_speed = {0};
pid_t aim_scope_angle = {0};
pid_t aim_scope_speed = {0};
pid_t aim_rotate      ={0};

camera_gimbal_mode_t camera_gimbal_mode;

int aim_scope_flag=0;
int aim_scope_change_flag=0;
int aim_scope_time=0;
int aim_init_flag=2;
int scope_set_time=0;

float scope_angle=0;
float k_angle=1;
float scope_position=0;
float init_angle = 0;

void aim_scope_control(void)
{
	switch(aim_init_flag)
	{
    case 0:
			{
				 aim_scope_init();
			}
			break;
		case 1:
			{
				if (aim_scope_flag==0)
					scope_angle = init_angle - 2;
				else if (aim_scope_flag==1)
					scope_angle = init_angle - 181;

				aim_scope_angle.set=scope_angle;
				aim_scope_angle.get=BUS1_CM8Encoder.ecd_angle;
				pid_calc(&aim_scope_angle,aim_scope_angle.get,aim_scope_angle.set);

				aim_scope_speed.set=aim_scope_angle.out;
				aim_scope_speed.get=BUS1_CM8Encoder.rate_rpm;
				pid_calc(&aim_scope_speed,aim_scope_speed.get,aim_scope_speed.set);
			}
			break;
		case 2:
		{
         pid_clr(&aim_scope_angle);
         pid_clr(&aim_scope_speed);
       		aim_scope_flag=0;
		      aim_scope_time=0;
		      aim_scope_change_flag=0;
//      if(gim.ctrl_mode == GIMBAL_INIT)
         aim_init_flag = 0;
		}
		break;
	}
}

void aim_scope_init(void)
{
	aim_rotate.set=100;
	aim_rotate.get=BUS1_CM8Encoder.rate_rpm;
	pid_calc(&aim_rotate,aim_rotate.get,aim_rotate.set);
	
	Set_GM_CM_Current(CAN1,0,0,0,aim_rotate.out);
	
	if (BUS1_CM8Encoder.current>1500)
	{
		scope_set_time++;
	}
	if (scope_set_time>50)
	{
		aim_init_flag=1;
		scope_set_time=0;
    init_angle = BUS1_CM8Encoder.ecd_angle;
	}
}

void camera_gimbal_control(void)
{
	switch (camera_gimbal_mode)//闭双环 给角度环赋值
	{
		case INIT://云台初始化
				cam_gim_init();
			break;
		case NORMAL:
				cam_gim_normal();
			break;
		case AUTO_ANGLE:  //吊射模式
				cam_gim_auto_angle();
			break;
		default:
				cam_gim_stop();         //停止
			break;
	}
}

float cangle;
float cam_angle = 0;
float lcam_angle = 0;
float lcam_angle_err = 0;

void cam_gim_auto_angle(void)
{
  if(auto_angle_shot_flag == 1)
      cangle = cangle;
  else
      cangle = roll_Angle - lcam_angle_err;
	VAL_LIMIT(cangle, -14, 41);
 
	camera_gim_angle.set = cangle;
	camera_gim_angle.get = BUS1_CM9Encoder.ecd_angle;
	pid_calc(&camera_gim_angle,camera_gim_angle.get,camera_gim_angle.set);

	camera_gim_speed.set=camera_gim_angle.out;
	camera_gim_speed.get=BUS1_CM9Encoder.rate_rpm;
	pid_calc(&camera_gim_speed,camera_gim_speed.get,camera_gim_speed.set);

	Set_GM_Current(CAN1,(int16_t)camera_gim_speed.out,0,0);
}

void cam_gim_normal(void)
{
	cangle = roll_Angle*k_angle;
	VAL_LIMIT(cangle, -14, 41);

	camera_gim_angle.set = cangle;
	camera_gim_angle.get = BUS1_CM9Encoder.ecd_angle;
	pid_calc(&camera_gim_angle,camera_gim_angle.get,camera_gim_angle.set);

	camera_gim_speed.set=camera_gim_angle.out;
	camera_gim_speed.get=BUS1_CM9Encoder.rate_rpm;
	pid_calc(&camera_gim_speed,camera_gim_speed.get,camera_gim_speed.set);

	Set_GM_Current(CAN1,(int16_t)camera_gim_speed.out,0,0);
}

void cam_gim_init(void)
{
			camera_gimbal_mode = INIT;

	camera_gim_angle.set=0;
	camera_gim_angle.get=BUS1_CM9Encoder.ecd_angle;
	pid_calc(&camera_gim_angle,camera_gim_angle.get,camera_gim_angle.set);

	camera_gim_speed.set=camera_gim_angle.out;
	camera_gim_speed.get=BUS1_CM9Encoder.rate_rpm;
	pid_calc(&camera_gim_speed,camera_gim_speed.get,camera_gim_speed.set);

   Set_GM_Current(CAN1,(int16_t)camera_gim_speed.out,0,0);

	if (camera_gim_angle.get-camera_gim_angle.set<=0.5f && camera_gim_angle.get-camera_gim_angle.set>=-0.5f)
	{
		camera_gimbal_mode = NORMAL;
	}
}

void cam_gim_stop(void)
{
	camera_gimbal_mode = RELAX;     //模式转换
  aim_init_flag = 2;

   Set_GM_Current(CAN1,0,0,0);
  Set_GM_CM_Current(CAN1,0,0,0,0);
//	cam_gim_back_param();              //云台重新初始化
	pid_clr(&camera_gim_angle);
	pid_clr(&camera_gim_speed);
}

void cam_gim_param_init(void)
{
		camera_gimbal_mode     = RELAX;

   Set_GM_Current(CAN1,0,0,0);

		PID_struct_init(&camera_gim_angle, POSITION_PID, 1000, 50, 0, 10 ,0.1, 30.0f);
		PID_struct_init(&camera_gim_speed, POSITION_PID, 10000, 100, 0, 100, 0, 5 );

		PID_struct_init(&aim_scope_angle, POSITION_PID, 12000, 100, 0, 5, 0, 20);
		PID_struct_init(&aim_scope_speed, POSITION_PID, 7000, 100, 0, 50, 0, 0);

		PID_struct_init(&aim_rotate, POSITION_PID, 12000, 4000, 0, 20, 0, 5);

}