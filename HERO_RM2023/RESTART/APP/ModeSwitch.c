#include "main.h"

//模式切换函数
void mode_switch_task(void)
{
		get_gimbal_mode();    
    get_chassis_mode();
}

//
void get_gimbal_mode(void)
{
  if (gim.ctrl_mode==GIMBAL_RELAX && GetInputMode() != STOP)     //模式不为停止   并且  模式为 GIMBAL_RELAX   （普通模式时的云台初始化）
  {
      gim.ctrl_mode = GIMBAL_INIT;
			gimbal_back_param();                                //斜坡开启供电，初始值赋值，此时云台不能动
	}
	if (GetInputMode() == STOP)                             //云台关控
	{
			gim.ctrl_mode = GIMBAL_RELAX;
			autoshoot_mode = NORMAL_SHOOT;
			position_set_flag=0;
			gimbal_back_param();
    
	}
}

//获取底盘的模式
void get_chassis_mode(void)
{
	  //如果云台的模式为，初始化模式  或者  （底盘）停止模式  或者  关控模式     底盘的模式为 关控模式
	  if (gim.ctrl_mode == GIMBAL_INIT || GetInputMode() == STOP || gim.ctrl_mode == GIMBAL_RELAX)
	  {
			chassis.ctrl_mode = CHASSIS_RELAX;
	  }
		//如果底盘不为陀螺模式 ，并且  不为 底盘云台分离模式  并且 不为 底盘颠倒模式  并且  不为吊射模式  ————让底盘的模式变为底盘跟随云台的模式
	  else if((chassis.ctrl_mode != CHASSIS_ROTATE)&&(chassis.ctrl_mode != MANUAL_SEPARATE_GIMBAL)&&(chassis.ctrl_mode != CHASSIS_REVERSE)&&(gim.ctrl_mode != GIMBAL_AUTO_ANGLE)&&(autoshoot_mode != AUTO_SHOOT))
	  {
			chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
	  } 
}

//云台可控
uint8_t gimbal_is_controllable(void)
{
	//如果云台的模式为 关控  或者  输出模式为关闭   或者  有错误 ———— 此时返回0  否则返回1
  if (gim.ctrl_mode == GIMBAL_RELAX || GetInputMode() == STOP || Is_Lost_Error_Set(LOST_ERROR_RC)
      
			
//			||Is_Lost_Error_Set(LOST_ERROR_IMU) 
			)
    return 0;
  else
    return 1;
}

//底盘可控   
uint8_t chassis_is_controllable(void)
{
	//如果 底盘的模式为 关控  或者  输出为停止   或者  出现错误  ————此时返回0，否则返回1
  if (chassis.ctrl_mode == CHASSIS_RELAX
      ||GetInputMode() == STOP
			||Is_Lost_Error_Set(LOST_ERROR_IMU)
     )
    return 0;
  else
    return 1;
}
