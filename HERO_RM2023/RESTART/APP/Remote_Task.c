#include <stm32f4xx.h>
#include "main.h"

RC_Ctl_t RC_CtrlData;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
FrictionWheelState_e friction_wheel_last_state = FRICTION_WHEEL_OFF;
ChassisSpeed_Ref_t ChassisSpeedRef;
u8 friction_rotor=0;                        //摩擦轮开启标志位
Shoot_State_e shootState = NOSHOOTING;
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //摩擦轮斜坡
Gimbal_Ref_t GimbalRef;
static RemoteSwitch_t switch1;   //遥控器左侧拨杆
static InputMode_e inputmode = STOP;//REMOTE_INPUT;   //输入模式设定
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse左右移动斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouse前后移动斜坡
RampGen_t Chassis_Ramp = RAMP_GEN_DAFAULT;  //速度斜坡函数
int rotate_num_ture=0;	//云台反向判断

WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState = PREPARE_STATE;

float ramp_count=0;



void SetWorkState(WorkState_e state)
{
  workState = state;
}

WorkState_e GetWorkState()
{
  return workState;
}


//遥控器读值
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
  static uint32_t switch_cnt = 0;
  /* 最新状态值 */
  sw->switch_value_raw = val;
  sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;
  /* 取最新值和上一次值 */
  sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
                      (sw->switch_value_buf[sw->buf_index]);

  /* 最老的状态值的索引 */
  sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

  /* 合并三个值 */
  sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;
  /* 长按判断 */
  if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
    {
      switch_cnt++;
    }
  else
    {
      switch_cnt = 0;
    }

  if(switch_cnt >= 40)
    {
      sw->switch_long_value = sw->switch_value_buf[sw->buf_index];
    }

  sw->buf_last_index = sw->buf_index;
  sw->buf_index++;
  if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
    {
      sw->buf_index = 0;
    }
}

//输入模式设置
void SetInputMode(Remote *rc)
{
		if(rc->s2 == 1)
		{
				inputmode = REMOTE_INPUT;   //输出模式

		}
		else if(rc->s2 == 3)        //s2为中间的时候是鼠标模式——初始化后
		{
				inputmode = KEY_MOUSE_INPUT;  //鼠标模式和初始化模式

		}
		else if(rc->s2 == 2)
		{
				inputmode = STOP;         //关控
		}
}

InputMode_e GetInputMode()
{
		return inputmode;
}

/*********遥控器读值*************/
void RemoteDataPrcess(uint8_t *pData)
{
		if(pData == NULL)
    {
				return;
    }
		//遥控器部分
		RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
		RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
		RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
													((int16_t)pData[4] << 10)) & 0x07FF;
		RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
		RC_CtrlData.rc.ch4 = ((int16_t)pData[16]|((int16_t)pData[17]<<8))&0x07FF;
		RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
		RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//模式切换
		//鼠标部分
		RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
		RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
		RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
		RC_CtrlData.mouse.press_l = pData[12];
		RC_CtrlData.mouse.press_r = pData[13];
		RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
		//设定输入模式
		SetInputMode(&RC_CtrlData.rc);

		switch(GetInputMode())
    {
				case REMOTE_INPUT:
				{
						//遥控器控制模式
						RemoteControlProcess(&(RC_CtrlData.rc));
						SetWorkState(NORMAL_STATE);
				}
				break;
				case KEY_MOUSE_INPUT:
				{
						//键盘控制模式
				    MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
						SetWorkState(NORMAL_STATE);
				}
				break;
				case STOP:
				{
						SetWorkState(PREPARE_STATE);
						taget_angle_2=BUS1_CM5_Poke_Encoder.rotor_out;
						pid_clr(&pid_shoot_bullet_position_speed_loop_2);
						//紧急停车
				}
				break;
		}
}

void RemoteControlProcess(Remote *rc)
{
	  //不是初始化模式时
		if(gim.ctrl_mode != GIMBAL_INIT)
			{
				ChassisSpeedRef.forward_back_ref = (RC_CtrlData.rc.ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
				ChassisSpeedRef.left_right_ref = (rc->ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
			}
		//是底盘跟随云台模式时
		if(gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
			{
				GimbalRef.pitch_angle_dynamic_ref += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
				GimbalRef.yaw_angle_dynamic_ref += (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
			}
		GimbalRef.pitch_speed_ref = rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;    //speed_ref仅做输入量判断用
		GimbalRef.yaw_speed_ref = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		GimbalAngleLimit();
	/*******************遥控器拨杆数据处理*******************/
#if REMOTE_SHOOT == 0
	remote_self_rotate(&switch1, rc->s1);	
#elif REMOTE_SHOOT == 1
	RemoteShootControl(&switch1, rc->s1);            //此处两个函数二选一，左拨杆功能分别是发射和小陀螺
#elif REMOTE_SHOOT == 2
	Remote_Auto_Shoot_Control(&switch1,rc->s1);
#elif REMOTE_SHOOT == 3
	Remote_Remove_Barrier(&switch1,rc->s1);
#endif
	/***********************************************************/
}

u8 barriers_flag = 0; 
u8 dir_change_flag = 0;

//障碍的遥控器部分
void Remote_Remove_Barrier(RemoteSwitch_t *sw, uint8_t val)
{
	GetRemoteSwitchAction(sw,val);                       //遥控器读值
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1) 	 //从关闭到start turning,向上搬动
	{
		if(dir_change_flag == 0)
		{
			barriers_mode = BARRIERS_DOWN;
			dir_change_flag ^= 1 ;
		}
		else if(dir_change_flag == 1)
		{
			barriers_mode = BARRIERS_UP;
			dir_change_flag ^= 1 ;
		}
	}
	else if(sw->switch_value1 ==REMOTE_SWITCH_CHANGE_3TO2)//init or return zero
	{	
		if(barriers_zero_flag==1)
		{
			barriers_mode = BARRIERS_INIT;
		}
		else
		{
			barriers_mode = BARRIERS_RETURN;
			dir_change_flag = 0;
		}
	}
}

int auto_return_flag=0;                //底盘分离标志位
void Remote_Auto_Shoot_Control(RemoteSwitch_t *sw, uint8_t val)
{
	GetRemoteSwitchAction(sw,val);
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //从关闭到start turning,向上搬动
	{
		pid_clr(&pid_yaw_follow);
		pid_clr(&pid_pit_follow);
		pid_clr(&pid_yaw_speed);
		pid_clr(&pid_pit_speed);
		pid_clr(&pid_yaw);
		pid_clr(&pid_pit);

		pid_clr(&pid_pit_safe);
		pid_clr(&pid_pit_speed_safe);

		autoshoot_mode = AUTO_SHOOT;
		switch_mode_flag= 0;
		chassis.ctrl_mode=MANUAL_SEPARATE_GIMBAL;
		auto_return_flag = 0;
	}
	if((sw->switch_value1 == 1)||(sw->switch_value1 == 2))
		auto_return_flag = 0;
	else if(sw->switch_value1 == 3)
		auto_return_flag = 1;
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)
	{
		pid_clr(&pid_yaw_follow);
		pid_clr(&pid_pit_follow);
		pid_clr(&pid_yaw_speed);
		pid_clr(&pid_pit_speed);
		pid_clr(&pid_yaw);
		pid_clr(&pid_pit);

		pid_clr(&pid_pit_safe);
		pid_clr(&pid_pit_speed_safe);

		GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;
		GimbalRef.pitch_angle_dynamic_ref = roll_Angle;//GMPitchEncoder.ecd_angle;
		autoshoot_mode = NORMAL_SHOOT;
		chassis.ctrl_mode=MANUAL_FOLLOW_GIMBAL;
		auto_return_flag = 1;
	}
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)   //从关闭到start turning,向上搬动
	{
		pid_clr(&pid_yaw_follow);
		pid_clr(&pid_pit_follow);
		pid_clr(&pid_yaw_speed);
		pid_clr(&pid_pit_speed);
		pid_clr(&pid_yaw);
		pid_clr(&pid_pit);

		pid_clr(&pid_pit_safe);
		pid_clr(&pid_pit_speed_safe);

		autoshoot_mode = AUTO_STATION_SHOOT;
		switch_mode_flag= 2;
		chassis.ctrl_mode=MANUAL_SEPARATE_GIMBAL;
		auto_return_flag = 0;
	}
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
	{
		pid_clr(&pid_yaw_follow);
		pid_clr(&pid_pit_follow);
		pid_clr(&pid_yaw_speed);
		pid_clr(&pid_pit_speed);
		pid_clr(&pid_yaw);
		pid_clr(&pid_pit);
		GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;
		GimbalRef.pitch_angle_dynamic_ref = roll_Angle;//GMPitchEncoder.ecd_angle;
		autoshoot_mode = NORMAL_SHOOT;
		chassis.ctrl_mode=MANUAL_FOLLOW_GIMBAL;
		auto_return_flag = 1;
	}
}

int down_poke_off_flag=0;
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val)
{
	GetRemoteSwitchAction(sw, val);
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //从关闭到start turning,向上搬动
        {
          SetShootState(NOSHOOTING);
          friction_rotor=friction_stop;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          friction_wheel_state =  FRICTION_WHEEL_START_TURNNING;
					turn_friction_flag=1;
					down_poke_off_flag=0;
					//flag=1;
        }
    }
    break;
    case FRICTION_WHEEL_START_TURNNING:
    {
			//表示刚启动就被关闭，也就是模式为正在启动的过程中，再次向上拨s1
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //刚启动就被关闭
        {
          SetShootState(NOSHOOTING);
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          friction_rotor=friction_slow;
					turn_friction_flag=1;
					
        }
      else    //如果没有刚启动就被关闭，那么切换模式为正在启动模式
        {
          //摩擦轮加速
          friction_rotor=friction_on;
          friction_wheel_state = FRICTION_WHEEL_ON;
					turn_friction_flag=1;
					LASER_ON();
        }
    }
    break;
    case FRICTION_WHEEL_ON:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //关闭摩擦轮
        {
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
					turn_friction_flag=1;
          friction_rotor=friction_slow;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          SetShootState(NOSHOOTING);
					LASER_OFF();
					down_poke_off_flag=1;
        }
      else if(sw->switch_value_raw == 2)//////////////////////////
        {
          SetShootState(SHOOTING);
        }
      else
        {
          SetShootState(NOSHOOTING);
        }
    }
    break;
    case FRICTION_WHEEL_STOP_TURNNING:
    {
      friction_rotor=friction_slow;
      if(frictionRamp.IsOverflow(&frictionRamp))
			{
				friction_wheel_state = FRICTION_WHEEL_OFF;
				friction_rotor=friction_stop;
				turn_friction_flag=0;
			}
    }
    break;
  }
}

void SetShootState(Shoot_State_e v)
{
  shootState = v;
}

FrictionWheelState_e GetFrictionState(void)
{
  return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
  friction_wheel_state = v;
}

int rcflag=0;
int rotate_num = 0;
u8 close_rotate_flag = 0;
void remote_self_rotate(RemoteSwitch_t *sw, uint8_t val)//自转和左右旋转		
{
	GetRemoteSwitchAction(sw, val);
	switch (chassis.ctrl_mode)
    {
    case MANUAL_FOLLOW_GIMBAL:                    //底盘跟随云台
    {
			rotate_return_flag = 0;
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)    //s1从3拨到1
        {
          chassis.ctrl_mode = CHASSIS_ROTATE;               //小陀螺模式
					chassis.last_ctrl_mode = MANUAL_FOLLOW_GIMBAL;
        }
		  else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)  //s1从3拨到2
        {
					GimbalRef.yaw_angle_dynamic_ref +=180;
          chassis.ctrl_mode = CHASSIS_REVERSE;              //反转模式
					chassis.last_ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					reverse_flag=1;                                   //反转位置1
        }
    }
    break;
    case CHASSIS_ROTATE:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)
        {
          // 底盘云台相对角度给到下一个360的倍数
          rotate_num = GMYawEncoder.ecd_angle/360 ;
					chassis_rotate_flag ^=1;
					rotate_return_flag = 1;
					chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					chassis.last_ctrl_mode = CHASSIS_ROTATE;
        }
    }
    break;
		case CHASSIS_REVERSE:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
        {
//          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					gim.ctrl_mode=GIMBAL_INIT;
        }
    }
    break;
    default:
      break;
    }
}		

int auto_m_angle=0;
int auto_n_angle=0;//u8不行

//云台角度限制
void GimbalAngleLimit(void)
{
	//跟随云台模式
   if(gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
    {
	   VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  pitch_middle - 1963,  + pitch_middle+8548);
    }
	//吊射模式
	else if(gim.ctrl_mode == GIMBAL_AUTO_ANGLE)
		{
			//标志位
			if(auto_angle_shot_flag==0)
			{
				VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  -15,  + 40);//gim.pid.pit_angle_ref
				VAL_LIMIT(gim.pid.pit_angle_ref, ROLL_MIN+3,  + ROLL_MAX-5);
			}
		  else if(auto_angle_shot_flag==1)
			{
				VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  ROLL_MIN+3,  + ROLL_MAX-5);
			}
			if((shot_radar_flag)&&(auto_angle_data.pit_angle_raw>=0))
			{
				auto_m_angle = ROLL_MAX-auto_angle_data.pit_angle_raw;
				auto_n_angle = ROLL_MIN-auto_angle_data.pit_angle_raw;
				VAL_LIMIT(auto_angle_data.pit_angle_offset,  auto_n_angle,  + auto_m_angle);
			}
			else if((shot_radar_flag)&&(auto_angle_data.pit_angle_raw<0))
			{
				auto_m_angle = ROLL_MAX-auto_angle_data.pit_angle_raw;
				auto_n_angle = ROLL_MIN-auto_angle_data.pit_angle_raw;
				VAL_LIMIT(auto_angle_data.pit_angle_offset,  auto_n_angle,  + auto_m_angle);
			}
			if(shot_radar_flag == 0)
			{
				VAL_LIMIT(auto_angle_data.pit_angle_offset,  ROLL_MIN,  + ROLL_MAX);
			}
		}
}

u8 rotate_key_flag=0;
u8 reverse_key_flag=0;
u32 rotate_change_time = 0;
u32 reverse_change_time=0;
u32 separate_change_time=0;

u8 separate_reverse_flag=0;
u32 separate_reverse_time=0;

u8 ctrl_flag=0;
u8 key_ctrl_flag=0;
u32 key_ctrl_change_time=0;
u32 key_ctrl_return_change_time=0;
u8 rotate_change_flag=0;
u8 rotate_return_flag = 0;
u8 reverse_state=0;
/*********************底盘车键盘鼠标模式小陀螺***********************/
void Mouse_Key_Rotate_Reverse_Control(void)
{
	switch (chassis.ctrl_mode)
    {
    case MANUAL_FOLLOW_GIMBAL:                                 
    {
			rotate_return_flag = 0;
			rotate_num_ture=2;
			reverse_state=0;                                         //反转标志位为0
      if(RC_CtrlData.key.v & KEY_CTRL)                            //鼠标逻辑
        {
          chassis.ctrl_mode = CHASSIS_ROTATE;               //小陀螺模式
					chassis.last_ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					rotate_key_flag = 1;
					rotate_change_flag = 1;
					reverse_state=1;
        }
//			if((RC_CtrlData.key.v & KEY_V)&&(reverse_key_flag == 0))//底盘云台分离模式
//        {
//					GimbalRef.yaw_angle_dynamic_ref +=180;
//					chassis.position_ref-=360;
//          chassis.ctrl_mode = CHASSIS_REVERSE;
//					chassis.last_ctrl_mode = MANUAL_FOLLOW_GIMBAL;
//					reverse_key_flag=1;
//					reverse_flag=1;
//					reverse_state=1;                                      //
//        }
			if((RC_CtrlData.key.v & KEY_F)&&(chassis_separate_flag == 1))
        {
          chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL;
					chassis.last_ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					chassis_separate_flag=2;
        }
    }
    break;
    case CHASSIS_ROTATE:
    {
			if(!(RC_CtrlData.key.v&KEY_CTRL))
				{
//          rotate_num = GMYawEncoder.ecd_angle/360 ;// 底盘云台相对角度给到下一个360的倍数
					chassis_rotate_flag ^=1;
					rotate_return_flag = 1;
					chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					chassis.last_ctrl_mode = CHASSIS_ROTATE;
        }
    }
    break;
//		case CHASSIS_REVERSE:					//底盘云台反向
//		{
//			rotate_num_ture=1;
//			if(RC_CtrlData.key.v & KEY_V&&reverse_key_flag==0)
//        {
//					GimbalRef.yaw_angle_dynamic_ref -= 180;
//					chassis.position_ref+=360;
//					chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
//					chassis.last_ctrl_mode = CHASSIS_REVERSE;
//					chassis_separate_flag=0;
//					reverse_key_flag=2;
//				}
//			else if(RC_CtrlData.key.v & KEY_CTRL)
//        {
//          chassis.ctrl_mode = CHASSIS_ROTATE;
//					chassis.last_ctrl_mode = CHASSIS_REVERSE;
//					rotate_key_flag = 1;
//					rotate_change_flag = 1;
//        }
//			else if((RC_CtrlData.key.v & KEY_F)&&(chassis_separate_flag == 1))
//        {
//          chassis.ctrl_mode = MANUAL_SEPARATE_GIMBAL;
//					chassis.last_ctrl_mode = CHASSIS_REVERSE;
//					chassis_separate_flag=4;
//        }
//		}
//		break;
		case MANUAL_SEPARATE_GIMBAL:
		{
			if((RC_CtrlData.key.v & KEY_F)&&(chassis_separate_flag==3))
        {
					if(chassis.last_ctrl_mode == CHASSIS_REVERSE)
					{
						chassis.ctrl_mode = CHASSIS_REVERSE;
						chassis.last_ctrl_mode = MANUAL_SEPARATE_GIMBAL;
					}
					else if(autoshoot_mode!=AUTO_SHOOT)
					{
						chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
						chassis_separate_flag=0;
					}	
					chassis_separate_flag=0;
				}
			if(RC_CtrlData.key.v & KEY_CTRL)
				{
          chassis.ctrl_mode = CHASSIS_ROTATE;
					chassis.last_ctrl_mode = MANUAL_SEPARATE_GIMBAL;
					rotate_key_flag = 1;
					rotate_change_flag = 1;
        }
//			if((RC_CtrlData.key.v & KEY_V)&&(separate_reverse_flag == 0))//底盘云台反向
//        {
//					GimbalRef.yaw_angle_dynamic_ref += 180;
//					separate_reverse_flag = 1;
//        }
		}
		break;
    default:
      break;
    }
		
		if(separate_reverse_flag == 1)//底盘云台分离时云台反向
			separate_reverse_time++;
		else
			separate_reverse_time = 0;
		if(separate_reverse_time == 50)
		{	
			separate_reverse_flag = 0;
			separate_reverse_time = 0;
		}
		
		if((reverse_key_flag == 1)||(reverse_key_flag == 2))//底盘云台反向
      reverse_change_time++;
		else
			reverse_change_time=0;
		if(reverse_change_time>40)
    {
      reverse_key_flag = 0;
      reverse_change_time = 0;
    }		
		
		if(chassis_separate_flag == 0)//底盘云台分离
		{
			key_ctrl_change_time++;
		}
		else
			key_ctrl_change_time=0;
		if(key_ctrl_change_time>40)
		{
			chassis_separate_flag = 1;
			key_ctrl_change_time = 0;
		}
		
		if((chassis_separate_flag==2)||(chassis_separate_flag==4))
		{
			key_ctrl_return_change_time++;
		}
		else
			key_ctrl_return_change_time = 0;
		if(key_ctrl_return_change_time>40)
		{
			chassis_separate_flag = 3;
			key_ctrl_return_change_time = 0;
		}
}

u32 key_X_push_times = 0;
Key_Flag_t Key_Flag;
key_state_t key_r,key_z,last_key_z=0;

u8 rear_flag=0;

u8 key_X_mode=0;
u8 last_key_X_mode=0;
u8 key_barriers_change_flag = 0;
u8 low_speed_flag = 0;
u8 high_speed_flag = 0;
u8 chassis_separate_flag=0;
u8 last_chassis_separate_flag=0;
u32 auto_angle_key_flag=0;//开吊射关闭方向键
u8 switch_mode_flag=0;
u32 switch_mode_time1=0;
u32 switch_mode_time2=0;
u8 dir_mov_flag=0;
int yaw_init=0;
u32 position_flag=0;
u32 position_stop_time=0;
u8 position_stop_flag=0;
//键盘鼠标控制模式处理
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
  if(gim.ctrl_mode!= GIMBAL_INIT)//&&(gim.auto_ctrl_cmd!=CMD_TARGET_NUM)
    {
      Mouse_Key_Rotate_Reverse_Control();						//小陀螺和模式切换
			if(autoshoot_mode == NORMAL_SHOOT)//普通射击模式
        {
//          if(mouse->press_r)
//            {
//							pid_clr(&pid_yaw_follow);
//              pid_clr(&pid_pit_follow);
//              pid_clr(&pid_yaw_speed);
//              pid_clr(&pid_pit_speed);
//              pid_clr(&pid_yaw_station);
//              pid_clr(&pid_pit_station);
//              pid_clr(&pid_yaw_speed_station);
//              pid_clr(&pid_pit_speed_station);
//							if(switch_mode_flag==2)
//								autoshoot_mode = AUTO_STATION_SHOOT;
//							if(switch_mode_flag==0)
//								autoshoot_mode = AUTO_SHOOT;
//							auto_return_flag = 0;
//            }
					x_diff=0;
					delay_flag=0;
					last_dir_x=0;
					last_dir_yaw=0;
					compensate_time1=time_tick_1ms;
        }
      else if(autoshoot_mode != AUTO_ANGLE_SHOOT)
        {
//          if(!mouse->press_r)
//            {
//              pid_clr(&pid_yaw_follow);
//              pid_clr(&pid_pit_follow);
//              pid_clr(&pid_yaw_speed);
//              pid_clr(&pid_pit_speed);
//              pid_clr(&pid_yaw_station);
//              pid_clr(&pid_pit_station);
//              pid_clr(&pid_yaw_speed_station);
//              pid_clr(&pid_pit_speed_station);
//              GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;
//              GimbalRef.pitch_angle_dynamic_ref = roll_Angle;//GMPitchEncoder.ecd_angle;
//              autoshoot_mode = NORMAL_SHOOT;
//							if((BUS1_CM1Encoder.filter_rate<=-10)&&(BUS1_CM2Encoder.filter_rate>=10))
//								LASER_ON();
//							else
//								LASER_OFF();
//							chassis.ctrl_mode=MANUAL_FOLLOW_GIMBAL;
//							auto_return_flag = 1;
//            }
        }
			if(((autoshoot_mode==AUTO_SHOOT)||(autoshoot_mode==AUTO_STATION_SHOOT))&&(auto_return_flag==0))
			{
				chassis.ctrl_mode=MANUAL_SEPARATE_GIMBAL;
				chassis_separate_flag=0;
			}

			auto_angle_control();//吊射模式 KEY_Q			——按下Q进入吊射模式
			if(key->v &KEY_SHIFT)//KEY_SHIFT
        {
					if(judge_rece_mesg.game_robot_state.chassis_power_limit<=120&&judge_rece_mesg.game_robot_state.chassis_power_limit>90)
					{
						forward_back_speed=HIGH_FORWARD_BACK_SPEED_120;
						left_right_speed=HIGH_LEFT_RIGHT_SPEED_120;
					}
					else if(judge_rece_mesg.game_robot_state.chassis_power_limit<=90&&judge_rece_mesg.game_robot_state.chassis_power_limit>80)
					{
						forward_back_speed=HIGH_FORWARD_BACK_SPEED_90;
						left_right_speed=HIGH_LEFT_RIGHT_SPEED_90;
					}
					else if(judge_rece_mesg.game_robot_state.chassis_power_limit<=80)
					{
						forward_back_speed=HIGH_FORWARD_BACK_SPEED_80;
						left_right_speed=HIGH_LEFT_RIGHT_SPEED_80;
					}
        }
      else
        {
					if(judge_rece_mesg.game_robot_state.chassis_power_limit<=120&&judge_rece_mesg.game_robot_state.chassis_power_limit>90)
					{
						forward_back_speed=NORMAL_FORWARD_BACK_SPEED_120;
						left_right_speed=NORMAL_LEFT_RIGHT_SPEED_120;
					}
					else if(judge_rece_mesg.game_robot_state.chassis_power_limit<=90&&judge_rece_mesg.game_robot_state.chassis_power_limit>80)
					{
						forward_back_speed=NORMAL_FORWARD_BACK_SPEED_90;
						left_right_speed=NORMAL_LEFT_RIGHT_SPEED_90;
					}
					else if(judge_rece_mesg.game_robot_state.chassis_power_limit<=80)
					{
						forward_back_speed=NORMAL_FORWARD_BACK_SPEED_80;
						left_right_speed=NORMAL_LEFT_RIGHT_SPEED_80;
					}

        }
			if((high_speed_flag == 0)&&((key->v &KEY_SHIFT)!=1))
			{
				chassis_speed_mode = NORMAL_SPEED_MODE;
				high_speed_flag = 0;
			}
			
			if(key->v &KEY_C)   //KEY_C 开摩擦轮
				{
					key_r=KEY_R_DOWN;
				}
			else
				{
					key_r=KEY_R_UP;
				}
		//KEY_Z 摩擦轮反向+拨盘反向+拨弹轮反向（卡弹清除程序）
			if(key->v &KEY_Z)   
				{
					key_z = KEY_R_DOWN;
				}
			else
				{
					key_z = KEY_R_UP;
				}
			if(key_z==KEY_R_DOWN)
			{
				friction_wheel_state = FRICTION_WHEEL_BACK;
				friction_lock();
				lock_time++;
				if (lock_time>200)
						lock_cilent_flag=1;
			}
			else if(key_z==KEY_R_UP&&last_key_z==KEY_R_DOWN)
			{
				friction_wheel_state = friction_wheel_last_state;
				lock_flag=0;
				lock_cilent_flag=0;
				lock_time=0;
			}
			
			if((key->v &KEY_Z)&&(key->v &KEY_R)&&(!position_stop_flag))   //key->v 猜测是任何一个按键按下就可以置为1      Z为摩擦轮反向  R为无视发射热量
			{
				position_flag=1;
				position_stop_flag=1;
				position_set_flag=0;
			}
			if(position_stop_flag)
				position_stop_time++;
			else
				position_stop_time=0;
			if(position_stop_time==1000)
				position_stop_flag=0;
			
//******倍镜控制*****************************************************************************/
			if(RC_CtrlData.key.v&KEY_V)
				{
					if ((RC_CtrlData.key.v&KEY_V) && aim_scope_change_flag==0)
						{
							aim_scope_time++;
							aim_scope_change_flag=1;
						}
				}
			else
				 {
						aim_scope_change_flag=0;
				 }
			if(aim_scope_time%2)
				{
					aim_scope_flag=1;
				}
			else
				{
					aim_scope_flag=0;
				}
/*********************************************************************************************/

		 if(key->v &KEY_R)   //KEY_R 无视发射热量
				{
					ignore_heat=1;
				}
		 else
				{
					ignore_heat=0;
				}
		 
//				if((key->v &KEY_E)&&(switch_mode_flag==0)&&(autoshoot_mode!=AUTO_ANGLE_SHOOT)&&( !(key->v &KEY_W))&&(!(key->v &KEY_SHIFT)))//切换模式
//				{
//					autoshoot_mode=AUTO_STATION_SHOOT;
//					switch_mode_flag=1;
//					rear_flag=0;
//				}
//				else if((key->v &KEY_E)&&(switch_mode_flag==2)&&(autoshoot_mode!=AUTO_ANGLE_SHOOT)&&(!(key->v &KEY_W))&&(!(key->v &KEY_SHIFT)))//切换自瞄模式
//				{
//					autoshoot_mode=AUTO_SHOOT;
//					switch_mode_flag=3;
//					rear_flag=0;
//				}
//				else if((key->v &KEY_E)&&(key->v &KEY_W)&&(key->v &KEY_SHIFT))
//					rear_flag=1;
//				else
//					rear_flag=0;
				
				if(switch_mode_flag==1)
				{
					switch_mode_time1++;
				}
				else
				{
					switch_mode_time1=0;
				}
				if(switch_mode_time1==50)
				{
					switch_mode_flag=2;
				}

				if(switch_mode_flag==3)
				{
					switch_mode_time2++;
				}
				else
				{
					switch_mode_time2=0;
				}
				if(switch_mode_time2==50)
				{
					switch_mode_flag=0;
				}
				
		 if(key->v &KEY_X)   //长按X复位
			{
				key_X_push_times++;
				if(key_X_push_times>130)
				{
					SoftReset();
					key_X_push_times = 0;
				}
			}
		 else
			{
				key_X_push_times = 0;
			}
			if(key->v &KEY_B)   //KEY_B 客户端图形复位
			{
				draw_cnt=0;
				robot_level_flag = 0;
			}
			
//			if(key->v &KEY_G)   //KEY_G 移动反向，2023赛季暂时无用-2023.04.08
//			{
//				dir_mov_flag=0;
//			}
			
			if((chassis.position_ref/360)%2!=0)
				rotate_num_ture=1;
			else
				rotate_num_ture=2;
			
			
			if(!(key->v &KEY_W)&&!(key->v &KEY_S)&&!(key->v &KEY_A)&&!(key->v &KEY_D)&&ramp_count>=100)
			{
				Chassis_Ramp.SetScale(&Chassis_Ramp, 50);
				Chassis_Ramp.ResetCounter(&Chassis_Ramp);	
				ramp_count=0;
			}
			ramp_count++;
			
			
			
			if(auto_angle_key_flag==0)
			{
				if(key->v &KEY_W)  // key: W
				{
					Key_Flag.Key_W_S_Flag = 1;
					ChassisSpeedRef.forward_back_ref = forward_back_speed*Chassis_Ramp.Calc(&Chassis_Ramp);
				}
				else if(key->v &KEY_S) //key: S
				{
					Key_Flag.Key_W_S_Flag = 1;
					ChassisSpeedRef.forward_back_ref = -forward_back_speed*Chassis_Ramp.Calc(&Chassis_Ramp);
				}
				else
				{
					Key_Flag.Key_W_S_Flag = 0;
					ChassisSpeedRef.forward_back_ref = 0;
				}
				if(dir_mov_flag)
						ChassisSpeedRef.forward_back_ref=ChassisSpeedRef.forward_back_ref;
				else
						ChassisSpeedRef.forward_back_ref=-ChassisSpeedRef.forward_back_ref;
				if(chassis_separate_flag==0||chassis_separate_flag==1)
				{
					if(key->v &KEY_A)  // key: A
						{

							Key_Flag.Key_A_D_Flag = 1;
							ChassisSpeedRef.left_right_ref = -left_right_speed*Chassis_Ramp.Calc(&Chassis_Ramp);
						}
					else if(key->v &KEY_D) //key: D
						{

							Key_Flag.Key_A_D_Flag = 1;
							ChassisSpeedRef.left_right_ref = left_right_speed*Chassis_Ramp.Calc(&Chassis_Ramp);
						}
					else
						{
							ChassisSpeedRef.left_right_ref = 0;
							Key_Flag.Key_A_D_Flag = 0;
						}
					if(dir_mov_flag)
						ChassisSpeedRef.left_right_ref=ChassisSpeedRef.left_right_ref;
					else
						ChassisSpeedRef.left_right_ref=-ChassisSpeedRef.left_right_ref;
				}
				else if(chassis_separate_flag==3&&ctrl_flag==0)
				{
					if(key->v &KEY_A)  // key: A
						{
							ChassisSpeedRef.left_right_ref = 0;
							turn_speed=-30;
						}
					else if(key->v&KEY_D) //key: D
						{
							ChassisSpeedRef.left_right_ref = 0;
							turn_speed=30;
						}
					else
						{
							ChassisSpeedRef.left_right_ref = 0;
							turn_speed=0;
						}
				}
			}
			VAL_LIMIT(mouse->x, -150, 150);
			VAL_LIMIT(mouse->y, -150, 150);
			if(!auto_angle_flag)
			{
				GimbalRef.pitch_angle_dynamic_ref -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
				GimbalRef.yaw_angle_dynamic_ref += mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
				GimbalRef.pitch_speed_ref = mouse->y;    //speed_ref仅做输入量判断用
				GimbalRef.yaw_speed_ref = mouse->x;
			}
			GimbalAngleLimit();
			MouseShootControl(mouse);
	}
}

u8 last_key_r=0;
int16_t closeDelayCount = 0;   //右键关闭摩擦轮3s延时计数
void MouseShootControl(Mouse *mouse)
{
	switch(friction_wheel_state)
    { 
    case FRICTION_WHEEL_OFF:
    {
      if(key_r==KEY_R_DOWN&&last_key_r==KEY_R_UP)   //从关闭到start turning
        {
          SetShootState(SHOOTING);
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
//					friction_wheel_last_state = friction_wheel_state;
          friction_wheel_state = FRICTION_WHEEL_START_TURNNING;
          closeDelayCount = 0;
          friction_rotor=friction_stop;
					turn_friction_flag=1;
					down_poke_off_flag=0;
        }
			else
				friction_rotor=friction_stop;
    }
    break;
    case FRICTION_WHEEL_START_TURNNING:
    {
      if(key_r==KEY_R_DOWN)
        {
          closeDelayCount++;
        }
      else
        {
          closeDelayCount = 0;
        }
      if(closeDelayCount>70)   //关闭摩擦轮
        {
//					friction_wheel_last_state = friction_wheel_state;
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          SetShootState(NOSHOOTING);
          friction_rotor=friction_slow;
					turn_friction_flag=0;
        }
      else
        {
          //摩擦轮加速
          friction_rotor=friction_on;
          if(friction_rotor==friction_on)
          {
						friction_wheel_last_state = friction_wheel_state;
						friction_wheel_state = FRICTION_WHEEL_ON;
						LASER_ON();
					}
        }
    }
    break;
    case FRICTION_WHEEL_ON: //启动完成
    {
			friction_rotor=friction_on;
//-------------------鼠标发射-------------------
      if(mouse->press_l == 1)
        {
          SetShootState(SHOOTING);
        }
      else
				{
					SetShootState(NOSHOOTING);
				}
//--------------------------------------							
	    if(key_r==KEY_R_DOWN)
        {
          closeDelayCount++;
        }
      else
        {
          closeDelayCount = 0;
        }
				
      if(closeDelayCount>70)   //关闭摩擦轮
        {
//					friction_wheel_last_state = friction_wheel_state;
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;				  //切到关闭摩擦轮状态
          friction_rotor=friction_slow;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          SetShootState(NOSHOOTING);
					closeDelayCount=0;
					LASER_OFF();
					down_poke_off_flag=1;
        }
    }
    break;
    case FRICTION_WHEEL_STOP_TURNNING:
    {
      friction_rotor=friction_slow;
      if(frictionRamp.IsOverflow(&frictionRamp))
        {
          friction_rotor=friction_stop;
					friction_wheel_last_state = friction_wheel_state;
          friction_wheel_state = FRICTION_WHEEL_OFF;
        }
    }
    break;
		case FRICTION_WHEEL_BACK:
		{
			friction_rotor=friction_back;
		}
    }
  mouse->last_press_r = mouse->press_r;
  mouse->last_press_l = mouse->press_l;
  last_key_r=key_r;
	last_key_z=key_z;
}

void SoftReset(void)
{
  __set_FAULTMASK(1);      // 关闭所有中端
  NVIC_SystemReset();// 复位
}



void RemoteTaskInit(void)//遥控器数据初始化
{
	//斜坡初始化(scale表示的是斜率)
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
  LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT1);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
  FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT1);
  FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	Chassis_Ramp.SetScale(&Chassis_Ramp, 100);
	Chassis_Ramp.ResetCounter(&Chassis_Ramp);
  //底盘云台给定值初始化
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  ChassisSpeedRef.forward_back_ref = 0.0f;
  ChassisSpeedRef.left_right_ref = 0.0f;
  ChassisSpeedRef.rotate_ref = 0.0f;
	
	SetFrictionState(FRICTION_WHEEL_OFF);
}
