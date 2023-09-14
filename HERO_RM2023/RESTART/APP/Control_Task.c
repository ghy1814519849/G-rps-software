#include "main.h"

RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMPokeRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
Power_Control_Struct Power_Control = POWER_CONTROL_DEFAULT;

int now_count=0;
int last_count=0;
int now_current=0,last_current=0;
uint32_t time_tick_1ms = 0;
int pwm_flag2=0;
//-104.545898		34.5100021

void Control_Task(void)
{
		time_tick_1ms++;                       //计数
		IWDG_ReloadCounter();                  //看门狗（未看）
		Hi220_getYawPitchRoll();//hi220        //陀螺仪读值
	  
	  //控制灯的开关   开0.2s  关0.1s 
		if(time_tick_1ms%300 == 0)            
		{
				GREEN_LED_ON();
		}
		if((time_tick_1ms+100)%300 == 0)
		{
				GREEN_LED_OFF();
		}
		
		//每3ms进入一次
		if(time_tick_1ms%3 == 0)
		{
				mode_switch_task();		 //模式切换
				gimbal_task();		     //云台任务
				barriers_task();
				last_current=now_current;       //更新数值
				now_current=BUS1_CM1Encoder.current;       //获取新的电流值（这个是电流值吗？？？？？？？？）
			camera_gimbal_control();

		}
		if(time_tick_1ms%3==1)
		{
			shot_task();		 //射击任务
			aim_scope_control();
    }
		if(time_tick_1ms%3 == 0)
    {
			send_protocol(yaw_Angle,roll_Angle);//pitch_Angle
    }
		if(time_tick_1ms% 25== 1) //上传用户信息
    {
			Client_send_handle();	//图像分析部分	
    }
		if(time_tick_1ms%2== 0)
    {
      SuperviseTask();//监控任务
			chassis_task();
			
    }
		else if(time_tick_1ms%10==9)
		{
			power_send_handle1();
		}
		else if(time_tick_1ms%10==5)
		{
			power_send_handle2();
		}
		if(time_tick_1ms%100==0)
		{
			last_count=now_count;
			now_count=BUS1_CM1Encoder.round_cnt;
		}
		
}

void ControtLoopTaskInit(void)//控制任务初始化
{
	//计数初始化
	time_tick_1ms = 0;   //中断中的计数清零
	//设置工作模式
  SetWorkState(PREPARE_STATE);
	gimbal_param_init();//云台参数初始化
	cam_gim_param_init();//图传云台参数初始化
	chassis_param_init();//底盘参数初始化
	shot_param_init();//射击任务初始化
	barriers_param_init();//搬障碍块任务初始化
	//斜坡初始化
  GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
  GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
  GMPokeRamp.SetScale(&GMPitchRamp, POKE_PREPARE_TIME_TICK_MS);
  GMPitchRamp.ResetCounter(&GMPitchRamp);
  GMYawRamp.ResetCounter(&GMYawRamp);
  GMPokeRamp.ResetCounter(&GMPokeRamp);
  LADRC_Init();
	//监控任务初始化
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
  LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));
}


