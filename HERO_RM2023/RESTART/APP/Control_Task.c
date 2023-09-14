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
		time_tick_1ms++;                       //����
		IWDG_ReloadCounter();                  //���Ź���δ����
		Hi220_getYawPitchRoll();//hi220        //�����Ƕ�ֵ
	  
	  //���ƵƵĿ���   ��0.2s  ��0.1s 
		if(time_tick_1ms%300 == 0)            
		{
				GREEN_LED_ON();
		}
		if((time_tick_1ms+100)%300 == 0)
		{
				GREEN_LED_OFF();
		}
		
		//ÿ3ms����һ��
		if(time_tick_1ms%3 == 0)
		{
				mode_switch_task();		 //ģʽ�л�
				gimbal_task();		     //��̨����
				barriers_task();
				last_current=now_current;       //������ֵ
				now_current=BUS1_CM1Encoder.current;       //��ȡ�µĵ���ֵ������ǵ���ֵ�𣿣���������������
			camera_gimbal_control();

		}
		if(time_tick_1ms%3==1)
		{
			shot_task();		 //�������
			aim_scope_control();
    }
		if(time_tick_1ms%3 == 0)
    {
			send_protocol(yaw_Angle,roll_Angle);//pitch_Angle
    }
		if(time_tick_1ms% 25== 1) //�ϴ��û���Ϣ
    {
			Client_send_handle();	//ͼ���������	
    }
		if(time_tick_1ms%2== 0)
    {
      SuperviseTask();//�������
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

void ControtLoopTaskInit(void)//���������ʼ��
{
	//������ʼ��
	time_tick_1ms = 0;   //�ж��еļ�������
	//���ù���ģʽ
  SetWorkState(PREPARE_STATE);
	gimbal_param_init();//��̨������ʼ��
	cam_gim_param_init();//ͼ����̨������ʼ��
	chassis_param_init();//���̲�����ʼ��
	shot_param_init();//��������ʼ��
	barriers_param_init();//���ϰ��������ʼ��
	//б�³�ʼ��
  GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
  GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
  GMPokeRamp.SetScale(&GMPitchRamp, POKE_PREPARE_TIME_TICK_MS);
  GMPitchRamp.ResetCounter(&GMPitchRamp);
  GMYawRamp.ResetCounter(&GMYawRamp);
  GMPokeRamp.ResetCounter(&GMPokeRamp);
  LADRC_Init();
	//��������ʼ��
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


