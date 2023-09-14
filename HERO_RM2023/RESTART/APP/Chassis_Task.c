#include "main.h"

int turn_speed=0;
chassis_t chassis;
chassis_speed_mode_e chassis_speed_mode;//���̿��� �����ٶȿ���
chassis_rotate_speed_t chassis_rotate_speed;
uint16_t forward_back_speed = 0;
uint16_t left_right_speed = 0;
u8 chassis_rotate_flag = 0;

//��������
float power_limit_rate=1;//��������ϵ��
int16_t power_limit_model; 
u8 CanVcap_flag = 0;

float total_chassis_wheel_speed_ref = 0.0f;
float total_chassis_wheel_speed_limit = 0.0f;
//��������
//float temp_power;
//u8 chassis_power_buffer_flag;
//u8 chassis_rotate_buffer_flag;//
//int chassis_power_buffer_time;
//int chassis_rotate_buffer_time;//
//u32 poe=0;
//int ddd=0;//����ٱ�

float pid_outmax=16000;
float pid_ioutmax=100; 
float pid_p=50;
float pid_i=0.2;
float pid_d=20;
float pid_output_deadband=0;
int max_current=3000;

void chassis_task(void)
{
		if(GMYawEncoder.ecd_angle<-360)            //����Ƕ�Ϊ���ģ���ô��ʼ��Ϊ��360   ����Ƕ�Ϊ������ô��ʼ��Ϊ��360����������������������������������������
				yaw_init = -360;
		else
				yaw_init = 360;
		
		switch(chassis.ctrl_mode)
		{
				case CHASSIS_STOP:  //����ֹͣ
				{
						chassis_stop_handle();
				}
				break;
				case MANUAL_FOLLOW_GIMBAL:  //���̸�����̨
				{
						follow_gimbal_handle();
				}
				break;
				case CHASSIS_REVERSE:  //������̨����
				{
						reverse_follow_gimbal_handle();
				}
				break;
				case CHASSIS_ROTATE:  //С����
				{
						rotate_handle();
				}
				break;
				case MANUAL_SEPARATE_GIMBAL: //������̨����
				{
						chassis_separate_handle();
				}
				break;
				default:
				{
						chassis_stop_handle();
				}
		}
		mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_speed_ref);
		chassis.wheel_speed_fdb[0]=BUS2_CM1Encoder.filter_rate;
		chassis.wheel_speed_fdb[1]=BUS2_CM2Encoder.filter_rate;
		chassis.wheel_speed_fdb[2]=BUS2_CM3Encoder.filter_rate;
		chassis.wheel_speed_fdb[3]=BUS2_CM4Encoder.filter_rate;
		power_limit_handle();   //��������

		//���Դ���
		for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,pid_outmax, pid_ioutmax, pid_output_deadband, pid_p, pid_i, pid_d); //24 0.3 10    38.0f,3.0f, 40
    }

		if(rear_flag)
		{
			for (int i = 2; i < 4; i++)
			{
				chassis.current[i] = (int)pid_calc(&pid_spd[i], chassis.wheel_speed_fdb[i],power_limit_rate*chassis.wheel_speed_ref[i]);
//				if (chassis.wheel_speed_fdb[i]<100 && chassis.wheel_speed_fdb[i]>-100)
//						{VAL_LIMIT(chassis.current[i],-max_current,max_current);}
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				chassis.current[i] = (int)pid_calc(&pid_spd[i], chassis.wheel_speed_fdb[i],power_limit_rate*chassis.wheel_speed_ref[i]);
//				if (chassis.wheel_speed_fdb[i]<100 && chassis.wheel_speed_fdb[i]>-100)
//						{VAL_LIMIT(chassis.current[i],-max_current,max_current);}
			}
		}
  //����ָ���������
		if(chassis.ctrl_mode == CHASSIS_RELAX||GetInputMode() == STOP)
			Set_CM_Speed(CAN2,0,0,0,0);
		else
		{
			if(rear_flag)
				Set_CM_Speed(CAN2, 0,0,CHASSIS_SPEED_ATTENUATION * chassis.current[2],CHASSIS_SPEED_ATTENUATION * chassis.current[3]);
			else
				Set_CM_Speed(CAN2, CHASSIS_SPEED_ATTENUATION * chassis.current[0],CHASSIS_SPEED_ATTENUATION * chassis.current[1],CHASSIS_SPEED_ATTENUATION * chassis.current[2],CHASSIS_SPEED_ATTENUATION * chassis.current[3]);
		}
}
float gimbal_chassis_diff = 0;
int rotate_num_now = 0;
int int_rotate_angle = 0;
float stop_diff=0;
void follow_gimbal_handle(void)
{
		//���ٶȸ�ֵ����6020���ٱ�1��2
//  if(rotate_num_ture%2==1)
//	{
//		chassis.vy = ChassisSpeedRef.left_right_ref;
//		chassis.vx = ChassisSpeedRef.forward_back_ref;
//	}
//	else if(rotate_num_ture%2==0)					
//	{
//		chassis.vy = -ChassisSpeedRef.left_right_ref;
//		chassis.vx = -ChassisSpeedRef.forward_back_ref;
//	}
		//���ٶȸ�ֵ����9025���ٱ�1��1
	chassis.vy = ChassisSpeedRef.left_right_ref;
	chassis.vx = ChassisSpeedRef.forward_back_ref;

	gimbal_chassis_diff = GMYawEncoder.ecd_angle;	
  if(dir_mov_flag)
		rotate_num_now = GMYawEncoder.ecd_angle/360;             //��ȡ��ǰ��Ȧ������ʼ��Ϊ0��
	else
		rotate_num_now = (yaw_init+GMYawEncoder.ecd_angle)/360;  //���߳�ʼ��Ϊ�����ǡ�������
  int_rotate_angle = rotate_num_now*360;
  stop_diff = int_rotate_angle;
	
	if((gimbal_chassis_diff-stop_diff>180))//&&(gimbal_chassis_diff-stop_diff<540)            //��֤diff��ֵ���Ŷ��ĽǶȵ����������
		stop_diff+=360;
	else if(gimbal_chassis_diff-stop_diff<-180)
		stop_diff-=360;
//	else if((gimbal_chassis_diff-stop_diff<-180)&&(!((GMYawEncoder.ecd_angle<-360)&&(yaw_init==360))))//&&(gimbal_chassis_diff-stop_diff>-540)
//		stop_diff-=360;
//	else if((gimbal_chassis_diff-stop_diff<-360)&&((GMYawEncoder.ecd_angle<-360)&&(yaw_init==360)))//&&(gimbal_chassis_diff-stop_diff>-540)
//		stop_diff-=720;		
//	else if((gimbal_chassis_diff-stop_diff<-180)&&((GMYawEncoder.ecd_angle<-360)&&(yaw_init==360)))//&&(gimbal_chassis_diff-stop_diff>-540)
//		stop_diff-=360;
	 chassis.position_ref=stop_diff;                 //ֹͣ��λ�á������̸�����̨
	
  if (chassis.follow_gimbal)
      chassis.vw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref);
  else
      chassis.vw = 0;
}

u32 reverse_time_count = 0;
u32 reverse_flag=1;
void reverse_follow_gimbal_handle(void)
{
	//���ݷ�ת��־λ���ı��ٶȵ�����
	if(rotate_num_ture%2==0)
	{
		chassis.vy = -ChassisSpeedRef.left_right_ref;
		chassis.vx = -ChassisSpeedRef.forward_back_ref;
	}
	else if(rotate_num_ture%2==1)					
	{
		chassis.vy = ChassisSpeedRef.left_right_ref;
		chassis.vx = ChassisSpeedRef.forward_back_ref;
	}
	 
	//���������̨�ı�־λΪ1
  if (chassis.follow_gimbal)
    {
			if(pid_yaw.get-pid_yaw.set<1.5f&&pid_yaw.get-pid_yaw.set>-1.5f&&reverse_flag)
      {
				reverse_time_count++;				
				chassis.vw = 0;
			}
			else
				chassis.vw = 0;
			if(reverse_time_count>2)
			{
				reverse_flag=0;
				reverse_time_count=0;
			}
			if(!reverse_flag)
			{
				chassis.vw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref);
				gimbal_chassis_diff = GMYawEncoder.ecd_angle;	
				if(dir_mov_flag)
					rotate_num_now = GMYawEncoder.ecd_angle/360;
				else
					rotate_num_now = (yaw_init+GMYawEncoder.ecd_angle)/360;
				int_rotate_angle = rotate_num_now*360;
				stop_diff = int_rotate_angle;
				if((gimbal_chassis_diff-stop_diff>180))//&&(gimbal_chassis_diff-stop_diff<540)
					stop_diff+=360;
				else if((gimbal_chassis_diff-stop_diff<-180))//&&(gimbal_chassis_diff-stop_diff>-540)
					stop_diff-=360;				
				 chassis.position_ref=stop_diff;
			}
    }
  else
    {
      chassis.vw = 0;
    }
}
int cacsaav=0;
int return_speed_flag=1;
int chassis_rotate_speed_vw_ref;
float real_angle=0;

//С����ģʽ
void rotate_handle(void)
{
	//��������
	switch(judge_rece_mesg.game_robot_state.chassis_power_limit)
	{
		case 55:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_55;
		break;
		case 60:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_60;
		break;
		case 65:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_65;
		break;

		case 70:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_70;
		break;
		case 90:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_90;
		break;
		case 120:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_120;
		break;

		default:
			chassis_rotate_speed_vw_ref=ROTATE_SPEED_REF_70;
		break;
	}
	
	//ģʽ�л��ɹ�
	if(gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
	{
		//����ת��Ҫ��
		if (chassis.follow_gimbal)
		{
			//�ж���ת���Ƿ�ת
			if(chassis_rotate_flag == 1)
			{
				//���ת��̫����������ת��		&&chassis_rotate_buffer_flag&&chassis_rotate_buffer_time<300
				if(judge_rece_mesg.power_heat_data.chassis_power_buffer>20)
					chassis.vw = chassis_rotate_speed_vw_ref+30;   //�˲���Ϊת��
				else
					chassis.vw = chassis_rotate_speed_vw_ref;
			}
			else if(chassis_rotate_flag == 0)
			{				//&&chassis_rotate_buffer_flag&&chassis_rotate_buffer_time<300
				if(judge_rece_mesg.power_heat_data.chassis_power_buffer>20)
					chassis.vw = -chassis_rotate_speed_vw_ref - 30;    //�˲���Ϊת��
				else 
					chassis.vw = -chassis_rotate_speed_vw_ref;
			}
		}
		else
			chassis.vw = 0;     //����ת��Ϊ0
	}
	
	//�ǶȵĲ�ֵ
	gimbal_chassis_diff = GMYawEncoder.ecd_angle;	               //�ǶȵĲ�ֵΪ����ĽǶ�ֵ
	if(dir_mov_flag)                                             //ת����Ȧ��
		rotate_num_now = GMYawEncoder.ecd_angle/360;
	else
		rotate_num_now = (yaw_init+GMYawEncoder.ecd_angle)/360;
	
  int_rotate_angle = rotate_num_now*360;                       //����Ƕ�ֵΪȦ����������
  stop_diff = int_rotate_angle;                                //stop Ϊ��ʼֵ
	
	//
  if(rotate_return_flag == 1)
		{
			rotate_return_flag = 2;                                  //��־λ���л�ģʽ
			if((gimbal_chassis_diff-stop_diff>180))//&&(gimbal_chassis_diff-stop_diff<540)          //����stop��ֵ���߼���Ȧ��ͬ��
				stop_diff+=360;
			else if((gimbal_chassis_diff-stop_diff<-180))//&&(gimbal_chassis_diff-stop_diff>-540)
				stop_diff-=360;				
			chassis.position_ref=stop_diff;                                                         //�ο�ֵ���յ���ʼֵ
	  }
		real_angle=(GMYawEncoder.ecd_angle);///720.0*360.0;                                          //1:2����Ƕ�
		chassis_rotate_speed.sin_chassis_angle = sin(real_angle*ANGLE_TO_RAD);
		chassis_rotate_speed.cos_chassis_angle = cos(real_angle*ANGLE_TO_RAD);
		
		//���⣺   ���Ǻ����Ĳ���Ϊ����ֵ         �ڶ���ǰ��Ӹ���
		chassis_rotate_speed.foward_back_to_foward_back_rotate_speed = ChassisSpeedRef.forward_back_ref*chassis_rotate_speed.cos_chassis_angle*RATATE_DCREASE;
		chassis_rotate_speed.foward_back_to_left_right_rotate_speed  = -ChassisSpeedRef.forward_back_ref*chassis_rotate_speed.sin_chassis_angle*RATATE_DCREASE;
		chassis_rotate_speed.left_right_to_foward_back_rotate_speed  = ChassisSpeedRef.left_right_ref*chassis_rotate_speed.sin_chassis_angle*RATATE_DCREASE;
		chassis_rotate_speed.left_right_to_left_right_rotate_speed   = ChassisSpeedRef.left_right_ref*chassis_rotate_speed.cos_chassis_angle*RATATE_DCREASE;
		chassis.vy = chassis_rotate_speed.foward_back_to_left_right_rotate_speed  + chassis_rotate_speed.left_right_to_left_right_rotate_speed;
		chassis.vx = chassis_rotate_speed.foward_back_to_foward_back_rotate_speed + chassis_rotate_speed.left_right_to_foward_back_rotate_speed;
		
		
		//��־λΪ2��ʱ���ʾС��������
		if(rotate_return_flag == 2)
		{
			chassis.vw = pid_calc(&pid_chassis_angle,GMYawEncoder.ecd_angle,chassis.position_ref);       //ת���ٶ�  ����PID��֤ת���ȶ���
			//��֤�ο�ֵ��α仯��С����ת��һֱ��һ������
			if(chassis.vw > 0)
				chassis.vw = chassis_rotate_speed_vw_ref;
			else
				chassis.vw = -chassis_rotate_speed_vw_ref;
		}
		
		//�������������ã�������������������������                   ��С�����������ң��ο�ֵ���ֵ�ľ���ֵ���С��1.5ʱ����if��
		if((rotate_return_flag == 2)&&((chassis.position_ref - gimbal_chassis_diff)<1.5&&(chassis.position_ref - gimbal_chassis_diff)>-1.5))
		{
			rotate_return_flag = 0;
			chassis.ctrl_mode = chassis.last_ctrl_mode;
			chassis.last_ctrl_mode = CHASSIS_ROTATE;
		}
}

//��������̨����
void chassis_separate_handle(void)
{
	if(reverse_state)        //��ת��־λ�����Ϊ�棬��ô���еļӸ��ţ�
	{
		chassis.vy = -0.75*ChassisSpeedRef.left_right_ref;
		chassis.vx = -ChassisSpeedRef.forward_back_ref;
	}
	else
	{
		chassis.vy = 0.75*ChassisSpeedRef.left_right_ref;
		chassis.vx = ChassisSpeedRef.forward_back_ref;
	}
	chassis.vw =turn_speed;         //��ת�ٶ�Ϊ�����ٶ�
}

//���ֵ�����
void mecanum_calc(float vx, float vy, float vw, int16_t *speed)
{
		int16_t wheel_rpm[4];
		float   max = 0;
	  wheel_rpm[0] = (-vx + vy + vw*3.9f)*2;
		wheel_rpm[1] = ( vx + vy + vw*3.9f)*2;
		wheel_rpm[2] = ( vx - vy + vw*3.9f)*2;
		wheel_rpm[3] = (-vx - vy + vw*3.9f)*2;
	
	  //��������ٶȹ��죬�ɱ����µ�
		for (uint8_t i = 0; i < 4; i++)//�ҵ�����ٶ�
    {
				if (abs(wheel_rpm[i]) > max)
						max = abs(wheel_rpm[i]);
    }
		if (max > MAX_WHEEL_RPM)
    {
				float rate = MAX_WHEEL_RPM / max;
				for (uint8_t i = 0; i < 4; i++)
						wheel_rpm[i] *= rate;
    }
		memcpy(speed, wheel_rpm, 4*sizeof(int16_t));//�ڴ濽��������rpmһ������ת��
}

//ֹͣ
void chassis_stop_handle(void)
{
	chassis_separate_flag=0;
	chassis.vy =0;
	chassis.vx =0;
	chassis.vw =0;
	pid_clr(&pid_chassis_angle);
	pid_clr(&pid_spd[0]);
	pid_clr(&pid_spd[1]);
	pid_clr(&pid_spd[2]);
	pid_clr(&pid_spd[3]);

}

u16  Max_Power    = 100;
u8  Max_Current   = 10;
u8 Cap_Reset_flag = 0;

static float get_the_limite_rear_rate(float max_power)
{
  float a[4];
  for(int i=2; i<4; i++)
    a[i]=(float)chassis.wheel_speed_ref[i]*(pid_spd[i].p+pid_spd[i].d);
  float b[4];
  for(int i=2; i<4; i++)
    b[i]=-pid_spd[i].p*(float)chassis.wheel_speed_fdb[i]+pid_spd[i].iout \
         -pid_spd[i].d*(float)chassis.wheel_speed_fdb[i]-pid_spd[i].d*pid_spd[i].err[LAST];
  // Max_power=heat_power+drive_power
  //	i_n=a[n]*k+b[n]	����
  //Max_Power=m*k^2+n*k+o
  //0=m*k^2+n*k+l(l=o-Max_Power)
  float m=(a[2]*a[2]+a[3]*a[3])*FACTOR_2;

  float n=2*FACTOR_2*(a[2]*b[2] + a[3]*b[3]) + \
          FACTOR_1*(a[2] + a[3]) + \
          I_TIMES_V_TO_WATT*(a[2]*(float)chassis.wheel_speed_fdb[2] + \
                             a[3]*(float)chassis.wheel_speed_fdb[3]);

  float l=(b[2]*b[2] + b[3]*b[3])*FACTOR_2 + \
          (b[2] + b[3])*FACTOR_1 + \
          I_TIMES_V_TO_WATT*(b[2]*(float)chassis.wheel_speed_fdb[2] + \
                             b[3]*(float)chassis.wheel_speed_fdb[3]) + \
          4*FACTOR_0 - \
          max_power;
  return (-n+(float)sqrt((double)(n*n-4*m*l)+1.0f))/(2*m);
}

//�˺���Ϊ���㹦������ϵ��k
static float get_the_limite_rate(float max_power)
{
  float a[4];
  for(int i=0; i<4; i++)
    a[i]=(float)chassis.wheel_speed_ref[i]*(pid_spd[i].p+pid_spd[i].d);
  float b[4];
  for(int i=0; i<4; i++)
    b[i]=-pid_spd[i].p*(float)chassis.wheel_speed_fdb[i]+pid_spd[i].iout \
         -pid_spd[i].d*(float)chassis.wheel_speed_fdb[i]-pid_spd[i].d*pid_spd[i].err[LAST];
  // Max_power=heat_power+drive_power
  //	i_n=a[n]*k+b[n]	����
  //Max_Power=m*k^2+n*k+o
  //0=m*k^2+n*k+l(l=o-Max_Power)
  float m=(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]+a[3]*a[3])*FACTOR_2;

  float n=2*FACTOR_2*(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]) + \
          FACTOR_1*(a[0] + a[1] + a[2] + a[3]) + \
          I_TIMES_V_TO_WATT*(a[0]*(float)chassis.wheel_speed_fdb[0] + \
                             a[1]*(float)chassis.wheel_speed_fdb[1] + \
                             a[2]*(float)chassis.wheel_speed_fdb[2] + \
                             a[3]*(float)chassis.wheel_speed_fdb[3]);

  float l=(b[0]*b[0] + b[1]*b[1] + b[2]*b[2] + b[3]*b[3])*FACTOR_2 + \
          (b[0] + b[1] + b[2] + b[3])*FACTOR_1 + \
          I_TIMES_V_TO_WATT*(b[0]*(float)chassis.wheel_speed_fdb[0] + \
                             b[1]*(float)chassis.wheel_speed_fdb[1] + \
                             b[2]*(float)chassis.wheel_speed_fdb[2] + \
                             b[3]*(float)chassis.wheel_speed_fdb[3]) + \
          4*FACTOR_0 - \
          max_power;
  return (-n+(float)sqrt((double)(n*n-4*m*l)+1.0f))/(2*m);
}

//���Ƶ�ѹ��ֹ��ѹ���͵��µ����λ
  int max_power=0;
float get_max_power(float voltage)///////////////
{
  if(voltage>WARNING_VOLTAGE+3)
    max_power=150;
  else
    max_power=(voltage-WARNING_VOLTAGE)/3.0f*200;
  VAL_LIMIT(max_power,0,150);
  return max_power;
//  return 80;
}

////�������ƺ���
//void power_limit_handle(void)
//{
//  CanVcap_flag=Check_Vcap_recieve();   //�鿴�Ƿ���յ����µ���Ϣ
//  Fault_judge();                       //��鳬�������Ƿ���������
//	buffer_power();
//  power_send_handle();                 //����ָ���������
//	if(rear_flag)
//		power_limit_rate=get_the_limite_rear_rate(get_max_power(capacitance_message1.cap_voltage_filte));
//	else
//		power_limit_rate=get_the_limite_rate(get_max_power(capacitance_message1.cap_voltage_filte));
//	VAL_LIMIT(power_limit_rate,0,1);
//}

//u8 Power_Work_Mode;
//void Fault_judge(void)
//{
//  CanVcap_flag=Check_Vcap_recieve();   //�鿴�Ƿ���յ��µ���Ϣ
//  if((capacitance_message2.fault_union.fault==0)&&(CanVcap_flag) \
//      &&(judge_rece_mesg.game_robot_state.mains_power_chassis_output==1))	 //û�д���
//    {
//      if((capacitance_message2.system_mode==2))   //��ȷ���в�û������(����ʱ��󴫻���������������)
//        {
//          Power_Work_Mode=1;
//        }
//    }
//  else
//    {
//      Power_Work_Mode = 0;
//    }
//  if(capacitance_message2.fault_union.fault==8) //��ضϵ�
//    Cap_Reset_flag |= 0x04;
//  else
//    Cap_Reset_flag &= ~0x04;
//}

//void power_send_handle(void)//�������ݸ���������
//{
//  static u8 time=0;
//  time++;
//  static u8 canbuf[8];
//  canbuf[0]=0x10;
//  canbuf[1]=Max_Power;
//  canbuf[2]=Max_Current;
//  canbuf[3]=Power_Work_Mode;
//  canbuf[4]=Cap_Reset_flag;
//  canbuf[5]=0;
//  canbuf[6]=0;
//  canbuf[7]=0;
//  if(time%10==0)
//    {
//      POWER_Control(canbuf);//����8���ֽ�
//    }
//}

//��Ʒģ��
float volatilAo; 
void power_limit_handle(void)
{
  volatilAo=capacitance_message3.out_v;
	capacitance_message1.cap_voltage_filte=volatilAo/100;
  power_limit_rate=get_the_limite_rate(get_max_power(capacitance_message1.cap_voltage_filte));
  VAL_LIMIT(power_limit_rate,0,1);	
}

void power_limit2(void)//�������ƣ���ֹ��ѹ���ͣ�δ����
{
  volatilAo=capacitance_message3.out_v;
	capacitance_message1.cap_voltage_filte=volatilAo/100;
  if(capacitance_message1.cap_voltage_filte<=15)
  {
    power_limit_rate=0;
  }
  if(capacitance_message1.cap_voltage_filte>=18)
  {
    power_limit_rate=1;
  }
}

//uint16_t voltage,electricity;
void power_send_handle2(void)    //��Ӧ���������ID�ţ����ǲ��÷����ݣ���ʱû��	2023.05.08
{
	POWER_Control2(0x610);
	POWER_Control2(0x611);
	POWER_Control2(0x612);
	POWER_Control2(0x613);
}

void power_send_handle1(void)
{
	buffer_power();
	POWER_Control1(2,0x600);//��������
	POWER_Control1(Max_Power*100,0x601);//������빦��
	POWER_Control1(2200,0x602);//�����ѹ
	POWER_Control1(15*100,0x603);//�������
}

void buffer_power(void)
{
	if(capacitance_message1.cap_voltage_filte<22.5)
		Max_Power = judge_rece_mesg.game_robot_state.chassis_power_limit+(judge_rece_mesg.power_heat_data.chassis_power_buffer-10)*2; //5
	else
		Max_Power=0;
	VAL_LIMIT(Max_Power,0,150);
}

void VoltageCalc(void)//
{
  total_chassis_wheel_speed_ref = 0;
  for(int i=0; i < 4; i++)
    {
      total_chassis_wheel_speed_ref+=abs(chassis.wheel_speed_ref[i]);
    }
  for(int a=0; a < 4; a++)
    {
      chassis.wheel_speed_ref[a]=(chassis.wheel_speed_ref[a]/total_chassis_wheel_speed_ref)*total_chassis_wheel_speed_limit;
    }
}

int Check_Vcap_recieve(void)  //������0.1sû���յ���Ϣʱ�л�����0,���򷵻�1
{
  if(not_receive_time<2500)
    not_receive_time++;
  if(not_receive_time>1000)
    return 0;
  else
    return 1;
}

void chassis_param_init(void)//���̲�����ʼ��
{
		memset(&chassis, 0, sizeof(chassis_t));
		chassis.ctrl_mode      = CHASSIS_STOP;
		chassis.last_ctrl_mode = CHASSIS_RELAX;
	
		chassis.position_ref = 0;
		chassis_rotate_flag = 0;
	  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,pid_outmax, pid_ioutmax,pid_output_deadband,pid_p,pid_i, pid_d); //24 0.3 10    38.0f,3.0f, 40
    }
		PID_struct_init(&pid_chassis_angle, POSITION_PID, MAX_CHASSIS_VR_SPEED, 30, 0, 2.0,0.0f,3.0f);//3.0,0.001,27);
}
