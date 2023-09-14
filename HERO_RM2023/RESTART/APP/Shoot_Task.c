#include "main.h"

u8 over_heat=0;//1������ 0��������
u8 press_l_first_in=1; //1���ν���0�ǳ��ν���
int max_back_lock_time=0;//��ǰ��תʱ��
int max_forward_lock_time=0;//�ͺ��תʱ��
float residue_heart;//ʣ���������
shoot_state_e shot_state=NOMAL;
int over_hot_motor=0;
int shoot_flag=1;//

uint16_t frictionSpeed;		//42mm����
int frictionSpeed1=3200;//���ٱ궨������ʹ�ñ���

int init_speed_flag=1;

//Ħ���ַ���ת����
int friction_back_flag=0;
int friction_back_count=0;
int friction_rotor_may_lock=0;

int friction_rotor_error=100;
int BUS1_CMEncoder_Rate1=0;
int BUS1_CMEncoder_Rate2=0;
int BUS1_CMEncoder_readtime2=0;
int friction_rotor_DIV=11;

//���ģʽ��������ٶȣ�ͨ������ϵͳ�õ����ݣ������л�ģʽ��
static void SwitchModeShoot(void)
{
  if(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==10)
    frictionSpeed=FRICTION_SPEED_10;
  else if((aim_scope_flag==0) && (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==16))
    frictionSpeed=FRICTION_SPEED_16;
	else if (aim_scope_flag==1)		//���������ǰ��վ
		frictionSpeed = 3500;
	else
		frictionSpeed=FRICTION_SPEED_10;
}

int sac_time=0;
int sac_flag=0;
int raw_value=0;
int last_raw_value=0;
u8 shot_flag=0;
u8 shot_judge_flag=1;
u32 shot_space_time=0;

void shot_task(void)
{
	test_mode_task();
	SwitchModeShoot();
	heat1_limit();//��������
	shoot_bullet_handle();
	shoot_friction_handle();
}

u8 bullet_flag=0;
u32 count_time=0;
u32 bullet_flag_time=0;

void test_mode_task(void)
{
	  //������û�е磬�����ۼ�
		if(now_current==last_current)
			count_time++;
		else
			count_time=0;
		if(count_time>=3000)
			bullet_flag=1;
		if((bullet_flag)&&(count_time<=30))  //����ȶϵ��ˣ���ͨ�磬��ʼ�ۼ�
			bullet_flag_time++;
		else
			bullet_flag_time=0;
		if(bullet_flag_time==50)    //���½����ʼ��
		{
			bullet_flag=0;
			bullet_flag_time=0;
			position_flag=1;
			position_set_flag=0;
		}
		
		last_raw_value=raw_value;                          //����valueֵ
		raw_value=BUS1_CM1Encoder.last_raw_value;          //��ȡ�µ�value
		
		//����ϵͳ��ֵ���ж�ͨ����  ����  Ħ���ֵ��ٶȴﵽһ��ֵ�������ж��ϵ���
//		if((now_count-last_count)>-3||judge_rece_mesg.game_robot_state.mains_power_shooter_output==0)//||judge_rece_mesg.bullet_remaining.bullet_remaining_num_42mm==0)//-5
//			sac_flag=1;
//		else
//			sac_flag=0;

		if(shot_judge_flag==0)
			shot_space_time++;
		if(shot_space_time==700)
		{
			shot_space_time=0;
			shot_judge_flag=1;
		}
		if(shot_judge_flag==1)
		{
			if(new_location.shot_flag==1)
				shot_flag=1;
			else
				shot_flag=0;
		}
}

//��������
void heat1_limit(void)
{
//-----------ʣ����������(���-�ۼ�)----------------
//-----------������ģʽ---------------------------
  residue_heart=(judge_rece_mesg.game_robot_state.shooter_id1_42mm_cooling_limit           //��ȡ��ص�����
                       -judge_rece_mesg.power_heat_data.shooter_id1_42mm_cooling_heat);

  if(residue_heart>=100)
      over_heat=0;                     //������־λ
  else
    {
      over_heat=1;
			new_location.shot_flag=0;         
    }
}

#define LOCK_SPEED 100 //��ת�ٶȴӶ�תλ����Ŀ��λ��ת          ������ת��ķ�ת
#define TIME_TO_LOCK_STATE  100 //ȷ����ת��ʱ��                �����������ﵽ���ֵ�ǣ�ȷ����ת
#define MAX_FORWARD_LOCK_TIME 500 //��������л�ģʽ��λ�ú�Ķ�תʱ��       ��������������ʲô��˼����������������������������
#define MAX_BACK_LOCK_TIME 300 	 //��������л�ģʽ��λ�ú�Ķ�תʱ��

int last_taget_angle=0;
char ignore_heat = 1;
int stop_bullet_flag = 0;
int stop_friction_flag = 0;
int turn_friction_flag = 0;

u8 lock_flag=0;//������������־λ
int lock_time=0;//�������嵯��ʱ
int lock_cilent_flag=0;//�ͻ���UI��ʾ��־λ

u8 return_flag=0;
u8 return_time=0;
u8 position_time=0;
u8 position_set_flag=0;            //�����ʼ������0   ���̿���������������1  ����Ϊ0ʱ�ؿ�  ������ʾ�Ƿ���Է���ı�־λ
int pid_compenstae=800;
float k_speed=0.005;

float taget_angle_2=0;//(��������)��תǰ��Ŀ��Ƕ�
float Torque_current=-40;//��ֹ�²��̵������
float poke_max_out=6000;//�²���pid����޷�
int set_time=0;

//����		
void shoot_bullet_handle(void)
{
	if(position_flag)            //��ʼ����ʱ�����
	{
			taget_angle_2=BUS1_CM5_Poke_Encoder.rotor_out;
			position_flag=0;
			position_time=0;
			position_set_flag=1;
			pid_clr(&pid_shoot_bullet_position_angle_loop_2);
			pid_clr(&pid_shoot_bullet_position_speed_loop_2);
	}
	else if((!sac_flag)&&(BUS1_CM1Encoder.rate_rpm<-1000&&BUS1_CM2Encoder.rate_rpm>1000)&&(!position_flag)&&(friction_rotor==friction_on))
    //���	�ϵ���	����	����ﵽһ����ת�٣�����Ħ���֣�	����	��ʼ����־λΪ0	�ſ��Է���
	{
		static int l_key_delay_time=0;
		l_key_delay_time++;
		pid_calc(&pid_shoot_bullet_position_speed_loop,BUS2_CM5_Poke_Encoder.rate_rpm,POKE_SPEED);
		
	//��ֹ�������
		if(press_l_first_in==0)
			pid_shoot_bullet_position_speed_loop.max_out=poke_max_out;
		else
		{
			if(pid_shoot_bullet_position_speed_loop.out<-4200 && abs(BUS2_CM5_Poke_Encoder.rate_rpm)<20)
				set_time++;
			else	
				set_time = 0;
			if(set_time == 2)
				pid_shoot_bullet_position_speed_loop.max_out=2000;
		}

		if(BUS2_CM5_Poke_Encoder.temperature >= 80)
		{
			stop_bullet_flag = 1;
			over_hot_motor=1;
		}
		if(BUS2_CM5_Poke_Encoder.temperature <= 60)
			stop_bullet_flag = 0;

		switch(shot_state)
		{
			case NOMAL:
			{
				//�¶�û������
				if((over_heat==0)||(ignore_heat==1))////////&&BUS1_CM1Encoder.rate_rpm<-500&&BUS1_CM2Encoder.rate_rpm>500
				{
					//������������ǵ���ģʽʱ���������ģʽΪ�������ʱ���������ģʽ
					if(((shootState==SHOOTING)||((shot_flag)&&(autoshoot_mode==AUTO_STATION_SHOOT)))&&(GetInputMode()!= STOP)&&(friction_rotor==1))//������ʱ
					{
						if((press_l_first_in==1)&&(l_key_delay_time>150))
						{
							//��������
							taget_angle_2 += ONE_POKE_ANGLE_2;                               //Ŀ��Ƕȵ���ԭ�Ƕȼ�ȥÿһ�����跢��ĽǶ�
							pid_shoot_bullet_position_angle_loop_2.set=taget_angle_2;        //λ�õ�setֵΪĿ��Ƕ�
							pid_shoot_bullet_position_angle_loop_2.iout=0;                 //��������

							press_l_first_in=0;                                                                         
							l_key_delay_time=0;                                          //�����ʱ
							shot_flag=0;                                                 
							shot_judge_flag=0;
						}
					}
					else
					{
						press_l_first_in=1;                                           
					}
				}
			pid_shoot_bullet_position_angle_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_out;
			pid_calc(&pid_shoot_bullet_position_angle_loop_2, pid_shoot_bullet_position_angle_loop_2.get, pid_shoot_bullet_position_angle_loop_2.set); //2006
			pid_shoot_bullet_position_speed_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_speed+pid_shoot_bullet_position_speed_loop_2.out*k_speed;
			pid_calc(&pid_shoot_bullet_position_speed_loop_2, pid_shoot_bullet_position_speed_loop_2.get, pid_shoot_bullet_position_angle_loop_2.out);
			}
			break;
			case LOCK:            //����Ƕ�תģʽ
			{
			}
			break;
		}
		//�ؿ�
		if(GetInputMode() == STOP)
			{
				Set_GM_CM_Current(CAN2,0,0,0,0);
				Set_GM_CM_Current(CAN1,0,0,0,0);
			}
		else //���ؿ�
			{
				//���û�п�����ת���̡������������Ͳ����������ʲô�����𣿣���������������������
				if(shot_state!=LOCK)
				{
					if(stop_bullet_flag||bullet_flag ||friction_rotor !=friction_on)//���ȡ�û���ۼӡ�������������������������������������������������������������������������������������
					{
						Set_GM_CM_Current(CAN2,0,0,0,0);
						Set_GM_CM_Current(CAN1,0,0,0,aim_scope_speed.out);
					}
					else //����
					{
						Set_GM_CM_Current(CAN2,pid_shoot_bullet_position_speed_loop.out,0,0,0);//pid_shoot_bullet_position_speed_loop.out
						Set_GM_CM_Current(CAN1,pid_shoot_bullet_position_speed_loop_2.out,0,0,aim_scope_speed.out);
					}
				}
			}
	}
	else   //û�п���Ħ���֣��Ҳ�Ϊ��ʼ��ʱ���롪��Ϊ�˲��ò��̶�����
	{
//		if(position_set_flag)
//		{
//			if(GetInputMode() == STOP)
//			{
//				pid_clr(&pid_shoot_bullet_position_speed_loop);
//				pid_clr(&pid_shoot_bullet_position_speed_loop_2);
//				pid_shoot_bullet_position_speed_loop_2.iout=0;//�������̻�������
//				pid_shoot_bullet_position_speed_loop.out=0;//��������
//			}
//			//��������
			pid_shoot_bullet_position_angle_loop_2.set=taget_angle_2;
			pid_shoot_bullet_position_angle_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_out;
			pid_calc(&pid_shoot_bullet_position_angle_loop_2,pid_shoot_bullet_position_angle_loop_2.get, pid_shoot_bullet_position_angle_loop_2.set);
			pid_shoot_bullet_position_speed_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_speed;
			pid_calc(&pid_shoot_bullet_position_speed_loop_2,pid_shoot_bullet_position_speed_loop_2.get,pid_shoot_bullet_position_angle_loop_2.out);

		  
//			else
//				{
					
//				}
      if(GetInputMode() == STOP)
				{
					Set_GM_CM_Current(CAN2,0,0,0,0);
					Set_GM_CM_Current(CAN1,0,0,0,aim_scope_speed.out);
					pid_clr(&pid_shoot_bullet_position_speed_loop);
					pid_clr(&pid_shoot_bullet_position_speed_loop_2);
				}else 
        {
          Set_GM_CM_Current(CAN1,pid_shoot_bullet_position_speed_loop_2.out,0,0,aim_scope_speed.out);
					Set_GM_CM_Current(CAN2,0,0,0,0);
        }
//    Set_GM_CM_Current(CAN2,0,0,0,0);
//		Set_GM_CM_Current(CAN1,0,0,0,aim_scope_speed.out);
	}
}

int change_pid_flag=0;
int stable_time=0;
u8 friction_normal_flag=0;

//Ħ���ֲ���
void shoot_friction_handle(void)
{
		switch(friction_rotor)
		{
			case friction_stop:   //ֹͣ
			{
				friction_rotor =0;
        pid_rotate[2].set=0;
        pid_rotate[1].set=0;
			}
			break;
			case friction_on:     //��ת
			{
			  pid_rotate[1].set=-(frictionSpeed)*frictionRamp.Calc(&frictionRamp);
				pid_rotate[2].set= (frictionSpeed)*frictionRamp.Calc(&frictionRamp);
			}
			break;
			case friction_slow:   //���٣���ת
			{
				pid_rotate[1].set=-(frictionSpeed-(frictionSpeed)*frictionRamp.Calc(&frictionRamp));
				pid_rotate[2].set= (frictionSpeed-(frictionSpeed)*frictionRamp.Calc(&frictionRamp));
			}
			break;
			case friction_back:   //��ת
			{
				pid_rotate[1].set=(frictionSpeed)*frictionRamp.Calc(&frictionRamp);
				pid_rotate[2].set=-(frictionSpeed)*frictionRamp.Calc(&frictionRamp);
			}
			break;
			default:               //ֹͣ
			{
				friction_rotor =0;
				pid_rotate[2].set=0;
				pid_rotate[1].set=0;
			}
		}

		BUS1_CMEncoder_readtime2++;
		BUS1_CMEncoder_Rate1+=BUS1_CM1Encoder.rate_rpm;
		BUS1_CMEncoder_Rate2+=BUS1_CM2Encoder.rate_rpm;

		if(BUS1_CMEncoder_readtime2>=friction_rotor_DIV)//�ֱ����ٶȵ�ƽ��ֵ
		{
			pid_rotate[1].get=BUS1_CMEncoder_Rate1/friction_rotor_DIV;//BUS1_CM1Encoder.rate_rpm;//BUS1_CMEncoder_Rate1/friction_rotor_DIV;
			pid_rotate[2].get=BUS1_CMEncoder_Rate2/friction_rotor_DIV;//BUS1_CM2Encoder.rate_rpm;//BUS1_CMEncoder_Rate2/friction_rotor_DIV;
			BUS1_CMEncoder_readtime2=0;
			BUS1_CMEncoder_Rate1=0;
			BUS1_CMEncoder_Rate2=0;
		}
		
		//������ٶȴﵽҪ��Ħ����������־λ��1
		if((friction_rotor==friction_on)&&(BUS1_CM1Encoder.filter_rate<-30)&&(BUS1_CM2Encoder.filter_rate>30))
		{
			friction_normal_flag = 1;
			friction_rotor_may_lock=0;
		}
		else if((friction_rotor==friction_on)&&(BUS1_CM1Encoder.filter_rate>-5)&&(BUS1_CM1Encoder.filter_rate<5)&&(BUS1_CM2Encoder.filter_rate<5)&&(BUS1_CM2Encoder.filter_rate>-5))
		{
			friction_rotor_may_lock++;
			if(friction_rotor_may_lock==50)
			{
				friction_rotor=friction_back;
				friction_back_flag=1;
				friction_rotor_may_lock=0;
			}
		}
		else if(friction_rotor==friction_stop)         //���ģʽΪֹͣ
		{
			friction_normal_flag = 0;
		}
		
		//��ֹ��ת
		if(friction_back_flag==1)
		{
			friction_back_count++;
			if(friction_back_count==400) 
			{
				friction_back_flag=0;
				friction_rotor=friction_on;
				friction_back_count=0;
			}			
		}

		//PID����
		pid_calc(&pid_rotate[1],pid_rotate[1].get, pid_rotate[1].set);
		pid_calc(&pid_rotate[2],pid_rotate[2].get, pid_rotate[2].set);
		float current1,current2;
		current1 = pid_rotate[1].out;
		current2 = pid_rotate[2].out;
//		current1 = LADRC_control_task(&ladrc_num,pid_rotate[1].set,pid_rotate[1].get);
//    current2 = LADRC_control_task(&ladrc_angle,pid_rotate[2].set,pid_rotate[2].get);
		//��ֹ�������
		if(BUS1_CM1Encoder.temperature >= 90||BUS1_CM2Encoder.temperature >= 90)
		{
			stop_friction_flag = 1;
			over_hot_motor=2;
		}
		if(BUS1_CM1Encoder.temperature <= 70&&BUS1_CM2Encoder.temperature <= 70)
			stop_friction_flag = 0;
		
		if(GetInputMode() == STOP)
			Set_CM_Speed(CAN1,0,0,0,0);
	 	else
		{
			if(stop_friction_flag==1||turn_friction_flag==0)
				Set_CM_Speed(CAN1,0,0,0,0);
			else
        Set_CM_Speed(CAN1,pid_rotate[1].out-friction_rotor_error,pid_rotate[2].out+friction_rotor_error,0,0);//Set_CM_Speed(CAN1,pid_rotate[1].out-pid_compenstae,pid_rotate[2].out+pid_compenstae,0,0);
		}
}

	float last_pit_angle;
	float last_control_angle;

void friction_lock(void)
{
	if (!lock_flag)
	{
		taget_angle_2-=30;
		lock_flag=1;
	}
	pid_shoot_bullet_position_angle_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_out;
	pid_calc(&pid_shoot_bullet_position_angle_loop_2, pid_shoot_bullet_position_angle_loop_2.get, pid_shoot_bullet_position_angle_loop_2.set); //2006
	pid_shoot_bullet_position_speed_loop_2.get=BUS1_CM5_Poke_Encoder.rotor_speed;//+pid_shoot_bullet_position_speed_loop_2.out*k_speed;
	pid_calc(&pid_shoot_bullet_position_speed_loop_2, pid_shoot_bullet_position_speed_loop_2.get, pid_shoot_bullet_position_angle_loop_2.out);

	pid_calc(&pid_shoot_bullet_position_speed_loop,BUS2_CM5_Poke_Encoder.rate_rpm,LOCK_SPEED);

	if(GetInputMode() == STOP)
	{
		Set_GM_CM_Current(CAN2,0,0,0,0);
		Set_GM_CM_Current(CAN1,0,0,0,0);
	}
	else
	{
		Set_GM_CM_Current(CAN2,pid_shoot_bullet_position_speed_loop.out,0,0,0);
		Set_GM_CM_Current(CAN1,pid_shoot_bullet_position_speed_loop_2.out,0,0,aim_scope_speed.out);
	}
}

//PID��ʼ��
void shot_param_init(void)
{
		Set_GM_CM_Current(CAN1,0,0,0,aim_scope_speed.out);

//  PID_struct_init(&pid_rotate[1], POSITION_PID,15000,1000, 0, 75,0.13f,70);//����Ħ����/7,0.33f,100//80,0.4,10��
//  PID_struct_init(&pid_rotate[2], POSITION_PID,15000,1000, 0, 75,0.13f,70);

  PID_struct_init(&pid_rotate[1], POSITION_PID,15000,1000, 0, 70 , 0.01f ,100);//����Ħ����/7,0.33f,100//75��0��70
  PID_struct_init(&pid_rotate[2], POSITION_PID,15000,1000, 0, 72 , 0.01f ,10);

  PID_struct_init(&pid_shoot_bullet_position_speed_loop, POSITION_PID, poke_max_out, 13000, 0, 5, 0.5, 0);//�²����ٶȻ�
	PID_struct_init(&pid_shoot_bullet_position_angle_loop_2, POSITION_PID, 2000, 0,  0, 120, 5, 10); //��������//120 5 10//1200
  PID_struct_init(&pid_shoot_bullet_position_speed_loop_2, POSITION_PID, 9900, 5500, 0, 20, 0, 0 );
}
