#include "main.h"

gimbal_t gim;

//�������
Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;
float Delta_Dect_Angle_Yaw;
float Delta_Dect_Angle_Pit;
float	Machine_Yaw_Compensation = 0;
float	Machine_Pitch_Compensation = 0;
float Army_Speed_Yaw,Army_Speed_Pit;
float yaw_speed_ratio=0;
float Speed_Distence_Compensation = 0;   //���ٺ;����pitch�Ჹ��
float now_distance = 0;
float last_distance = 0;
float Rotate_compensation = 0;
uint8_t Safe_flag=0;
auto_angle_data_t auto_angle_data;
auto_shoot_mode_e  autoshoot_mode = NORMAL_SHOOT ;//0���飬3ǰ��վ
#define BACK_CENTER_TIME 1500

int32_t init_pitch_round_cnt=0;

int yaw_temp_time = 0;
void gimbal_task(void)
{
	  switch (gim.ctrl_mode)//��˫�� ���ǶȻ���ֵ
    {
    case GIMBAL_INIT://��̨��ʼ��
				init_mode_handle();
			break;
    case GIMBAL_FOLLOW_ZGYRO://
				close_loop_handle();
			break;
    case GIMBAL_AUTO_ANGLE:  //����ģʽ
				auto_angle_handle();
			break;
    case GIMBAL_NO_ARTI_INPUT:  
//				auto_angle_task();
			break;

    default:
			gimbal_stop();         //ֹͣ
			break;
    }

		if(gimbal_is_controllable())   //����̨������������
		{
			//�����Ǹ��ݲ�ͬ�����ģʽ������̨�����ͬ�Ĳο�ֵ
			if(autoshoot_mode == NORMAL_SHOOT)
			{
				if(gim.ctrl_mode==GIMBAL_INIT)
				{
					pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
					pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
					cascade_pid_ctrl(pid_yaw.out,pid_pit.out,yaw_Gyro,-roll_Gyro);//
					pid_calc(&pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
					pid_calc(&pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
				}
				else
				{
					pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
					pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
					cascade_pid_ctrl(pid_yaw.out,pid_pit.out,yaw_Gyro,-roll_Gyro);//
					pid_calc(&pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
					pid_calc(&pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
				}
				Set_Pitch_Current(CAN1,(int16_t)pid_pit_speed.out);
				Set_Yaw_Current(CAN2,-(int16_t)pid_yaw_speed.out);
			}
			else if(autoshoot_mode == AUTO_ANGLE_SHOOT)   
			{
				pid_calc(&pid_yaw_auto_angle, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
				cascade_pid_ctrl(pid_yaw_auto_angle.out,pid_pit_auto_angle.out,yaw_Gyro,-roll_Gyro);//pid_pit_auto_angle.out
				pid_calc(&pid_yaw_speed_auto_angle, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
				if(auto_angle_shot_flag)
					Set_Pitch_Current(CAN1,(int16_t)auto_angle_speed.out);
				else
					Set_Pitch_Current(CAN1,(int16_t)auto_angle_speed.out);
					Set_Yaw_Current(CAN2,-(int16_t)pid_yaw_speed_auto_angle.out);
			}
		  else if(autoshoot_mode == AUTO_STATION_SHOOT)    //����
			{
				pid_calc(&pid_yaw_station, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
				pid_calc(&pid_pit_station, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
				cascade_pid_ctrl(pid_yaw_station.out,pid_pit_station.out,yaw_Gyro,-roll_Gyro);   //�ٶȸ�ֵ���̡���getֵΪ�����ǵ�ֵ   setֵΪ
				pid_calc(&pid_yaw_speed_station, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
				pid_calc(&pid_pit_speed_station, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
				Set_Pitch_Current(CAN1,-(int16_t)pid_pit_speed_station.out);
				Set_Yaw_Current(CAN2,-(int16_t)pid_yaw_speed_station.out);
			}
        else 
			{
				pid_calc(&pid_yaw_follow, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);	//����ǶȻ����+Gimbal_Auto_Shoot.Ball_Delay_Compensation
				pid_calc(&pid_pit_follow, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);	//����ǶȻ����
				cascade_pid_ctrl(pid_yaw_follow.out,pid_pit_follow.out,yaw_Gyro,-roll_Gyro);
//					gim.pid.yaw_speed_ref=pid_yaw_speed_follow.out*yaw_speed_ratio+Army_Speed_Yaw+pid_yaw_follow.out;
				pid_calc(&pid_yaw_speed_follow, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
				pid_calc(&pid_pit_speed_follow, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
				Set_Pitch_Current(CAN1,-(int16_t)pid_pit_speed_follow.out);
				Set_Yaw_Current(CAN2,-(int16_t)pid_yaw_speed_follow.out);
			}
    }
		else
    {
      gimbal_stop();
			cam_gim_stop();
			ddt_SetMotor(0);
			Set_Pitch_Current(CAN1,0);
			Set_Yaw_Current(CAN2,0);
    }
}

float now_pid_out;
float last_pid_out;
float ll_pid_out;
float pid_tend;

void cascade_pid_ctrl(float yaw_ref,float pit_ref,float yaw_fdb,float pit_fdb)
{
  gim.pid.yaw_speed_ref = yaw_ref;//pid_yaw.out;
  gim.pid.pit_speed_ref = pit_ref;//pid_pit.out;
  gim.pid.yaw_speed_fdb =	yaw_fdb;//yaw_Gyro;
  gim.pid.pit_speed_fdb =	pit_fdb;//-roll_Gyro;
}

int32_t init_rotate_num = 0;
int32_t init_Yaw_angle = 0;
int32_t init_get_time = 0;
int init_count=0;
float pitch_middle = 0;
void init_mode_handle(void)     //��ʼ����̨
{
  aim_init_flag = 0;
	camera_gimbal_mode = INIT;

      PID_struct_init(&pid_pit, POSITION_PID, 1000, 10, 0, -20,-0.1,-10);//hi220//-9,-0.1,3		//MF5015+����˿��
  		PID_struct_init(&pid_pit_speed, POSITION_PID, 800, 100, 0, 12 , 0, 50);//8,0.1,18

  gim.pid.pit_angle_fdb = -roll_Angle;//GMPitchEncoder.ecd_angle;   //����Ϊ�� //getֵΪ�����ֵ
  gim.pid.pit_angle_ref = 0;                                           //refֵΪ������ 0
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;                     //getֵΪ�����ֵ

  init_rotate_num=GMYawEncoder.ecd_angle/360.0f;                       //������ת��Ȧ��
  init_Yaw_angle = -init_rotate_num*360;                               //yaw��ĽǶ�ΪȦ��*360  ����setֵ��ӵ���getֵһ��Ȧ����Σ�

	//����yaw���setֵ
  gim.pid.yaw_angle_ref = init_Yaw_angle;//-GMYawEncoder.ecd_angle*(1 -GMYawRamp.Calc(&GMYawRamp));//��ʼ����ʱ���������ת����,��Ϊ���õ��ǵ��̵�ת��,������̨����λ

	//ͨ���ӻ�ת�����ĽǶ�
  if((gim.pid.yaw_angle_ref-gim.pid.yaw_angle_fdb)>=181)
    gim.pid.yaw_angle_ref-=360;
  else if((gim.pid.yaw_angle_ref-gim.pid.yaw_angle_fdb)<-179)
    gim.pid.yaw_angle_ref+=360;
	
//	init_count++;init_count>200||
	//�����̨�ĽǶȴﵽ��ʼ��������������if�������ʼ���ɹ�				 .
  if ((gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref >= -0.5f && gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref <= 0.5f
				&& gim.pid.pit_angle_fdb-gim.pid.pit_angle_ref >= -1.5f && gim.pid.pit_angle_fdb-gim.pid.pit_angle_ref <= 1.5f))
    {
			init_get_time++;//Ӧ��ûɶ��
//			init_count=0;
			init_pitch_round_cnt=GMPitchEncoder.round_cnt;
			if(init_get_time==1)     
			{
				gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;            //ģʽת��Ϊ���̸�����̨ģʽ
				chassis.follow_gimbal = 1;                      //��־λ��1
				gim.pid.yaw_angle_fdb = yaw_Angle;              //
				GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//����������Ϊ��
//				GimbalRef.pitch_angle_dynamic_ref = roll_Angle;	//����������Ϊ��

        GimbalRef.pitch_angle_dynamic_ref = GMPitchEncoder.ecd_angle;
        pitch_middle = GMPitchEncoder.ecd_angle;

				chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
				autoshoot_mode = NORMAL_SHOOT;
				auto_angle_key_flag=0;
				init_get_time = 0;
				dir_mov_flag=1;
				chassis.position_ref=0;
				position_flag=1;
				position_set_flag=0;

    PID_struct_init(&pid_pit, POSITION_PID, 400, 20, 0, 5.0f,0.02f,40);								//MF5015+������	2.25f,0.05f,0.7f
		PID_struct_init(&pid_pit_speed, POSITION_PID, 1000, 50, 0, 2,0.01f,2);//2500	2.25f,0.05f,0.7f
			}
    }
}

float new_locationx_last,new_locationy_last,x_temp=0,y_temp=0;

double shoot_angle_speed = 0;
double distance_s, distance_x, distance_y;
double x1, x2, x3, x4;
float angle_tan, shoot_radian, auto_shoot_angle;
int compensate=220;//100
int pit_compensate=0;
float pit_station_compensate=0,yaw_station_compensate=5;

float pit_const=-0.5;
float yaw_compensate=0;
float last_yaw_angle,last_pitch_angle_ref,last_pitch_angle_fdb;
float auto_time=1;
float x_diff=0;
int delay_flag=0,break_time=0,delay_time=18;
int dir_diff=0,last_dir_x=0,last_dir_yaw=0,compensate_time1=0;
float factor=-1.0;
float dis_point=0;
int pit_angle_lim=0;

void close_loop_handle(void)    
{
	static u8 flag_lost = 0;

	//���ģʽ
  switch (autoshoot_mode)
  {
    case NORMAL_SHOOT:               //��ͨ���ģʽ
    {
      gim.pid.yaw_angle_fdb = yaw_Angle;
      gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
			//getֵ�ֱ�Ϊ����������ǵĶ�ֵ
      gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;//GMPitchEncoder.ecd_angle;              
			//setֵΪ�ο�ֵ
      gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
			//�ǳ���ͨ�Ķ�ֵ
			last_pitch_angle_fdb=GMPitchEncoder.ecd_angle;
			last_pitch_angle_ref=gim.pid.pit_angle_ref;
    }
    break;
		case AUTO_SHOOT:                 //����
    {
			//getֵ��Ϊ�����ǵ�ֵ
			gim.pid.pit_angle_fdb = roll_Angle;         
      gim.pid.yaw_angle_fdb = yaw_Angle;
			//setֵΪ
			gim.pid.yaw_angle_ref = new_location.x+yaw_compensate;
			gim.pid.pit_angle_ref = new_location.y;//+pit_compensate;
    }
		break;
		case AUTO_STATION_SHOOT:                      //���������
		{
			gim.pid.pit_angle_fdb = -roll_Angle;
      gim.pid.yaw_angle_fdb = yaw_Angle;
			gim.pid.yaw_angle_ref = new_location.x+yaw_station_compensate;     //ǰ�����ת���ĽǶ�
			gim.pid.pit_angle_ref = new_location.y+pit_station_compensate;     //compensateӦ����Ϊ�˵����Ƕ��õĲο�ֵ
//			gim.pid.yaw_angle_ref = new_location.x;     //ǰ�����ת���ĽǶ�
//			gim.pid.pit_angle_ref = -new_location.y;     //compensateӦ����Ϊ�˵����Ƕ��õĲο�ֵ
		}
		break;
    default:
    break;
  }
}

//����ģʽ

void auto_angle_handle(void)
{
	if(auto_angle_shot_flag==0)   //��־λΪ0����ʾʹ�ñ�������ֵ
	{
      PID_struct_init(&auto_angle, POSITION_PID, 200, 100, 0, 1.5f,0.02f,2);								//MF5015+������	2.25f,0.05f,0.7f
		  PID_struct_init(&auto_angle_speed, POSITION_PID, 500, 50, 0, 2,0.01f,2);//2500	2.25f,0.05f,0.7f
      auto_angle.set=auto_angle_data.pit_angle_offset_1;//����ģʽMF5015˿�˽ṹʹ�ñ���
			auto_angle.get=GMPitchEncoder.ecd_angle-auto_angle_data.pit_angle_raw;
	}
	else if(auto_angle_shot_flag==1)   //��־λΪ1����ʾʹ�������ǵ�ֵ//��ಹ��
  {
    PID_struct_init(&auto_angle, POSITION_PID, 1000, 10, 0, 20,0.1,10);//hi220//-9,-0.1,3		//MF5015+����˿��
  	PID_struct_init(&auto_angle_speed, POSITION_PID, 1000, 100, 0, 10 ,0,50);//8,0.1,18
		auto_angle.get = roll_Angle;       //���򷵻������ǵ�ֵ
    auto_angle.set = angle_cal;
		if(fabs(auto_angle.set-auto_angle.get)<0.03)          //�������ֵ��Ŀ��ֵ�Ĳ�ֵ��������������
		{
			auto_angle_shot_flag = 0;                                           //��־λ��0
			shot_radar_flag = 0;                                                //�������־λ
			auto_angle_data.pit_angle_raw=GMPitchEncoder.ecd_angle;//;-roll_Angle
			auto_angle_data.yaw_angle_raw=-GMYawEncoder.ecd_angle;
      cam_angle = roll_Angle;
		}
	}
	    pid_calc(&auto_angle,auto_angle.get,auto_angle.set);
			pid_calc(&auto_angle_speed,-roll_Gyro,auto_angle.out);//GMPitchEncoder.rate_rpm
      
			gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;                                                 //����ֵΪ�����ת��
			gim.pid.yaw_angle_ref = auto_angle_data.yaw_angle_raw + auto_angle_data.yaw_angle_offset;        //�ο�ֵΪƫ������  ��ʼֵ
}


//��ֹ̨ͣ
void gimbal_stop(void)
{
		gim.ctrl_mode = GIMBAL_RELAX;     //ģʽת��
		gimbal_back_param();              //��̨���³�ʼ��
		pid_clr(&pid_pit);
		pid_clr(&pid_pit_speed);
		pid_clr(&pid_yaw);
		pid_clr(&pid_yaw_speed);
		pid_clr(&pid_pit_follow);
		pid_clr(&pid_pit_speed_follow);
		pid_clr(&pid_yaw_follow);
		pid_clr(&pid_yaw_speed_follow);
}

float AvgFilter(float new_value)//��YAW����ٶ���ǰ�ĸ�������ƽ��ֵ
{
	float avg_value;
	float sum = 0;
	static float value[FILTER_NUM] = {0};
	for(int i = 0; i < FILTER_NUM - 1 ; i ++)
	{
			value[i] = value[i + 1];
			sum += value[i];
	}
	value[FILTER_NUM - 1] = new_value;
	sum += value[FILTER_NUM - 1];
	avg_value = sum / FILTER_NUM;
	return avg_value;
}

float lab_shoot_task ( float shoot_speed , float distance , float angle ) //����
{
    float shoot_distance = 0 , distance_x = 0 , distance_y = 0;
    float shoot_angle_speed = 0;
    float shoot_angle = 0;
    float shoot_radian = 0;
    float x1, x2, x3, x4;

    //�Զ�����
    //��֪��
    shoot_angle_speed = shoot_speed;
    shoot_distance = distance;
    shoot_angle = angle;
    distance_x = ( cos ( shoot_angle * ANGLE_TO_RAD ) * shoot_distance );
    distance_y = ( sin ( shoot_angle * ANGLE_TO_RAD ) * shoot_distance );

    //�Ƕȹ�ʽ
    x1 = shoot_angle_speed * shoot_angle_speed;
    x2 = shoot_distance * shoot_distance;
    x3 = sqrt ( x2 - ( 19.6f * x2 * ( ( 9.8f * x2 ) / ( 2.0f * x1 ) + distance_y ) ) / x1 );
    x4 = 9.8f * x2;
    shoot_radian = ( x1 * ( distance_x - x3 ) ) / ( x4 );
    shoot_angle = atan ( shoot_radian ) * RAD_TO_ANGLE;

    return shoot_angle;
}

//ģʽPID��ʼ��
void gimbal_param_init(void)
{
		memset(&gim, 0, sizeof(gimbal_t));
		gim.ctrl_mode      		 = GIMBAL_RELAX;
		gim.last_ctrl_mode 		 = GIMBAL_RELAX;
		gim.input.ac_mode      = NO_ACTION;
		gim.input.action_angle = 5.0f;
//PID_struct_init(*pid, mode, maxout, intergral_limit, output_deadband, kp, ki, kd);
//-----------------��ͨģʽ---------------------------	

    PID_struct_init(&pid_pit, POSITION_PID, 1000, 10, 0, -20,-0.1,-10);//hi220//-9,-0.1,3		//MF5015+����˿��
  	PID_struct_init(&pid_pit_speed, POSITION_PID, 1000, 100, 0, 10 ,0,50);//8,0.1,18

		PID_struct_init(&pid_yaw, POSITION_PID, 3000, 20, 0, 13 ,0.15f, 10);//12, 0.5, 8					//MF9025
		PID_struct_init(&pid_yaw_speed, POSITION_PID, 2000, 50, 0, 12, 0.5f, 0 );//10, 0.1 , 15


//-----------------����ģʽ---------------------------

		PID_struct_init(&auto_angle, POSITION_PID, 200, 100, 0, 1.5f,0.02f,2);								//MF5015+������	2.25f,0.05f,0.7f
		PID_struct_init(&auto_angle_speed, POSITION_PID, 500, 50, 0, 2,0.01f,2);//2500	2.25f,0.05f,0.7f

		PID_struct_init(&pid_yaw_auto_angle, POSITION_PID,3000, 20, 0, 17, 0.1 , 2);							//MF9025//����������
		PID_struct_init(&pid_yaw_speed_auto_angle, POSITION_PID,2000, 50, 0, 19, 0.1, 1.9f);
//		PID_struct_init(&pid_yaw_auto_angle, POSITION_PID,3000, 20, 0, 12, 0.1 , 2);							//MF9025
//		PID_struct_init(&pid_yaw_speed_auto_angle, POSITION_PID,2000, 50, 0, 10.5, 0.1, 1.9f);

//-----------------����ģʽ---------------------------
		PID_struct_init(&pid_pit_follow, POSITION_PID, 2000, 3, 0, 15,0.1,0);
		PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 6000, 0, 10,0.17,0);	
		PID_struct_init(&pid_yaw_follow, POSITION_PID, 3000, 17, 0, 21.0f, 0.25f, 15.0f);
		PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 26000, 3800, 0, 25.0f,  2.1f, 21);
	
		PID_struct_init(&pid_pit_station, POSITION_PID, 200, 100, 0, 10.0f,0.33f,25.0f);
		PID_struct_init(&pid_pit_speed_station, POSITION_PID, 20000,2700, 0, 100.0f, 0.4f,30.0f);
		PID_struct_init(&pid_yaw_station, POSITION_PID,1000,21, 0, 110.0f,0.34f,12.0f );
		PID_struct_init(&pid_yaw_speed_station, POSITION_PID,20000,4000, 0, 13.0f,0.50f,17);
}


//��Ҫ�˽�б�µ�ԭ����
void gimbal_back_param(void)     
{
	  //б�³�ʼ��
		GMPitchRamp.SetScale(&GMPitchRamp,BACK_CENTER_TIME/GIMBAL_PERIOD);
		GMYawRamp.SetScale(&GMYawRamp, BACK_CENTER_TIME/GIMBAL_PERIOD);
		GMPitchRamp.ResetCounter(&GMPitchRamp);
		GMYawRamp.ResetCounter(&GMYawRamp);
		//��̨�����Ƕȳ�ʼ��                           
		GimbalRef.pitch_angle_dynamic_ref = 0.0f;
		GimbalRef.yaw_angle_dynamic_ref   = 0.0f;
}

