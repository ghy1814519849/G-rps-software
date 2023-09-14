#include "main.h"

double gravity  = 9.7915;
double image_muzzle_angle = 0;//ͼ��ǹ�ڽǶ�
double projectile_speed = 0;//��ʼֵ
float shoot_angle = 25;//̧���Ƕ�
float beta = 0;
float a1 = 0,a2 = 0;
float molecule = 0,denominator = 0;
float projectile_16_speed_offset=14.3;
float projectile_10_speed_offset=9.5;
float speed_compensate = 0.0;
float avg_shot_speed = 14.4;
void chip_task(void)
{
//	  projectile_speed = avg_shot_speed;//judge_rece_mesg.shoot_data.bullet_speed;
//		a1 = -gravity*gravity*shot_radar_dis*shot_radar_dis*arm_cos_f32(beta*ANGLE_TO_RAD)*arm_cos_f32(beta*ANGLE_TO_RAD);
//		a2 = -2*arm_sin_f32(beta*ANGLE_TO_RAD)*gravity*shot_radar_dis*projectile_speed*projectile_speed;
//		molecule = sqrt(a1+a2+projectile_speed*projectile_speed*projectile_speed*projectile_speed)-projectile_speed*projectile_speed;
//		denominator = gravity*shot_radar_dis*arm_cos_f32(beta*ANGLE_TO_RAD);
//		shoot_angle = atan(-molecule/denominator)/ANGLE_TO_RAD;
}

u8 auto_angle_flag = 0;//����ģʽ������־λ
u32 auto_angle_change_time = 0; //�������ʱ���־

u8 accelerate_flag = 0;	//���������
u8 auto_w_flag=1;
u32 auto_w_time =0;
u8 auto_s_flag=1;
u32 auto_s_time =0;
u8 auto_a_flag=1;
u32 auto_a_time =0;
u8 auto_d_flag=1;
u32 auto_d_time =0;

u8 shot_radar_flag = 1; //�������־λ
float shot_angle_offset = 0;
float shot_radar_dis = 0;
float beta_compensate = -0.04;
float dis_compensate = 0;
float shot_angle_compensate = 1.7;//0.4;
float shot_yaw_compensate = 0;

float angle;
pid_t auto_angle;
pid_t auto_angle_speed;

u8 auto_angle_shot_flag = 0;//�����л�������־λ
u8 b_12_speed_flag=0;
float angle_cal = 0;

//����ģʽ�ǶȵĿ���
void auto_angle_control(void)
{
	switch (gim.ctrl_mode)
	{
		case GIMBAL_FOLLOW_ZGYRO:
		{
				camera_gimbal_mode = NORMAL;

			//��ʱһ��ʱ��ſ����л�
      auto_angle_change_time++;
			auto_angle_flag=0;
      if((RC_CtrlData.key.v&KEY_Q)&&(auto_angle_flag == 0)&&(auto_angle_change_time>50))    //����Q�󣬽������ģʽ
        {
          auto_angle_data.pit_angle_raw=GMPitchEncoder.ecd_angle;//;-roll_Angle
			    auto_angle_data.yaw_angle_raw=-GMYawEncoder.ecd_angle;

          auto_angle_change_time=0;
          gim.ctrl_mode = GIMBAL_AUTO_ANGLE;
					chassis.last_ctrl_mode = chassis.ctrl_mode;
					chassis.ctrl_mode = CHASSIS_STOP;
					autoshoot_mode = AUTO_ANGLE_SHOOT;
          auto_angle_flag = 1;
          
          auto_angle_data.yaw_angle_offset=0;
					auto_angle_data.pit_angle_offset_1=0;

					accelerate_flag=0;
					shot_radar_flag = 1;
					switch_mode_flag=1;
          lcam_angle = roll_Angle;
				}
		}
		break;
		case GIMBAL_AUTO_ANGLE:           
		{
			camera_gimbal_mode = AUTO_ANGLE;
			auto_angle_key_flag=1;
			if(auto_angle_flag == 1)
        {
          auto_angle_change_time++;
        }
			if(auto_angle_change_time>50&&auto_angle_flag==1)
				{
					auto_angle_flag=2;
					auto_angle_change_time=0;
				}

			if(RC_CtrlData.key.v&KEY_E)//�����ಹ��
			{
				auto_angle_shot_flag = 1;
        if((judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==16 && judge_rece_mesg.shoot_data.bullet_speed >= 12.0)
						||(judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit==10 && judge_rece_mesg.shoot_data.bullet_speed >= 5.0))
					auto_angle_data.bullet_speed=judge_rece_mesg.shoot_data.bullet_speed;//װ�ò���ϵͳ������Ϊ����ϵͳ�Ĳ����ٶ�
				else
          auto_angle_data.bullet_speed = 14.8;
				if (TF02.Dist <= 2400 && TF02.Strength >= 150)
					auto_angle_data.distance = TF02.Dist/100.0f;
				else
					auto_angle_data.distance = 12.0;
				angle_cal = 22.0;//lab_shoot_task(auto_angle_data.bullet_speed , auto_angle_data.distance, roll_Angle);
				VAL_LIMIT(angle_cal, -10, 45);
        lcam_angle_err = angle_cal - lcam_angle;
			}

			if((RC_CtrlData.key.v&KEY_Q)&&(auto_angle_flag == 2))//�˳�����ģʽ
				{
					switch_mode_flag=3;
					auto_angle_flag = 0;
					auto_angle_key_flag=0;
					auto_angle_shot_flag=0;
					auto_angle_change_time = 0;
					auto_angle_data.pit_angle_offset=0;
					auto_angle_data.yaw_angle_offset=0;

					cam_angle = 0;
					lcam_angle = 0;
					lcam_angle_err = 0;

					auto_angle_data.pit_angle_offset_1=0;//����ģʽMF5015˿�˽ṹʹ�ñ���
					auto_angle.get=0;
					pid_clr(&auto_angle);
					pid_clr(&auto_angle_speed);

					gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;

					if(chassis.last_ctrl_mode==MANUAL_SEPARATE_GIMBAL)
							chassis_separate_flag = 3;
					chassis.ctrl_mode = chassis.last_ctrl_mode;
					autoshoot_mode = NORMAL_SHOOT;
					GimbalRef.pitch_angle_dynamic_ref=GMPitchEncoder.ecd_angle;
					GimbalRef.yaw_angle_dynamic_ref=yaw_Angle;//-GMYawEncoder.ecd_angle
					pid_clr(&pid_yaw_auto_angle);
					pid_clr(&pid_pit_auto_angle);
					pid_clr(&pid_yaw_speed_auto_angle);
					pid_clr(&pid_pit_speed_auto_angle);

					camera_gimbal_mode = NORMAL;
			}
				
			if(RC_CtrlData.key.v&KEY_SHIFT)
				accelerate_flag = 1;
			else
				accelerate_flag = 0;
			
			//WASD��΢����̨����	SHIFT����	������ʱ�����ǲ����ģ�ֻ������̨			
			if((RC_CtrlData.key.v&KEY_W)&&(accelerate_flag == 0)&&(auto_w_flag == 1))
        {
          auto_angle_data.pit_angle_offset_1+=15;
					auto_w_flag=0;
        }
			else if((RC_CtrlData.key.v&KEY_W)&&(accelerate_flag == 1)&&(auto_w_flag == 1))
				{
          auto_angle_data.pit_angle_offset_1+=50;
					auto_w_flag=0;
        }
      if((RC_CtrlData.key.v&KEY_S)&&(accelerate_flag == 0)&&(auto_s_flag == 1))
        {
          auto_angle_data.pit_angle_offset_1-=15;
					auto_s_flag=0;
        }
			else if((RC_CtrlData.key.v&KEY_S)&&(accelerate_flag == 1)&&(auto_s_flag == 1))
				{
          auto_angle_data.pit_angle_offset_1-=50;
					auto_s_flag=0;
        }
      if((RC_CtrlData.key.v&KEY_A)&&(accelerate_flag == 0)&&(auto_a_flag == 1))
        {
          auto_angle_data.yaw_angle_offset-=0.2f;
					auto_a_flag=0;
        }
			else if((RC_CtrlData.key.v&KEY_A)&&(accelerate_flag == 1)&&(auto_a_flag == 1))
				{
          auto_angle_data.yaw_angle_offset-=0.5f;
					auto_a_flag=0;
        }
      if((RC_CtrlData.key.v&KEY_D)&&(accelerate_flag == 0)&&(auto_d_flag == 1))
        {
          auto_angle_data.yaw_angle_offset+=0.2f;
					auto_d_flag=0;
        }
			else if((RC_CtrlData.key.v&KEY_D)&&(accelerate_flag == 1)&&(auto_d_flag == 1))
				{
          auto_angle_data.yaw_angle_offset+=0.5f;
					auto_d_flag=0;
        }
				
				if(auto_w_flag == 0)
					auto_w_time++;
				else
					auto_w_time = 0;
				if(auto_w_time == 20)
				{
					auto_w_flag = 1;
					auto_w_time = 0;
				}
				
				if(auto_s_flag == 0)
					auto_s_time++;
				else
					auto_s_time = 0;
				if(auto_s_time == 20)
				{
					auto_s_flag = 1;
					auto_s_time = 0;
				}
				
				if(auto_a_flag == 0)
					auto_a_time++;
				else
					auto_a_time = 0;
				if(auto_a_time == 20)
				{
					auto_a_flag = 1;
					auto_a_time = 0;
				}
				
				if(auto_d_flag == 0)
					auto_d_time++;
				else
					auto_d_time = 0;
				if(auto_d_time == 20)
				{
					auto_d_flag = 1;
					auto_d_time = 0;
				}
		}
		break;
		default:
			break;
	}
	GimbalAngleLimit();
}

u32 shot_speed_num=1;

//����ƽ����
float avg(float new_value)
{
	float avg_speed;
	float sum = 0;
	static float value[FILTER_NUM] = {0};
	for(int i = 0; i < FILTER_NUM - 1 ; i ++)
	{
		value[i] = value[i + 1];
		sum += value[i];
	}
	if(value[0] == 0)
		sum += 14.7f;
	value[FILTER_NUM - 1] = new_value;
	sum += value[FILTER_NUM - 1];
	avg_speed = sum / shot_speed_num;
	return avg_speed;
}
