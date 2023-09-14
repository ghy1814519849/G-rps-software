#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "main.h"

#define MAX_WHEEL_RPM  8000//7500
#define MAX_CHASSIS_VR_SPEED 430//300//430

#define SOFTWARE_LIMIT 0
#define CAP_LIMIT 1

#define CHASSIS_EXTRA_POWER_BUFFER 60//30

#define ROTATE_SPEED_REF_55       60//55
#define ROTATE_SPEED_REF_60       75
#define ROTATE_SPEED_REF_65				90

#define ROTATE_SPEED_REF_70       70//85//55
#define ROTATE_SPEED_REF_90       90//80
#define ROTATE_SPEED_REF_120			120//60

#if CAP_CTRL == 1//��
#define  WARNING_VOLTAGE      13
#define ADJUST_LEVEL   1
#elif CAP_CTRL == 0//��
#define  WARNING_VOLTAGE       18
#endif

#define  I_TIMES_V_TO_WATT    0.0000231f    //I -16384~+16384 V .filter_rate
//������ȼ��� p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; 
//i��ֱ�ӷ������������Ҳ��pid���������out,��������ֱ����pid.out����Ҫ�Լ����������ֲỻ������������C620������Ƶ�����Χ16384~16384 
//ͨ��ʹ��ʾ������ֵ��matlab���
//(���ֲ���������Ϲ�����������)
#define FACTOR_2	0.000000161f//0.00000000006561f//
#define FACTOR_1	-0.0000229f//-0.0000006107f//
#define FACTOR_0  0.458f//0.006234f//

#define RATATE_DCREASE             0.8

//���� �Ƕ�(��)ת���� ���ȵı���
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//���� ���� ת���� �Ƕȵı���
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

typedef enum//////////////////////////////////////////��ɾȥ��������
{
		CHASSIS_RELAX          = 0,
		CHASSIS_STOP           = 1,
		MANUAL_SEPARATE_GIMBAL = 2,
		MANUAL_FOLLOW_GIMBAL   = 3,
//		DODGE_MODE             = 4,
//		AUTO_SEPARATE_GIMBAL   = 5,
//		AUTO_FOLLOW_GIMBAL     = 6,
		CHASSIS_ROTATE         = 7,
		CHASSIS_REVERSE        = 8,
//		CHASSIS_CHANGE_REVERSE = 9,
		CHASSIS_SEPARATE 		   = 10,
//		CHASSIS_AUTO_SUP       = 11,
} chassis_mode_e;

typedef struct
{
		double           vx; // forward/back
		double           vy; // left/right
		double           vw; // 
		
		chassis_mode_e  ctrl_mode;
		chassis_mode_e  last_ctrl_mode;

		float           gyro_angle;
		float           gyro_palstance;

		int16_t         wheel_speed_fdb[4];
		int16_t         wheel_speed_ref[4];
		int16_t         current[4];
		
		int32_t         position_ref;
		uint8_t         follow_gimbal;
} chassis_t;

typedef enum
{
  NORMAL_SPEED_MODE          = 0,
	HIGH_SPEED_MODE            = 1,
	LOW_SPEED_MODE             = 2,
} chassis_speed_mode_e;

typedef struct
{
  double foward_back_to_foward_back_rotate_speed;
  double foward_back_to_left_right_rotate_speed;
  double left_right_to_left_right_rotate_speed;
  double left_right_to_foward_back_rotate_speed; 
  double sin_chassis_angle;
  double cos_chassis_angle;
}chassis_rotate_speed_t;

extern int turn_speed;
extern chassis_t chassis;
extern u16  Max_Power;
extern u8  Max_Current;
extern u8 Power_Work_Mode;
extern u32 reverse_flag;
extern uint16_t forward_back_speed;
extern uint16_t left_right_speed;
extern u8 chassis_rotate_flag;
extern int chassis_rotate_speed_vw_ref;
extern chassis_speed_mode_e chassis_speed_mode;
extern float power_limit_rate;
extern float stop_diff;

void chassis_task(void);//��������
void follow_gimbal_handle(void);//���̸�����̨
void reverse_follow_gimbal_handle(void);//������̨����
void rotate_handle(void);//����С����
void chassis_separate_handle(void);//������̨����
void mecanum_calc(float vx, float vy, float vw, int16_t *speed);//���ּ���
void chassis_stop_handle(void);//����ֹͣ
void chassis_param_init(void);//���̲�����ʼ��
void power_limit_handle(void);//��������///////////////////////////////////////////////////////////
int Check_Vcap_recieve(void);//��鳬�������Ƿ���������
void Fault_judge(void);//����ָ���������
static float power_limit_filter(float power_limit_rate);
static float heat_power_calc(int i);//���ȹ��ʼ���
static float i_predict(int motor_number,float factor);//pid�㷨�������������
void power_send_handle(void);//�������ݸ���������
int Check_Vcap_recieve(void);//������0.1sû���յ���Ϣʱ�л�����0,���򷵻�1

void buffer_power(void);
void power_send_handle1(void);
void power_send_handle2(void);
#endif

