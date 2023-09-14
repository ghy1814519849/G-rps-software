#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_
#include "main.h"
//������:
//3200~10m/s	3500~12m/s	
//#define ONE_POKE_ANGLE 60.0f *19.203f		//һ������ĽǶ�
#define ONE_POKE_ANGLE_2 90.0f 		//һ������ĽǶȣ��������̣�*36.109f//24.0

#define LOCK_ANGLE 30.0f 	
#define POKE_SPEED      -200      //�����ٶ�

#define FRICTION_SPEED_10 2000			//Ħ����ת�٣��͵����й�	//2000����
#define FRICTION_SPEED_12 2100		  //ǰ�޺���
#define FRICTION_SPEED_14 2575			//�����ٶ�
#define FRICTION_SPEED_16 3200//3000			//2900	//13.0//2650��//3200
#define friction_stop 0
#define friction_on 1
#define friction_slow 2
#define friction_back 4

typedef enum
{
  NOMAL=0,
  LOCK=1,
} shoot_state_e;  //0���� 1��ת

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,
} shoot_mode_e;


typedef __packed struct
{
  /* shoot task relevant param */
  shoot_mode_e ctrl_mode;
  uint8_t      shoot_cmd;
  uint32_t     c_shoot_time;   //continuous
  uint8_t      c_shoot_cmd;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
  uint16_t     ref_shot_bullets;
  uint16_t     shot_bullets;
  uint16_t     remain_bullets;
  float        total_speed;
  float        limit_heart0;
  float        residue_heart1;
  uint16_t     max_heart0;
  uint16_t     max_heart1;
  uint16_t     handle_timescouter;
  uint16_t     cooling_ratio;
  uint16_t     ShootMotorSpeed;
  uint16_t     NoLimitHeat;
  uint8_t			 Speed_Gear;
  float        shoot_heat1;
  u8           max_buttlets;
} shoot_t;

typedef __packed struct
{
  float     ecd_angle_ref ;   //�ջ�Ŀ��Ƕ�
  float     now_angle ;       //�ջ���ǰ�Ƕ�
  u32       heat1_shoot;      //�������Ʊ�־λ
  float     senror_Angle;     //����������������Ƕ�
  int16_t   start_shoot;      //���̿�ʼ��ת

  u32       lock_angle;       //������¼�Ƕ�
  uint8_t   lock_rotor1;      //��ת��־λ
  u32       last_poke_state;  //����״̬��¼
  u32       free_time;        //������Ƶ
  u32       lock_turn;        //��ת��ת��־λ
  u32       lock_turn_count;  //��ת��ת����
  u32       shoot_Flag;       //���״̬ѭ��

  int16_t   sensor_count;     //�Ƕȴ���������
  float     last_sensor_angle;//���̳�ʼ������
  float     start_Angle;      //���̳�ʼ�Ƕ�
  u8        state;            //0:δ��ʼ�� 1:��ʼ�����
  u32       init_cnt;
  u8        can_received_flag;
  u8        init_bullets;
  u8        shoot_step;
  u32       step_cnt;
  u8        free_cnt;
} poke_shoot_t;

extern int flag;
extern float taget_angle_2;
extern int shoot_flag;
extern char ignore_heat;
extern int over_hot_motor;
extern u8 friction_normal_flag;
extern int friction_speed_16;
extern int friction_speed_14;
extern uint16_t frictionSpeed;
extern int frictionSpeed1;
extern u8 shot_judge_flag;             //���룬���ܷ�������жϱ�־λ���������1������һ���ж�
extern u8 position_set_flag;
extern int turn_friction_flag;
extern int friction_rotor_error;

extern u8 lock_flag; 
extern int lock_time;
extern int lock_cilent_flag;//�ͻ���UI��ʾ��־λ

void shot_task(void);
void test_mode_task(void);
void heat1_limit(void);
void shoot_bullet_handle(void);
void shoot_friction_handle(void);
void shot_param_init(void);
void friction_lock(void);
#endif
