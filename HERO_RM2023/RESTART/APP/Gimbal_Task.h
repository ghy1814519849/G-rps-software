#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__
#include "main.h"
/* gimbal control period time (ms) */
#define Default_bullet_Speed  14.5
#define GIMBAL_PERIOD 5
#define FILTER_NUM 5

typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
  GIMBAL_AUTO_AIM	   = 8,
  GIMBAL_AUTO_SUP      = 9,
  GIMBAL_REVERSE       = 10,
  GIMBAL_AUTO_SMALL_BUFF   = 11,
  GIMBAL_AUTO_BIG_BUFF     = 12,
  GIMBAL_AUTO_ANGLE    = 13,
  GIMBAL_FOLLOW_CHASSIS=14,
} gimbal_mode_e;

typedef struct
{

  //进入吊射模式前的初始数据
  float pit_angle_raw;
  float yaw_angle_raw;
	float roll_angle_raw;
  float  distance;
  float bullet_speed;

  float pitch_taget_angle;   //pich轴解算过后的目标角度
	float roll_taget_angle;   //roll轴解算过后的目标角度
	
  float pit_angle_offset;   //偏移量！！
  float yaw_angle_offset;
	float roll_angle_offset;
	float pit_angle_offset_1;

} auto_angle_data_t;  //吊射数据

typedef enum
{
  NO_ACTION = 0,
  IS_ACTION,
} action_mode_e;

typedef enum
{ 
  CMD_NO =0,
  CMD_CALI_FIVE,
  CMD_CALI_NINE,
  CMD_TARGET_NUM
} gimbal_cmd_e;

typedef struct
{
  /* position loop */
  float yaw_angle_ref;
  float pit_angle_ref;
	float roll_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  float roll_angle_fdb;	
  /* speed loop */
  float yaw_speed_ref;
  float pit_speed_ref;
	float roll_speed_ref;
  float yaw_speed_fdb;
  float pit_speed_fdb;
	float roll_speed_fdb;
} gim_pid_t;

typedef struct
{
  /* unit: degree */
  float pit_relative_angle;
  float yaw_relative_angle;
  float gyro_angle;
  /* uint: degree/s */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;

typedef struct
{
  action_mode_e ac_mode;
  float         action_angle;
  uint8_t       no_action_flag;
  uint32_t      no_action_time;
} no_action_t;

typedef struct
{
  /* ctrl mode */
  gimbal_mode_e ctrl_mode;
  gimbal_mode_e last_ctrl_mode;
  
  /* gimbal information */
  gim_sensor_t  sensor;
  float         ecd_offset_angle;
  float         yaw_offset_angle;
  
  /* gimbal ctrl parameter */
  gim_pid_t     pid;
  no_action_t   input;
  
  /* read from flash */
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  
  gimbal_cmd_e  auto_ctrl_cmd;
} gimbal_t;

typedef enum
{
    NORMAL_SHOOT    = 0,
    AUTO_SHOOT      = 1,            //自瞄
		AUTO_ANGLE_SHOOT = 2,           //吊射
    AUTO_STATION_SHOOT = 3,         //吊射
		LOCK_POKE = 4,
} auto_shoot_mode_e;

typedef struct
{
    float Yaw_Angle_Pre;
    float Yaw_Angle_Now;
    float Pit_Angle_Pre;
    float Pit_Angle_Now;
    uint32_t Time_Sample;
	  float Angular_Yaw_Speed;
	  float Angular_Yaw_Speed_Pre;
    float Angular_Pit_Speed;
	  float Angular_Pit_Speed_Pre;
	  float Yaw_Acceleration;
	  float Pit_Acceleration;
	  uint32_t time1;
	  uint32_t time2;
	  float time_error;
	  float yaw_angle_error;
	  float pit_angle_error;
	  float time_delay;
} Speed_Prediction_t;

typedef struct
{
  Speed_Prediction_t Speed_Prediction;
  Speed_Prediction_t Speed_Prediction_Gimbal;
  float Filtered_Angular_Yaw_Speed;
  float Filtered_Angular_Pit_Speed;
  u8	  Recognized_Flag;
  u16   Recognized_Timer;
  s16   Continue_Recognized_Cnt;
  int Err_Pixels_Yaw;
  int Err_Pixels_Pit;
  float Distance;
  float Ballistic_Compensation;
  float Yaw_Image_Gimbal_Delay_Compensation;
  float Pit_Image_Gimbal_Delay_Compensation;
  u8 Image_Gimbal_Delay_Compensation_Flag;
  float Delta_Dect_Angle_Pit;
  float Delta_Dect_Angle_Yaw;
  float Last_Delta_Dect_Angle_Yaw;

  u16   Continue_Large_Err_Cnt;
  float Ball_Delay_Compensation;
  float Delta_Dect_Angle_Yaw_Temp;
  int   Auto_Shoot_Permit;
	float Armor_yaw;
	float Armor_pit;
} Gimbal_Auto_Shoot_t;


extern gimbal_t gim;
extern auto_angle_data_t auto_angle_data;
extern auto_shoot_mode_e  autoshoot_mode;
extern float x_diff;
extern int delay_flag;
extern int delay_time;
extern int compensate_time1;
extern int last_dir_x;
extern int last_dir_yaw;
extern float yaw_compensate;
extern int csajkcns;
extern float pitch_middle;

void gimbal_task(void);
void cascade_pid_ctrl(float yaw_ref,float pitch_ref,float yaw_fdb,float pitch_fdb);
void init_mode_handle(void);
void close_loop_handle(void);
void auto_angle_handle(void);
void gimbal_stop(void);
float AvgFilter(float new_value);
void gimbal_param_init(void);
void gimbal_back_param(void);
float lab_shoot_task ( float shoot_speed , float distance , float angle );
void auto_angle_task(void);

#endif

