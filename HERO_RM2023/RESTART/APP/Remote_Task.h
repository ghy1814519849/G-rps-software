#ifndef _REOMTE_TASK_H_
#define _REOMTE_TASK_H_
#include "main.h"
#include "ramp_second.h"
/*
****************************************************************************
*
*																	MARCO
****************************************************************************
*/
//remote control parameters
#define INFANTRY 1				//编号

#define FRICTION_ON          1
#define FRICTION_OFF         0

#define NORMAL_FORWARD_BACK_SPEED_80 	  160						//键鼠模式底盘速度设定
#define NORMAL_LEFT_RIGHT_SPEED_80   		150
#define NORMAL_FORWARD_BACK_SPEED_90 	  200
#define NORMAL_LEFT_RIGHT_SPEED_90   		210
#define NORMAL_FORWARD_BACK_SPEED_120	250
#define NORMAL_LEFT_RIGHT_SPEED_120		240

#define HIGH_FORWARD_BACK_SPEED_80 			1200
#define HIGH_LEFT_RIGHT_SPEED_80   			350
#define HIGH_FORWARD_BACK_SPEED_90 			2000
#define HIGH_LEFT_RIGHT_SPEED_90   			450
#define HIGH_FORWARD_BACK_SPEED_120		2000
#define HIGH_LEFT_RIGHT_SPEED_120			650

//#define LOW_FORWARD_BACK_SPEED 			  100
//#define LOW_LEFT_RIGHT_SPEED   			  150

#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   
#define STICK_TO_CHASSIS_SPEED_REF_FACT     0.3f//0.6f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.1f								//遥控控制灵敏度系数
#define STICK_TO_YAW_ANGLE_INC_FACT         0.003f//0.004f//0.005f

#define FRICTION_RAMP_TICK_COUNT			100
#define FRICTION_RAMP_OFF_TICK_COUNT	500

#define MOUSE_LR_RAMP_TICK_COUNT1			75
#define MOUSR_FB_RAMP_TICK_COUNT1		  70//起步斜坡斜率调整
#define MOUSE_LR_RAMP_TICK_COUNT2			40
#define MOUSR_FB_RAMP_TICK_COUNT2		  35//正常斜坡斜率调整


#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		4.0f//鼠标操作灵敏度系数
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.042f

#define REMOTE_SWITCH_VALUE_UP         	0x01u
#define REMOTE_SWITCH_VALUE_DOWN			  0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u
#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)   
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)  
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)
#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))   

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP)) 

#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u

#define  KEY_W  		0X0001  //前
#define  KEY_S  		0X0002	//后
#define  KEY_A  		0X0004	//左
#define  KEY_D  		0X0008	//右
#define  KEY_SHIFT  0X0010	//加速
#define  KEY_CTRL  	0X0020	//大陀螺
#define  KEY_Q  		0X0040	//吊射
#define  KEY_E  		0X0080	//
#define  KEY_R  		0X0100	//爆发
#define  KEY_F  		0X0200	//底盘云台分离
#define  KEY_G  		0X0400	//移动反向
#define  KEY_Z  		0X0800	//摩擦轮反转
#define  KEY_X  		0X1000	//软件复位
#define  KEY_C  		0X2000	//摩擦轮正转
#define  KEY_V  		0X4000	//底盘云台反向
#define  KEY_B  		0X8000	//客户端图形复位

typedef enum
{
    PREPARE_STATE,     	//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,				//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
	uint16_t last_v;
}Key;

typedef enum
{
  KEY_R_UP=0,
  KEY_R_DOWN=1,
 
} key_state_t;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

typedef enum
{
  NOSHOOTING = 0,
  SHOOTING = 1,
} Shoot_State_e;

//输入模式:遥控器/键盘鼠标/停止运行
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;

typedef enum
{
  FRICTION_WHEEL_OFF = 0,
  FRICTION_WHEEL_START_TURNNING = 1,
  FRICTION_WHEEL_ON = 2,
  FRICTION_WHEEL_STOP_TURNNING = 3,
	FRICTION_WHEEL_BACK = 4,
} FrictionWheelState_e;
//拨杆动作枚举

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
		float roll_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
		float roll_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
		float roll_speed_ref;
}Gimbal_Ref_t;

typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;

typedef struct
{
  u8 Key_W_Flag ;
	u8 Key_A_Flag ;
	u8 Key_S_Flag ;
	u8 Key_D_Flag ;
	u8 Last_Key_W_Flag ;
	u8 Last_Key_A_Flag ;
	u8 Last_Key_S_Flag ;
	u8 Last_Key_D_Flag ;
	u8 Key_A_D_Flag;
	u8 Key_E_R_Flag;//
	u8 Key_W_S_Flag;
}Key_Flag_t;

extern RC_Ctl_t RC_CtrlData; 
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;
extern u8 barriers_flag;
extern RampGen_t frictionRamp;
extern u8 friction_rotor;
extern Shoot_State_e shootState;
extern Key_Flag_t Key_Flag;
extern RampGen_t FBSpeedRamp;
extern u8 chassis_separate_flag;
extern u8 last_chassis_separate_flag;
extern u8 rotate_return_flag;
extern int rotate_num;
extern int auto_return_flag;
extern u8 reverse_state;
extern u8 switch_mode_flag;
extern u8 dir_mov_flag;
extern int rotate_num_ture;
extern int yaw_init;
extern u32 position_flag;
extern u8 rear_flag;
extern int down_poke_off_flag;


WorkState_e GetWorkState(void);
void SetWorkState(WorkState_e state);

void SetInputMode(Remote *rc);
InputMode_e GetInputMode(void);
void RemoteDataPrcess(uint8_t *pData);
void Remote_Remove_Barrier(RemoteSwitch_t *sw, uint8_t val);
void Remote_Auto_Shoot_Control(RemoteSwitch_t *sw, uint8_t val);
void RemoteControlProcess(Remote *rc);
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);
void SetShootState(Shoot_State_e v);
FrictionWheelState_e GetFrictionState(void);
void SetFrictionState(FrictionWheelState_e v);
void remote_self_rotate(RemoteSwitch_t *sw, uint8_t val);
void GimbalAngleLimit(void);
void Mouse_Key_Rotate_Reverse_Control(void);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void MouseShootControl(Mouse *mouse);
void RemoteTaskInit(void);
void SoftReset(void);
#endif

