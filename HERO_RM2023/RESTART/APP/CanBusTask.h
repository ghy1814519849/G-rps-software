#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include "main.h"

/* CAN Bus 1 */  
#define CAN_BUS1_PITCH_FEEDBACK_MSG_ID            0x141//0x204//   //PITCH ID 1//MF5015

#define CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID 0x201		//摩擦轮1
#define CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID 0x202 	//摩擦轮2
#define CAN_BUS1_MOTOR5_FEEDBACK_MSG_ID 0x205   //上拨盘
#define CAN_BUS1_MOTOR8_FEEDBACK_MSG_ID 0x208		//倍镜电机3508
#define CAN_BUS1_MOTOR9_FEEDBACK_MSG_ID 0x209		//图传云台

/* CAN Bus 2 */  
#define CAN_BUS2_YAW_FEEDBACK_MSG_ID              0x142   //YAW ID 2//MF9025

#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID 0x201  //底盘四个电机
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID 0x202
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID 0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID 0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID 0x205  //下拨盘 ID 5//pid_shoot_bullet_position_speed_loop正常情况下拨盘转动的pid
#define CAN_BUS2_POWER_FEEDBACK_MSG_ID  0x407  //电容

#define VOLTAGE_BUF_SIZE  50

typedef struct
{
  uint8_t  charge_power;                       //充电功率/W
  char     charge_current;                     //充电电流/A
  float    cap_voltage_filte;                  //电容电压/V
  uint8_t  boost_voltage;                      //boost电压/V
  uint8_t  battery_voltage;                    //电池电压/V
  uint8_t  output_voltage;                     //输出电压/V
  uint8_t  cap_voltage_buff[VOLTAGE_BUF_SIZE];
  uint8_t  raw_cap_voltage;
  uint8_t buf_count;
} capacitance_message1_t;

typedef struct
{
  uint8_t  max_charge_power;                   //最大充电功率/W      0-100
  char     max_charge_current;                 //最大充电电流/A       0-10
  uint8_t  boost_voltage_ref;                  //boost电压给定值/V     20-30
  uint8_t  output_mode_set;                    //设置的输出模式  0：电池输出;1：电容升压输出
  uint8_t  output_mode_get;                    //当前输出模式    0：电池输出;1：电容升压输出

  union
  {
    u8 fault;
    uint8_t  charge_over_current_flag:1;           //充电过流
    uint8_t  cap_over_voltage_flag:1;              //电容过压
    uint8_t  battery_over_under_voltage_flag:1;    //电池欠压或过压
    uint8_t  battery_off_flag:1;                   //电池掉电,检测到电池断电
    uint8_t  two_leg_over_current_flag:1;          //两桥臂下管过流,该故障一般为mos损坏导致桥臂直通
    uint8_t  output_change_switch_flag:1;          //输出电源切换开关直通
    uint8_t  boost_over_voltage_flag:1;            //boost升压过压
    uint8_t  boost_over_current_flag:1;            //boost升压过流
  }
  fault_union;	//0：无故障  1：故障

  uint8_t  system_mode;                        //系统运行模式  0：系统初始化 1：等待模式 2：正常运行 3：系统故障 4：系统关机 5：系统复位
  int8_t   output_power;                       //输出功率
} capacitance_message2_t;

typedef struct
{
	uint16_t mode;
	uint16_t mode_sure;
	
	uint16_t in_power;
	uint16_t in_v;
	uint16_t in_i;
	
	uint16_t out_power;
	uint16_t out_v;
	uint16_t out_i;

	uint16_t tempureture;
	uint16_t time;
	uint16_t this_time;	
}capacitance_message3_t;

extern volatile Encoder BUS2_CM1Encoder;
extern volatile Encoder BUS2_CM2Encoder;
extern volatile Encoder BUS2_CM3Encoder;
extern volatile Encoder BUS2_CM4Encoder;
extern volatile Encoder BUS2_CM5_Poke_Encoder;//下拨盘
extern volatile Encoder GMYawEncoder;
extern volatile Encoder BUS1_CM1Encoder;
extern volatile Encoder BUS1_CM2Encoder;
extern volatile M2006_info_t BUS1_CM5_Poke_Encoder;//上拨盘
extern volatile Encoder BUS1_CM8Encoder;//倍镜电机
extern volatile Encoder BUS1_CM9Encoder;//图传云台
extern volatile Encoder GMPitchEncoder;
extern volatile capacitance_message1_t capacitance_message1;
extern volatile capacitance_message2_t capacitance_message2;
extern volatile capacitance_message3_t capacitance_message3;
extern int  not_receive_time;

extern int BUS1_CM2Encoder_readtime;

void POWER_Control1(uint16_t Power,uint16_t StdId);
void POWER_Control2(uint16_t StdId);

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void YawEncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void PitchEncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void POWER_Control(u8* msg);
void Set_GM_CM_Current(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq);
void Set_GM_Current(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq);
void Set_Pitch_Current(CAN_TypeDef *CANx, int16_t control);
void Set_Yaw_Current(CAN_TypeDef *CANx, int16_t control);
void Encoder_M2006_ammo(M2006_info_t *M2006_info, CanRxMsg *msg);
void CAN_9015angleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl);
void CAN_9015Command(CAN_TypeDef *CANx ,uint8_t command);
void Set_EncodeOffset(CAN_TypeDef *CANx);
void CAN_9015setpidCommand(CAN_TypeDef *CANx ,float akp,\
                                                float aki,\
                                              float skp,\
                                              float ski,\
                                              float iqkp,\
                                              float iqki);
#endif

