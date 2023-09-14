#include "main.h"
#include "stdio.h"

static uint32_t can2_count = 0,can1_count = 0;
volatile Encoder BUS2_CM1Encoder = {0,0,0,0,0,0,0,0,0};           //4个轮子电机
volatile Encoder BUS2_CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder BUS2_CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder BUS2_CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder BUS2_CM5_Poke_Encoder = {0,0,0,0,0,0,0,0,0};     //下拨盘
volatile Encoder BUS1_CM1Encoder = {0,0,0,0,0,0,0,0,0};           //两个摩擦轮
volatile Encoder BUS1_CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder BUS1_CM8Encoder = {0,0,0,0,0,0,0,0,0};						//倍镜电机
volatile Encoder BUS1_CM9Encoder = {0,0,0,0,0,0,0,0,0};						//图传云台
volatile Encoder GMYawEncoder 	 = {0,0,0,0,0,0,0,0,0};						//CAN2
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};						//CAN1
volatile capacitance_message1_t capacitance_message1 = {0};       //超级电容部分_CAN2
volatile capacitance_message2_t capacitance_message2 = {0};
volatile capacitance_message3_t capacitance_message3 = {0};
volatile M2006_info_t BUS1_CM5_Poke_Encoder = {0,0,0,0,0,0,0,0};     //上拨盘

u8 can_buf[8]={0};
int  not_receive_time=0;

int BUS1_CM2Encoder_readtime=0;

int GetTFDistance ( CanRxMsg * msg )	
{
		int  dis = ( msg->Data[1] << 8 ) | msg->Data[0];
		return dis;
}

void CapacitanceProcess1(volatile capacitance_message1_t *v,CanRxMsg * msg)
{
  float temp_sum=0;
  v->charge_power    = msg->Data[1];
  v->charge_current  = msg->Data[2];
  v->raw_cap_voltage = msg->Data[3];
  v->boost_voltage   = msg->Data[4];
  v->battery_voltage = msg->Data[5];
  v->output_voltage	= msg->Data[6];
  v->cap_voltage_buff[v->buf_count++] = v->raw_cap_voltage;  //单位10v
  if(v->buf_count == VOLTAGE_BUF_SIZE)
    {
      v->buf_count = 0;
    }
  //计算电容平均值
  for(int i = 0; i < VOLTAGE_BUF_SIZE; i++)
    {
      temp_sum += v->cap_voltage_buff[i];
    }
  v->cap_voltage_filte =(temp_sum/VOLTAGE_BUF_SIZE)/10.0f;
  not_receive_time=0;
}

void CapacitanceProcess2(volatile capacitance_message2_t *v,CanRxMsg * msg)
{
  v->max_charge_power   = msg->Data[1];
  v->max_charge_current = msg->Data[2];
  v->boost_voltage_ref  = msg->Data[3];
  v->output_mode_set    = msg->Data[4];
  v->output_mode_get    = msg->Data[5];
  v->fault_union.fault = msg->Data[6];
//	v->charge_over_current_flag  = (msg->Data[6]&0x01);
//	v->cap_over_voltage_flag     = (msg->Data[6]&0x02)>>1;
//	v->battery_over_under_voltage_flag = (msg->Data[6]&0x04)>>2;
//	v->battery_off_flag   = (msg->Data[6]&0x08)>>3;
//	v->two_leg_over_current_flag = (msg->Data[6]&0x10)>>4;
//	v->output_change_switch_flag = (msg->Data[6]&0x20)>>5;
//	v->boost_over_voltage_flag   = (msg->Data[6]&0x40)>>6;
//	v->boost_over_current_flag   = (msg->Data[6]&0x80)>>7;
  v->system_mode = msg->Data[7];
}

void Encoder_M2006_ammo(M2006_info_t *M2006_info, CanRxMsg *msg)
{
    M2006_info->rotor_angle    = ((msg->Data[0] << 8) | msg->Data[1]);
    M2006_info->rotor_speed    = ((msg->Data[2] << 8) | msg->Data[3]);
	  M2006_info->rotor_speed_fdata += (1.0 / (1.0 + 1.0/(2.0f * 3.14f *0.001*0.1)))*(M2006_info->rotor_speed - M2006_info->rotor_speed_fdata);

    M2006_info->torque_current = ((msg->Data[4] << 8) | msg->Data[5]);

	  if(M2006_info->rotor_speed_fdata>0 && (M2006_info->rotor_angle_last-M2006_info->rotor_angle)>4096)
	   {
     M2006_info->DIAL_N++;
	   }
	  else if(M2006_info->rotor_speed_fdata<0 && (M2006_info->rotor_angle_last-M2006_info->rotor_angle)<-4096)
	   {
		 M2006_info->DIAL_N--;
	   }
	  M2006_info->rotor_out=((M2006_info->rotor_angle-M2006_info->rotor_basic)/22.0f+M2006_info->DIAL_N*360.0f)/36.109f;
		 
    M2006_info->rotor_out_fdata += (1.0 / (1.0 + 1.0/(2.0f * 3.14f *0.001*1)))*(M2006_info->rotor_out - M2006_info->rotor_out_fdata);
		 
	  M2006_info->rotor_angle_last=M2006_info->rotor_angle;	
}


void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)//由3508顺时针旋转，机械角度增大，可自定义旋转圈数减少
{

		v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
		v->ecd_value = v->ecd_bias;
		v->last_raw_value = v->ecd_bias;
		v->temp_count++;
}

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
		int i=0;

		int32_t temp_sum = 0;    
		v->last_raw_value = v->raw_value;
		v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
		v->diff = v->raw_value - v->last_raw_value;
		if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
				v->round_cnt++;
				v->ecd_raw_rate = v->diff + 8192;
		}
		else if(v->diff>4096)
		{
				v->round_cnt--;
				v->ecd_raw_rate = v->diff- 8192;
		}		
		else
		{
				v->ecd_raw_rate = v->diff;
		}
		//计算得到连续的编码器输出值
		v->ecd_value = v->raw_value + v->round_cnt * 8192;
		//计算得到角度值，范围正负无穷大
		v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f + v->round_cnt * 360;

		v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
		if(v->buf_count == RATE_BUF_SIZE)
		{
				v->buf_count = 0;
		}
		//计算速度平均值
		for(i = 0;i < RATE_BUF_SIZE; i++)
		{
				temp_sum += v->rate_buf[i];
		}
		v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		//均值
		v->rate_rpm = (msg->Data[2]<<8)|msg->Data[3];			//瞬时值
		if(msg->Data[4]&0x80)		//为什么要用位运算？？？
			v->current = ((msg->Data[4]<<8)|msg->Data[5]-0x8000);
		else
			v->current = (msg->Data[4]<<8)|msg->Data[5];
		v->temperature = msg->Data[6];	//温度  //注：C610电调反馈没有DATA[6]温度
}

//void PitchEncoderProcess(volatile Encoder *v, CanRxMsg * msg)
//{
//	int i=0;
//	int32_t temp_sum = 0;
//	v->last_raw_value = v->raw_value;
//	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
//	v->diff = v->raw_value - v->last_raw_value;
//	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
//	{
//		v->round_cnt++;
//		v->ecd_raw_rate = v->diff + 8192;
//	}
//	else if(v->diff>4096)
//	{
//		v->round_cnt--;
//		v->ecd_raw_rate = v->diff- 8192;
//	}		
//	else
//	{
//		v->ecd_raw_rate = v->diff;
//	}
//	v->ecd_value = v->raw_value + v->round_cnt * 8192;
//	//计算得到角度值，范围正负无穷大
//	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f  + v->round_cnt * 360;
//	
//	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
//	if(v->buf_count == RATE_BUF_SIZE)
//	{
//		v->buf_count = 0;
//	}
//	//计算速度平均值
//	for(i = 0;i < RATE_BUF_SIZE; i++)
//	{
//		temp_sum += v->rate_buf[i];
//	}
//	v->rate_rpm = ((temp_sum * 1.0)/RATE_BUF_SIZE) * 57.3;//		为毛要×57.3？？？
//	//从电机编码器读取的速度
//	v->filter_rate = (msg->Data[2]<<8)|msg->Data[3];
//	if(msg->Data[4]&0x80)
//		v->current = ((msg->Data[4]<<8)|msg->Data[5]-0x8000);
//	else
//		v->current = (msg->Data[4]<<8)|msg->Data[5];
//	v->temperature = msg->Data[6];
//}

void YawEncoderProcess(volatile Encoder *v, CanRxMsg * msg)//云台yaw，pitch共用
{
	int i=0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[7]<<8)|msg->Data[6];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 65536;
	}
	else if(v->diff>32768)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 65536;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 65536;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0054931641f  + v->round_cnt * 360;
	//用一圈的角度除以编码器最大值=======================↑↑↑↑↑//
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->rate_rpm = ((temp_sum * 1.0)/RATE_BUF_SIZE) * 57.3;//		这个57.3到底有什么秘密啊啊啊啊啊啊！
	//从电机编码器读取的速度
	v->filter_rate = (msg->Data[5]<<8)|msg->Data[4];
	if(msg->Data[4]&0x80)
		v->current = ((msg->Data[3]<<8)|msg->Data[2]-0x8000);
	else
		v->current = (msg->Data[3]<<8)|msg->Data[2];
	v->temperature = msg->Data[1];
}



void CAN_9015angleControl(CAN_TypeDef *CANx ,int16_t maxSpeed ,uint32_t angleControl)
{
    CanTxMsg txmsg;
    txmsg.StdId = 0x142;
    txmsg.DLC = 0x08;
    txmsg.IDE = CAN_Id_Standard;
    txmsg.RTR = CAN_RTR_Data;
    txmsg.Data[0] = 0xA4;
    txmsg.Data[1] = 0x00;
    txmsg.Data[2] = (uint8_t)maxSpeed;
    txmsg.Data[3] = (uint8_t)(maxSpeed >> 8);
    txmsg.Data[4] = (uint8_t)angleControl;
    txmsg.Data[5] = (uint8_t)(angleControl >> 8);
    txmsg.Data[6] = (uint8_t)(angleControl >> 16);
    txmsg.Data[7] = (uint8_t)(angleControl >> 24);
    
    CAN_Transmit(CANx,&txmsg);
}



void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
	  can1_count++;
		switch(msg->StdId) 
    {

			case CAN_BUS1_PITCH_FEEDBACK_MSG_ID:				//MF5015
			{
				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
				YawEncoderProcess(&GMPitchEncoder,msg);
				//码盘中间值设定也需要修改
				if(can1_count<=100)
					{
						if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-32700)
							{
								GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset + 65536;
							}
						else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 32700)
							{
								GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset - 65536;
							}
					}
			}
      break;
			case CAN_BUS1_MOTOR9_FEEDBACK_MSG_ID:				//GM6020图传云台
			{
				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
				EncoderProcess(&BUS1_CM9Encoder,msg);				//码盘中间值设定也需要修改
				if(can1_count<=100)
					{
						if((BUS1_CM9Encoder.ecd_bias - BUS1_CM9Encoder.ecd_value) <-4096)
							{
								BUS1_CM9Encoder.ecd_bias = CameraEncoder_Offset + 8192;
							}
						else if((BUS1_CM9Encoder.ecd_bias - BUS1_CM9Encoder.ecd_value) > 4096)
							{
								BUS1_CM9Encoder.ecd_bias = CameraEncoder_Offset - 8192;
							}
					}
			}
			break;
			case CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID://摩擦轮
			{
				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
				(can1_count<=50) ? GetEncoderBias(&BUS1_CM1Encoder,msg):EncoderProcess(&BUS1_CM1Encoder,msg);        //获取到编码器的初始偏差值
			}
			break;
			case CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID://摩擦轮
			{
				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
				(can1_count<=50) ? GetEncoderBias(&BUS1_CM2Encoder,msg):EncoderProcess(&BUS1_CM2Encoder,msg);
				BUS1_CM2Encoder_readtime++;
			}
			break;
			case CAN_BUS1_MOTOR5_FEEDBACK_MSG_ID://上拨盘
			{
				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));//(can1_count<=50) ? GetEncoderBias(&BUS1_CM5_Poke_Encoder,msg):
				Encoder_M2006_ammo(&BUS1_CM5_Poke_Encoder,msg);
			}
			break;
			case CAN_BUS1_MOTOR8_FEEDBACK_MSG_ID://倍镜
			{
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
					(can2_count<=50) ? GetEncoderBias(&BUS1_CM8Encoder,msg):EncoderProcess(&BUS1_CM8Encoder,msg);
			}
			break;

			default:
			{

			}
		}
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    can2_count++;
		switch(msg->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{
						LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
						(can2_count<=50) ? GetEncoderBias(&BUS2_CM1Encoder ,msg):EncoderProcess(&BUS2_CM1Encoder ,msg);       //获取到编码器的初始偏差值            									
				}
				break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{
						LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
						(can2_count<=50) ? GetEncoderBias(&BUS2_CM2Encoder ,msg):EncoderProcess(&BUS2_CM2Encoder ,msg);       //获取到编码器的初始偏差值            									
				}
				break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{
						LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
						(can2_count<=50) ? GetEncoderBias(&BUS2_CM3Encoder ,msg):EncoderProcess(&BUS2_CM3Encoder ,msg);       //获取到编码器的初始偏差值            									
				}
				break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{
						LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
						(can2_count<=50) ? GetEncoderBias(&BUS2_CM4Encoder ,msg):EncoderProcess(&BUS2_CM4Encoder ,msg);       //获取到编码器的初始偏差值            									
				}
				break;
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID://下拨盘
				{
						LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
						(can2_count<=50) ? GetEncoderBias(&BUS2_CM5_Poke_Encoder,msg):EncoderProcess(&BUS2_CM5_Poke_Encoder,msg);
				}
				break;
				case CAN_BUS2_YAW_FEEDBACK_MSG_ID://CAN_BUS2_YAW_FEEDBACK_MSG_ID
				{
						LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
						YawEncoderProcess(&GMYawEncoder ,msg);
						// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
						// if(can_count>=90 && can_count<=100)
						if(GetWorkState() == PREPARE_STATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
						{
								if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-32700)
								{
										GMYawEncoder.ecd_bias =GMYawEncoder_Offset + 65536;
								}
								else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 32700)
								{
										GMYawEncoder.ecd_bias = GMYawEncoder_Offset - 65536;
								}
						}
				}break;
//				case CAN_BUS2_POWER_FEEDBACK_MSG_ID:
//				{
//					not_receive_time=0;
//#if  CAP_CTRL == 0
////					if((u8)(msg->Data[0]) == 0x00)
//			{
//				capacitance_message2.fault_union.fault = (u8)(msg->Data[0]);
//				capacitance_message2.system_mode = (u8)(msg->Data[1]);
//				capacitance_message1.cap_voltage = 0.1f*((int16_t)(msg->Data[2]<<8)|( int16_t)(msg->Data[3]));
//			}
//#elif CAP_CTRL == 1

//			if((u8)(msg->Data[0]) == 0x01)
//				{
//					CapacitanceProcess1(&capacitance_message1,msg);
//				}
//			else if((u8)(msg->Data[0]) == 0x02)
//				{
//					CapacitanceProcess2(&capacitance_message2,msg);
//				}
//#endif
//				}
//				break;

		//超级电容PM01
				case 0x610:
				{
					capacitance_message3.mode=(msg->Data[0]<<8)|msg->Data[1];//模块状态
					capacitance_message3.mode_sure=(msg->Data[2]<<8)|msg->Data[3];//故障提示
				}break; 
				case 0x611:
				{
					capacitance_message3.in_power=(msg->Data[0]<<8)|msg->Data[1];//输入功率
					capacitance_message3.in_v=(msg->Data[2]<<8)|msg->Data[3];//输入电压
					capacitance_message3.in_i=(msg->Data[4]<<8)|msg->Data[5];//输入电流
				}break; 
				case 0x612:
				{
					capacitance_message3.out_power=(msg->Data[0]<<8)|msg->Data[1];//输出功率
					capacitance_message3.out_v=(msg->Data[2]<<8)|msg->Data[3];//输出电压
					capacitance_message3.out_i=(msg->Data[4]<<8)|msg->Data[5];//输出电流
				}break; 
				case 0x613:
				{
					capacitance_message3.tempureture=(msg->Data[0]<<8)|msg->Data[1];//温度
					capacitance_message3.time=(msg->Data[2]<<8)|msg->Data[3];//累计运行时间
					capacitance_message3.this_time=(msg->Data[4]<<8)|msg->Data[5];//本次运行时间
				}break; 
				default:
				{
				}
		}
		LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
//		if(!LostCounterOverflowCheck(fabs(pid_yaw.get), 70.0f) || GetWorkState() == STOP_STATE)  //如果是停止模式，一直喂狗防止重新启动失败
//    {
//      LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
//    }
}

//void POWER_Control(u8* msg)//给超级电容发数据		超级电容与CAN2连接		自研小方盒
//{
//    CanTxMsg tx_message;
//    
//    tx_message.StdId = 0x405;//send to  controll board
//    tx_message.IDE = CAN_Id_Standard;
//    tx_message.RTR = CAN_RTR_Data;
//    tx_message.DLC = 0x08;
//    
//    tx_message.Data[0] = msg[0];
//    tx_message.Data[1] = msg[1];
//    tx_message.Data[2] = msg[2];
//    tx_message.Data[3] = msg[3];
//    tx_message.Data[4] = msg[4];
//    tx_message.Data[5] = msg[5];
//    tx_message.Data[6] = msg[6];
//    tx_message.Data[7] = msg[7];
//    
//    CAN_Transmit(CAN2,&tx_message);
//}

void POWER_Control1(uint16_t Power,uint16_t StdId)		//成品模块PM01
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (Power >> 8);
    tx_message.Data[1] = Power;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CAN2,&tx_message);
}

void POWER_Control2(uint16_t StdId)		//暂时没用
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Remote;//CAN_RTR_Data;
    tx_message.DLC = 0x06;
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;

    CAN_Transmit(CAN2,&tx_message);
}

/**************************************************************************************************
   给电调板发送指令，ID号为0x200返回ID为0x201-0x204 ****** CM3508 M2006 ****** >>底盘，摩擦轮
**************************************************************************************************/
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}

/**************************************************************************************************
   给电机电调板发送指令，ID号为0x1FF，数据回传ID为0x205-0x208  ****** M3508 M2006 ******	ID 5-8
	 给电机电调板发送指令，ID号为0x1FF,数据回传ID为0x205-0x208	 ****** GM6020 ******		ID 1-4
	 cyq:更改为发送三个电调的指令。  >>上下拨盘，倍镜
**************************************************************************************************/
void Set_GM_CM_Current(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (unsigned char)(cm5_iq >> 8);
    tx_message.Data[1] = (unsigned char)cm5_iq;
    tx_message.Data[2] = (unsigned char)(cm6_iq >> 8);
    tx_message.Data[3] = (unsigned char)cm6_iq;
    tx_message.Data[4] = (unsigned char)(cm7_iq >> 8);
    tx_message.Data[5] = (unsigned char)cm7_iq;
    tx_message.Data[6] = (unsigned char)(cm8_iq >> 8);
    tx_message.Data[7] = (unsigned char)cm8_iq;
    CAN_Transmit(CANx,&tx_message);
}

/**************************************************************************************************
   给电机电调板发送指令，ID号为0x2FF，数据回传ID为0x209-0x211  ****** GM6020 ******  ID 5-7
	 cyq:更改为发送三个电调的指令。  >>图传云台
**************************************************************************************************/
void Set_GM_Current(CAN_TypeDef *CANx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x2FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (unsigned char)(cm5_iq >> 8);
    tx_message.Data[1] = (unsigned char)cm5_iq;
    tx_message.Data[2] = (unsigned char)(cm6_iq >> 8);
    tx_message.Data[3] = (unsigned char)cm6_iq;
    tx_message.Data[4] = (unsigned char)(cm7_iq >> 8);
    tx_message.Data[5] = (unsigned char)cm7_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

/**************************************************************************************************
   给电调板发送指令，ID号为0x140+ID返回ID为0x140+ID   ****** MF5015******  ID 0-32 >>Pitch轴
**************************************************************************************************/
void Set_Pitch_Current(CAN_TypeDef *CANx, int16_t control)//MF5015
{
  CanTxMsg tx_message;
  tx_message.StdId = 0x141;//send to gyro controll board
  tx_message.IDE = CAN_Id_Standard;
  tx_message.RTR = CAN_RTR_Data;
  tx_message.DLC = 0x08;
  tx_message.Data[0] = 0xA1;
  tx_message.Data[1] = 0x00;
  tx_message.Data[2] = 0x00;
  tx_message.Data[3] = 0x00;
  tx_message.Data[4] = *(uint8_t*)(&control);//(uint8_t)control
  tx_message.Data[5] = *((uint8_t*)(&control)+1);//(unsigned char)
  tx_message.Data[6] = 0x00;
  tx_message.Data[7] = 0x00;
  CAN_Transmit(CANx,&tx_message);
}

//void Set_Pitch_Current(CAN_TypeDef *CANx, int16_t cm4_iq)//M3508
//{
//    CanTxMsg tx_message;
//    tx_message.StdId = 0x200;
//    tx_message.IDE = CAN_Id_Standard;//send to gyro controll board
//    tx_message.RTR = CAN_RTR_Data;
//    tx_message.DLC = 0x08;
//    tx_message.Data[0] = 0;
//    tx_message.Data[1] = 0;
//    tx_message.Data[2] = 0;
//    tx_message.Data[3] = 0;
//    tx_message.Data[4] = 0;
//    tx_message.Data[5] = 0;
//    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
//    tx_message.Data[7] = (uint8_t)cm4_iq;
//    CAN_Transmit(CANx,&tx_message);
//}

/**************************************************************************************************
   给电调板发送指令，ID号为0x140+ID返回ID为0x140+ID   ****** MF9025 ******  ID 0-32  >>Yaw轴
**************************************************************************************************/
void Set_Yaw_Current(CAN_TypeDef *CANx, int16_t control)//MF9025
{
  CanTxMsg tx_message;
  tx_message.StdId = 0x142;//send to gyro controll board
  tx_message.IDE = CAN_Id_Standard;
  tx_message.RTR = CAN_RTR_Data;
  tx_message.DLC = 0x08;
  tx_message.Data[0] = 0xA1;
  tx_message.Data[1] = 0x00;
  tx_message.Data[2] = 0x00;
  tx_message.Data[3] = 0x00;
  tx_message.Data[4] = (uint8_t)control;
  tx_message.Data[5] = (unsigned char)(control >> 8);
  tx_message.Data[6] = 0x00;
  tx_message.Data[7] = 0x00;
  CAN_Transmit(CANx,&tx_message);
}

/**************************************************************************************************
		设置yaw轴9025电机、pitch轴5015电机零点
**************************************************************************************************/
void Set_EncodeOffset(CAN_TypeDef *CANx)
{
  CanTxMsg tx_message;
  tx_message.StdId = 0x142;//send to gyro controll board
  tx_message.IDE = CAN_Id_Standard;
  tx_message.RTR = CAN_RTR_Data;
  tx_message.DLC = 0x08;
  tx_message.Data[0] = 0x19;
  tx_message.Data[1] = 0x00;
  tx_message.Data[2] = 0x00;
  tx_message.Data[3] = 0x00;
  tx_message.Data[4] = 0x00;
  tx_message.Data[5] = 0x00;
  tx_message.Data[6] = 0x00;
  tx_message.Data[7] = 0x00;
  CAN_Transmit(CANx,&tx_message);
}
