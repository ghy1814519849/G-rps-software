#include "main.h"

uint32_t receive_Dart;
uint32_t receive_Sentinel;
uint16_t Test_Info = 0;
reserve_aerial_info_t VehicleReserveInfo;
uint8_t VehicleShootFlagLow = 1;
uint8_t VehicleShootFlagUp = 1;
uint8_t VehicleGimbalFlag = 0;
void Vehicle_Send_Data ( void )   //自定义 本次 通讯为0x203
{
    //ID标识发送数据具体含义：0x200-0x2FF为学生端通信，此处发送为0x203，接收端需要对应
    ddata[0] = 0x03;
    ddata[1] = 0x02;

    //发送端机器人id，两个字节
    ddata[2] = ( judge_rece_mesg.game_robot_state.robot_id & 0x00FF );
    ddata[3] = ( judge_rece_mesg.game_robot_state.robot_id & 0xFF00 ) >> 8;

    //目标机器人id，两个字节
    if ( judge_rece_mesg.game_robot_state.robot_id < 10 ) //红方
    {
        ddata[4] = 0x03;     //0x03为红方3号步兵
        ddata[5] = 0x00;
    }
    else
    {
        ddata[4] = 0x67;     //0x67为蓝方3号步兵,十进制103
        ddata[5] = 0x00;
    }
				
		if ( hero_remain_hp <= 100 )
    {
			//从裁判系统接收的英雄血量是2个字节，由于血量可能会高于256，故通讯时将从裁判系统接收的两个字节都发送出去
			ddata[6] = ( hero_remain_hp & 0x00FF );//低8位  0~256
			ddata[7] = ( hero_remain_hp & 0xFF00 ) >> 8;//高8位	 256~65536
		}
		
		ddata[8] = 0x42;//数据段 发十六进制与十进制没有区别
		ddata[9] = 66;

    data_upload_handle ( STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata, 10, DN_REG_ID, tx_buf );
    //TUDENT_INTERACTIVE_HEADER_DATA_ID为裁判系统接收的ID
    //ddata为发送的数据（包含各种学生端通信需要的头id）
    //sizeof(ddata)-6为发送的(除了帧头ID的)数据段数据量
    //DN_REG_ID为裁判系统接收的头帧
    //发送的数据处理保存至tx_buf
}

void Vehicle_Receive ( void )
{
    if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x200 )  //飞镖信息
    {
        if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
                && ( judge_rece_mesg.student_interactive_header_data.send_ID == 4
                     ||  judge_rece_mesg.student_interactive_header_data.send_ID == 104 ) )    //发送端4  步兵
        {
            receive_Dart = ( judge_rece_mesg.robot_interactive_data.data[0] )    | ( judge_rece_mesg.robot_interactive_data.data[1] << 8 ) |
                           ( judge_rece_mesg.robot_interactive_data.data[2] << 16 ) | ( judge_rece_mesg.robot_interactive_data.data[3] << 24 );
        }
    }

    else if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x201 ) //哨兵信息
    {
        if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
                && ( judge_rece_mesg.student_interactive_header_data.send_ID == 1
                     ||  judge_rece_mesg.student_interactive_header_data.send_ID == 101 ) )    //发送端1  英雄
        {
            receive_Sentinel = ( judge_rece_mesg.robot_interactive_data.data[0] )    | ( judge_rece_mesg.robot_interactive_data.data[1] << 8 ) |
                               ( judge_rece_mesg.robot_interactive_data.data[2] << 16 ) | ( judge_rece_mesg.robot_interactive_data.data[3] << 24 );
        }
    }
}

void Vehicle_Com_Task ( void )
{
   Vehicle_Send_Data();
	Vehicle_Receive();
}

