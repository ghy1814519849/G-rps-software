#include "main.h"

uint32_t receive_Dart;
uint32_t receive_Sentinel;
uint16_t Test_Info = 0;
reserve_aerial_info_t VehicleReserveInfo;
uint8_t VehicleShootFlagLow = 1;
uint8_t VehicleShootFlagUp = 1;
uint8_t VehicleGimbalFlag = 0;
void Vehicle_Send_Data ( void )   //�Զ��� ���� ͨѶΪ0x203
{
    //ID��ʶ�������ݾ��庬�壺0x200-0x2FFΪѧ����ͨ�ţ��˴�����Ϊ0x203�����ն���Ҫ��Ӧ
    ddata[0] = 0x03;
    ddata[1] = 0x02;

    //���Ͷ˻�����id�������ֽ�
    ddata[2] = ( judge_rece_mesg.game_robot_state.robot_id & 0x00FF );
    ddata[3] = ( judge_rece_mesg.game_robot_state.robot_id & 0xFF00 ) >> 8;

    //Ŀ�������id�������ֽ�
    if ( judge_rece_mesg.game_robot_state.robot_id < 10 ) //�췽
    {
        ddata[4] = 0x03;     //0x03Ϊ�췽3�Ų���
        ddata[5] = 0x00;
    }
    else
    {
        ddata[4] = 0x67;     //0x67Ϊ����3�Ų���,ʮ����103
        ddata[5] = 0x00;
    }
				
		if ( hero_remain_hp <= 100 )
    {
			//�Ӳ���ϵͳ���յ�Ӣ��Ѫ����2���ֽڣ�����Ѫ�����ܻ����256����ͨѶʱ���Ӳ���ϵͳ���յ������ֽڶ����ͳ�ȥ
			ddata[6] = ( hero_remain_hp & 0x00FF );//��8λ  0~256
			ddata[7] = ( hero_remain_hp & 0xFF00 ) >> 8;//��8λ	 256~65536
		}
		
		ddata[8] = 0x42;//���ݶ� ��ʮ��������ʮ����û������
		ddata[9] = 66;

    data_upload_handle ( STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata, 10, DN_REG_ID, tx_buf );
    //TUDENT_INTERACTIVE_HEADER_DATA_IDΪ����ϵͳ���յ�ID
    //ddataΪ���͵����ݣ���������ѧ����ͨ����Ҫ��ͷid��
    //sizeof(ddata)-6Ϊ���͵�(����֡ͷID��)���ݶ�������
    //DN_REG_IDΪ����ϵͳ���յ�ͷ֡
    //���͵����ݴ�������tx_buf
}

void Vehicle_Receive ( void )
{
    if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x200 )  //������Ϣ
    {
        if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
                && ( judge_rece_mesg.student_interactive_header_data.send_ID == 4
                     ||  judge_rece_mesg.student_interactive_header_data.send_ID == 104 ) )    //���Ͷ�4  ����
        {
            receive_Dart = ( judge_rece_mesg.robot_interactive_data.data[0] )    | ( judge_rece_mesg.robot_interactive_data.data[1] << 8 ) |
                           ( judge_rece_mesg.robot_interactive_data.data[2] << 16 ) | ( judge_rece_mesg.robot_interactive_data.data[3] << 24 );
        }
    }

    else if ( judge_rece_mesg.student_interactive_header_data.data_cmd_id == 0x201 ) //�ڱ���Ϣ
    {
        if ( ( judge_rece_mesg.game_robot_state.robot_id == judge_rece_mesg.student_interactive_header_data.receiver_ID )
                && ( judge_rece_mesg.student_interactive_header_data.send_ID == 1
                     ||  judge_rece_mesg.student_interactive_header_data.send_ID == 101 ) )    //���Ͷ�1  Ӣ��
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

