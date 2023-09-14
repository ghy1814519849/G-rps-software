#include "main.h"

int receive = 0;    //���յ����ݸ���
location new_location;            //�Ӿ�������������ֵ
/* *** *** ********************** *** *** */
robot_color_e robot_color = blue ;   //0-9���±�ʶ�Լ����Ǻ췽��������������
int wExpected = 0;   //CRCУ��
HostToDevice__Frame *Uart4_Protobuf_Receive_Gimbal_Angle;//����

void targetOffsetDataDeal ( uint8_t len, u8 *buf )
{
  receive++;
  process_general_message(buf,len);
}

#ifndef DEBUG_MODE
#define DEBUG_MODE
#endif

#ifdef DEBUG_MODE
  // Counter for received packages.
  int counter_receive = 0;//���յ��������ݵĸ���
  // Counter for packages that passed the head and tail check.
  int counter_complete = 0;//�ɹ����յ���֡�ĸ���
  // Contuner for pacakages that passed the CRC8 check.
  int counter_crc_passed = 0;//��CRCУ���֡�ĸ���
#endif
u8 first_len[4];
unsigned short content_size;
unsigned short content_size1;
float ab1,ab2;
float flag_x,flag_y;
float last_x,last_y;
//int dis=0;
int i=0;
unsigned char getaddress[100];
void process_general_message(unsigned char* address, unsigned int length)
{
	#ifdef DEBUG_MODE
 	 if (length > 0) ++counter_receive;          //���ӽ��յ����ݣ�����
	#endif
	
	if(address[0] != 0xBE)                       //�жϰ�ͷ�Ƿ���ȷ
		return;
		
	i=0;
	for(u8 k=0;k<length;k++)                     //0xEDӦ���Ǹ���־λ��Ȼ���������0xED��ֵ����Ŷ�����ס
	{
		if(address[k]==0xED)                        
		{
			first_len[i]=k;
			i++;
		}	
	}
	content_size=first_len[0]-3;
	content_size1 = address[1];

  if(address[content_size+3] != 0xED) 
		return;
	for(int k=0;k<content_size;k++)//ȥ��֡ͷ0xBE
		getaddress[k]=address[k+2];	

#ifdef DEBUG_MODE
 ++counter_complete;
#endif

unsigned char* content_address;
	content_address = getaddress;
unsigned char crc8 = address[2 + content_size];
	ab1=crc8;
	ab2=get_crc8(content_address, content_size);
  if(crc8 != get_crc8(content_address, content_size)) 
		return;
	
#ifdef DEBUG_MODE
	++counter_crc_passed;
#endif	
	
	Uart4_Protobuf_Receive_Gimbal_Angle=host_to_device__frame__unpack(NULL,content_size,content_address);
	flag_x=Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_;
	flag_y=Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_;
	if(flag_x!=0&&flag_y!=0)                    //����յ�������
		{
			new_location.x=  Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_;//�Ӿ�������
			new_location.y=  Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_;
			new_location.dis= Uart4_Protobuf_Receive_Gimbal_Angle->distance;
			new_location.pitch_speed = Uart4_Protobuf_Receive_Gimbal_Angle->pitchspeed;
			new_location.yaw_speed = Uart4_Protobuf_Receive_Gimbal_Angle->yawspeed;
			new_location.shot_flag= Uart4_Protobuf_Receive_Gimbal_Angle->shoot;
			new_location.flag= 1;
			LASER_ON();
		}
	else 
		{
			LASER_OFF();
			new_location.flag=0;			
		}
		
		if((last_x == new_location.x)&&(last_y == new_location.y))
		{
			LASER_OFF();
			new_location.flag=0;
		}
		last_x = new_location.x;
		last_y = new_location.y;  
			
		host_to_device__frame__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
}


DeviceToHost__Frame msg;

u8 DateLength;
void send_protocol(float x,float y)
{
  device_to_host__frame__init(&msg);
	
	if(switch_mode_flag==0)
		msg.mode_= 0;
	else if(switch_mode_flag==2)
		msg.mode_= 3;
	
  msg.current_pitch_= y;
	msg.current_yaw_= x;
	msg.current_color_=judge_rece_mesg.game_robot_state.robot_id;
	msg.bullet_speed_=12;//judge_rece_mesg.shoot_data.bullet_speed;//
	
  device_to_host__frame__pack(&msg,UART4_DMA_TX_BUF+2);
  DateLength=device_to_host__frame__get_packed_size(&msg);
  UART4_DMA_TX_BUF[0]=0xBE;
	UART4_DMA_TX_BUF[1]=DateLength;
  Append_CRC8_Check_Sum(&UART4_DMA_TX_BUF[2],DateLength+1);
  UART4_DMA_TX_BUF[DateLength+3]=0xED;
	Uart4SendBytesInfoProc(UART4_DMA_TX_BUF,DateLength+4);

}
