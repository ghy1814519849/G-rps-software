#include "main.h"

u8  draw_cnt=0;
u16 draw_data_ID=0x0101;
u16 data_ID=0xD180;
u16 client_custom_ID=0;

ext_client_custom_character_t client_custom_friction_rotor_state;
ext_client_custom_character_t client_custom_friction_rotor_state1;
ext_client_custom_character_t client_custom_seperate_state;
ext_client_custom_character_t client_custom_shot_state;
ext_client_custom_character_t client_custom_cale_character;//rotate
ext_client_custom_character_t client_custom_radar_dis;//测距
ext_client_custom_character_t client_custom_dis_state;//测距字符
ext_client_custom_character_t client_custom_roll_Angle;//pitch轴角度
ext_client_custom_character_t client_angle_state;//pitch轴角度字符
ext_client_custom_character_t	cilent_clean_poke_lock;//清除卡弹程序

ext_client_custom_graphic_single_t client_custom_graphic_vol;//电量
ext_client_custom_graphic_double_t client_custom_graphic_shot_state;//发射
ext_client_custom_graphic_five_t client_custom_graphic_five1;//小陀螺提示
ext_client_custom_graphic_seven_t client_custom_graphic_seven_aim1;//瞄准线
ext_client_custom_graphic_seven_t client_custom_graphic_seven_auto;//吊射模式提示
int robot_level_flag;

extern shoot_state_e shot_state;
#define START_POINT_X 0
#define START_POINT_Y -50
#define END_POINT_X   0
#define END_POINT_Y   50
#define RADIOS    20
#define WIDTH    2
#define OFFSET_X 1600
#define OFFSET_Y 600

typedef struct {
int16_t x;
int16_t y;
}point;

point rotate_point(int16_t x,int16_t y,float angle)
{
	 point result;
	 float rad_angle=angle*ANGLE_TO_RAD;
   result.x=(int)(x*cos(rad_angle)-y*sin(rad_angle));
   result.y=(int)(x*sin(rad_angle)+y*cos(rad_angle));
   return result;
}
void Client_send_handle()//x=1920*y=1080
{
		if(judge_rece_mesg.game_robot_state.robot_id==1)//红色英雄
	  {
		  client_custom_ID=0x0101;//红色英雄客户端ID
	  }
	  if	(judge_rece_mesg.game_robot_state.robot_id==101)//蓝色英雄
	  {
		  client_custom_ID=0x0165;//蓝色英雄
	  }	  
	  switch(draw_cnt)
	  {
			case 1:
			{
				ddata[0]=0x0104;
	      ddata[1]=0x0104>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
			/*********************瞄准线显示****************************************/
						
				client_custom_graphic_seven_aim1.grapic_data_struct[0].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[0].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[0].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[0].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[0].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[0].graphic_name[2]=0;
				client_custom_graphic_seven_aim1.grapic_data_struct[0].start_x=760;
				client_custom_graphic_seven_aim1.grapic_data_struct[0].start_y=460;
				client_custom_graphic_seven_aim1.grapic_data_struct[0].end_y=460;
				client_custom_graphic_seven_aim1.grapic_data_struct[0].end_x=1160;//400，准心下2
				client_custom_graphic_seven_aim1.grapic_data_struct[0].color=3;
				client_custom_graphic_seven_aim1.grapic_data_struct[0].width=2;								
				
				client_custom_graphic_seven_aim1.grapic_data_struct[1].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[1].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[1].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[1].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[1].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[1].graphic_name[2]=1;
				client_custom_graphic_seven_aim1.grapic_data_struct[1].start_x=800;
				client_custom_graphic_seven_aim1.grapic_data_struct[1].start_y=420;
				client_custom_graphic_seven_aim1.grapic_data_struct[1].end_y=420;
				client_custom_graphic_seven_aim1.grapic_data_struct[1].end_x=1120;//320准心下3
				client_custom_graphic_seven_aim1.grapic_data_struct[1].color=4;
				client_custom_graphic_seven_aim1.grapic_data_struct[1].width=2;			
				
				client_custom_graphic_seven_aim1.grapic_data_struct[2].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[2].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[2].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[2].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[2].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[2].graphic_name[2]=2;
				client_custom_graphic_seven_aim1.grapic_data_struct[2].start_x=840;
				client_custom_graphic_seven_aim1.grapic_data_struct[2].start_y=380;
				client_custom_graphic_seven_aim1.grapic_data_struct[2].end_y=380;
				client_custom_graphic_seven_aim1.grapic_data_struct[2].end_x=1080;//240，准心下4
				client_custom_graphic_seven_aim1.grapic_data_struct[2].color=5;
				client_custom_graphic_seven_aim1.grapic_data_struct[2].width=2;				
				
				client_custom_graphic_seven_aim1.grapic_data_struct[3].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[3].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[3].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[3].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[3].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[3].graphic_name[2]=3;
				client_custom_graphic_seven_aim1.grapic_data_struct[3].start_x=880;
				client_custom_graphic_seven_aim1.grapic_data_struct[3].start_y=340;
				client_custom_graphic_seven_aim1.grapic_data_struct[3].end_y=340;
				client_custom_graphic_seven_aim1.grapic_data_struct[3].end_x=1040;//160，准心下5
				client_custom_graphic_seven_aim1.grapic_data_struct[3].color=6;
				client_custom_graphic_seven_aim1.grapic_data_struct[3].width=2;
				
				client_custom_graphic_seven_aim1.grapic_data_struct[4].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[4].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[4].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[4].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[4].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[4].graphic_name[2]=4;
				client_custom_graphic_seven_aim1.grapic_data_struct[4].start_x=920;
				client_custom_graphic_seven_aim1.grapic_data_struct[4].start_y=300;				
				client_custom_graphic_seven_aim1.grapic_data_struct[4].end_y=300;
				client_custom_graphic_seven_aim1.grapic_data_struct[4].end_x=1000;//80，准心下6
				client_custom_graphic_seven_aim1.grapic_data_struct[4].color=2;
				client_custom_graphic_seven_aim1.grapic_data_struct[4].width=2;

				client_custom_graphic_seven_aim1.grapic_data_struct[5].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[5].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[5].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[5].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[5].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[5].graphic_name[2]=5;
				client_custom_graphic_seven_aim1.grapic_data_struct[5].start_x=940;
				client_custom_graphic_seven_aim1.grapic_data_struct[5].start_y=260;				
				client_custom_graphic_seven_aim1.grapic_data_struct[5].end_y=260;
				client_custom_graphic_seven_aim1.grapic_data_struct[5].end_x=980;//40，准心下7
				client_custom_graphic_seven_aim1.grapic_data_struct[5].color=2;
				client_custom_graphic_seven_aim1.grapic_data_struct[5].width=2;

			  client_custom_graphic_seven_aim1.grapic_data_struct[6].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_seven_aim1.grapic_data_struct[6].layer=1;   //图层
				client_custom_graphic_seven_aim1.grapic_data_struct[6].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_seven_aim1.grapic_data_struct[6].graphic_name[0]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[6].graphic_name[1]=0;	
				client_custom_graphic_seven_aim1.grapic_data_struct[6].graphic_name[2]=6;
				client_custom_graphic_seven_aim1.grapic_data_struct[6].start_x=960;
				client_custom_graphic_seven_aim1.grapic_data_struct[6].start_y=220;//260;				
				client_custom_graphic_seven_aim1.grapic_data_struct[6].end_y=520;
				client_custom_graphic_seven_aim1.grapic_data_struct[6].end_x=960;//下竖线
				client_custom_graphic_seven_aim1.grapic_data_struct[6].color=1;
				client_custom_graphic_seven_aim1.grapic_data_struct[6].width=1;

				*(ext_client_custom_graphic_seven_t*)(&ddata[6])=client_custom_graphic_seven_aim1;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_graphic_seven_aim1),DN_REG_ID,tx_buf);
			}break;
			case 2:
			{
				ddata[0]=0x0110;
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

			/*********************pitch角度字符****************************************/
					client_angle_state.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_angle_state.grapic_data_struct.layer=1;   //图层
					client_angle_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_angle_state.grapic_data_struct.graphic_name[0]=0;	
					client_angle_state.grapic_data_struct.graphic_name[1]=0;	
					client_angle_state.grapic_data_struct.graphic_name[2]=7;
					
					client_angle_state.grapic_data_struct.start_x=1600;
					client_angle_state.grapic_data_struct.start_y=470;
					client_angle_state.grapic_data_struct.width=2;
					client_angle_state.grapic_data_struct.start_angle=20;
					client_angle_state.grapic_data_struct.end_angle=9;
					client_angle_state.grapic_data_struct.color=UI_WHITE;
					client_angle_state.data[0]='P';
					client_angle_state.data[1]='i';
					client_angle_state.data[2]='t';
					client_angle_state.data[3]='c';
					client_angle_state.data[4]='h';
					client_angle_state.data[5]='_';
			
				*(ext_client_custom_character_t*)(&ddata[6])=client_angle_state;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_angle_state),DN_REG_ID,tx_buf);
			}
			break;
			case 3:
			{
				ddata[0]=0x0110;
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
			/*********************测距字符****************************************/
					client_custom_dis_state.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_dis_state.grapic_data_struct.layer=2;   //图层
					client_custom_dis_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_dis_state.grapic_data_struct.graphic_name[0]=0;	
					client_custom_dis_state.grapic_data_struct.graphic_name[1]=0;	
					client_custom_dis_state.grapic_data_struct.graphic_name[2]=8;
					
					client_custom_dis_state.grapic_data_struct.start_x=1600;
					client_custom_dis_state.grapic_data_struct.start_y=510;
					client_custom_dis_state.grapic_data_struct.width=2;
					client_custom_dis_state.grapic_data_struct.start_angle=20;
					client_custom_dis_state.grapic_data_struct.end_angle=9;
					client_custom_dis_state.grapic_data_struct.color=UI_WHITE;
					client_custom_dis_state.data[0]='D';
					client_custom_dis_state.data[1]='i';
					client_custom_dis_state.data[2]='s';
					client_custom_dis_state.data[3]='t';
					client_custom_dis_state.data[4]='_';

				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_dis_state;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_dis_state),DN_REG_ID,tx_buf);
			}
			break;
			case 4:
			{
				ddata[0]=0x0101;	
	      ddata[1]=0x0101>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
				/*************************超级电容电量显示*******************************/
			
				client_custom_graphic_vol.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_vol.grapic_data_struct.layer=1;   //图层
				client_custom_graphic_vol.grapic_data_struct.graphic_type=4;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_vol.grapic_data_struct.graphic_name[0]=0;	
				client_custom_graphic_vol.grapic_data_struct.graphic_name[1]=0;	
				client_custom_graphic_vol.grapic_data_struct.graphic_name[2]=9;				
				client_custom_graphic_vol.grapic_data_struct.start_x=300;
				client_custom_graphic_vol.grapic_data_struct.start_y=580;
				client_custom_graphic_vol.grapic_data_struct.end_x=50;
				client_custom_graphic_vol.grapic_data_struct.end_y=50;
				client_custom_graphic_vol.grapic_data_struct.width=7;
				client_custom_graphic_vol.grapic_data_struct.start_angle=0;
				client_custom_graphic_vol.grapic_data_struct.end_angle=(capacitance_message1.cap_voltage_filte-12)/11.5*360;
				client_custom_graphic_vol.grapic_data_struct.color=8;

				*(ext_client_custom_graphic_single_t*)(&ddata[6])=client_custom_graphic_vol;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_graphic_vol),DN_REG_ID,tx_buf);	
			}break;		
			case 5:
			{
				ddata[0]=0x0110;	
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

			/*********************pitch轴角度显示****************************************/

				client_custom_roll_Angle.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
				client_custom_roll_Angle.grapic_data_struct.layer=1;   //图层
				client_custom_roll_Angle.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_roll_Angle.grapic_data_struct.graphic_name[0]=0;	
				client_custom_roll_Angle.grapic_data_struct.graphic_name[1]=1;	
				client_custom_roll_Angle.grapic_data_struct.graphic_name[2]=0;
				
			  client_custom_roll_Angle.grapic_data_struct.start_x=1715;
			  client_custom_roll_Angle.grapic_data_struct.start_y=470;
			  client_custom_roll_Angle.grapic_data_struct.width=WIDTH;
				client_custom_roll_Angle.grapic_data_struct.start_angle=20;
				client_custom_roll_Angle.grapic_data_struct.end_angle=2;
				
				sprintf(client_custom_roll_Angle.data,"%f",roll_Angle);
				
				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_roll_Angle;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_roll_Angle),DN_REG_ID,tx_buf);	
			}break;
			case 6:
			{
				ddata[0]=0x0110;	
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

			/*********************测距距离显示****************************************/

				client_custom_radar_dis.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
				client_custom_radar_dis.grapic_data_struct.layer=2;   //图层
				client_custom_radar_dis.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_radar_dis.grapic_data_struct.graphic_name[0]=0;	
				client_custom_radar_dis.grapic_data_struct.graphic_name[1]=1;	
				client_custom_radar_dis.grapic_data_struct.graphic_name[2]=1;
				
			  client_custom_radar_dis.grapic_data_struct.start_x=1715;
			  client_custom_radar_dis.grapic_data_struct.start_y=510;
			  client_custom_radar_dis.grapic_data_struct.width=WIDTH;
				client_custom_radar_dis.grapic_data_struct.start_angle=20;
				client_custom_radar_dis.grapic_data_struct.end_angle=2;
				
				sprintf(client_custom_radar_dis.data,"%f", TF02.Dist*0.01);
				
				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_radar_dis;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_radar_dis),DN_REG_ID,tx_buf);	
			}break;
			case 7:
			{
				ddata[0]=0x0104;	
	      ddata[1]=0x0104>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				if(switch_mode_flag == 2)
				{
					/*********************吊射模式提示****************************************/
					client_custom_graphic_seven_auto.grapic_data_struct[0].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_name[2]=2;
					client_custom_graphic_seven_auto.grapic_data_struct[0].start_x=137;
					client_custom_graphic_seven_auto.grapic_data_struct[0].start_y=630;
					client_custom_graphic_seven_auto.grapic_data_struct[0].end_y=630;
					client_custom_graphic_seven_auto.grapic_data_struct[0].end_x=163;
					client_custom_graphic_seven_auto.grapic_data_struct[0].color=2;
					client_custom_graphic_seven_auto.grapic_data_struct[0].width=2;		
					
					client_custom_graphic_seven_auto.grapic_data_struct[1].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_name[2]=3;
					client_custom_graphic_seven_auto.grapic_data_struct[1].start_x=125;
					client_custom_graphic_seven_auto.grapic_data_struct[1].start_y=530;				
					client_custom_graphic_seven_auto.grapic_data_struct[1].end_y=530;
					client_custom_graphic_seven_auto.grapic_data_struct[1].end_x=175;
					client_custom_graphic_seven_auto.grapic_data_struct[1].color=2;
					client_custom_graphic_seven_auto.grapic_data_struct[1].width=2;
					
					client_custom_graphic_seven_auto.grapic_data_struct[2].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[2].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_name[2]=4;
					client_custom_graphic_seven_auto.grapic_data_struct[2].start_x=137;
					client_custom_graphic_seven_auto.grapic_data_struct[2].start_y=630;				
					client_custom_graphic_seven_auto.grapic_data_struct[2].end_y=530;
					client_custom_graphic_seven_auto.grapic_data_struct[2].end_x=125;
					client_custom_graphic_seven_auto.grapic_data_struct[2].color=1;
					client_custom_graphic_seven_auto.grapic_data_struct[2].width=2;								
							
					client_custom_graphic_seven_auto.grapic_data_struct[3].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[3].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_name[2]=5;
					client_custom_graphic_seven_auto.grapic_data_struct[3].start_x=163;
					client_custom_graphic_seven_auto.grapic_data_struct[3].start_y=630;
					client_custom_graphic_seven_auto.grapic_data_struct[3].end_y=530;
					client_custom_graphic_seven_auto.grapic_data_struct[3].end_x=175;
					client_custom_graphic_seven_auto.grapic_data_struct[3].color=3;
					client_custom_graphic_seven_auto.grapic_data_struct[3].width=2;								
					
					client_custom_graphic_seven_auto.grapic_data_struct[4].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[4].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_name[2]=6;
					client_custom_graphic_seven_auto.grapic_data_struct[4].start_x=150;
					client_custom_graphic_seven_auto.grapic_data_struct[4].start_y=620;
					client_custom_graphic_seven_auto.grapic_data_struct[4].radius=5;
					client_custom_graphic_seven_auto.grapic_data_struct[4].color=4;
					client_custom_graphic_seven_auto.grapic_data_struct[4].width=2;			
					
					client_custom_graphic_seven_auto.grapic_data_struct[5].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[5].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_name[2]=7;
					client_custom_graphic_seven_auto.grapic_data_struct[5].start_x=135;
					client_custom_graphic_seven_auto.grapic_data_struct[5].start_y=610;
					client_custom_graphic_seven_auto.grapic_data_struct[5].end_y=610;
					client_custom_graphic_seven_auto.grapic_data_struct[5].end_x=165;
					client_custom_graphic_seven_auto.grapic_data_struct[5].color=5;
					client_custom_graphic_seven_auto.grapic_data_struct[5].width=2;				
					
					client_custom_graphic_seven_auto.grapic_data_struct[6].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[6].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_name[2]=8;
					client_custom_graphic_seven_auto.grapic_data_struct[6].start_x=150;
					client_custom_graphic_seven_auto.grapic_data_struct[6].start_y=610;
					client_custom_graphic_seven_auto.grapic_data_struct[6].end_y=530;
					client_custom_graphic_seven_auto.grapic_data_struct[6].end_x=150;
					client_custom_graphic_seven_auto.grapic_data_struct[6].color=6;
					client_custom_graphic_seven_auto.grapic_data_struct[6].width=2;
				}
				else
				{
					client_custom_graphic_seven_auto.grapic_data_struct[0].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[0].graphic_name[2]=2;
					
					client_custom_graphic_seven_auto.grapic_data_struct[1].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[1].graphic_name[2]=3;
					
					client_custom_graphic_seven_auto.grapic_data_struct[2].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[2].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[2].graphic_name[2]=4;
					
					client_custom_graphic_seven_auto.grapic_data_struct[3].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[3].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[3].graphic_name[2]=5;
					
					client_custom_graphic_seven_auto.grapic_data_struct[4].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[4].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[4].graphic_name[2]=6;					
					
					client_custom_graphic_seven_auto.grapic_data_struct[5].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[5].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[5].graphic_name[2]=7;
					
					client_custom_graphic_seven_auto.grapic_data_struct[6].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_seven_auto.grapic_data_struct[6].layer=1;   //图层
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_type=0;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_name[0]=0;	
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_name[1]=1;	
					client_custom_graphic_seven_auto.grapic_data_struct[6].graphic_name[2]=8;		
				}
			
				*(ext_client_custom_graphic_seven_t*)(&ddata[6])=client_custom_graphic_seven_auto;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_graphic_seven_auto),DN_REG_ID,tx_buf);				
			}break;
			case 8:
			{
				ddata[0]=0x0102;	
	      ddata[1]=0x0102>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
				/*************************发射状态提示*******************************/

				if(friction_normal_flag==1)
				{
					client_custom_graphic_shot_state.grapic_data_struct[0].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_shot_state.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[1]=1;	
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[2]=9;
					client_custom_graphic_shot_state.grapic_data_struct[0].start_x=1730;
					client_custom_graphic_shot_state.grapic_data_struct[0].start_y=400;
					client_custom_graphic_shot_state.grapic_data_struct[0].radius=10;
					client_custom_graphic_shot_state.grapic_data_struct[0].color=2;
					client_custom_graphic_shot_state.grapic_data_struct[0].width=20;
				
					client_custom_graphic_shot_state.grapic_data_struct[1].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_shot_state.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[1]=2;	
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[2]=0;
					client_custom_graphic_shot_state.grapic_data_struct[1].start_x=1790;
					client_custom_graphic_shot_state.grapic_data_struct[1].start_y=400;
					client_custom_graphic_shot_state.grapic_data_struct[1].radius=10;
					client_custom_graphic_shot_state.grapic_data_struct[1].color=2;
					client_custom_graphic_shot_state.grapic_data_struct[1].width=20;
				}
				else if(friction_normal_flag==2)
				{
					client_custom_graphic_shot_state.grapic_data_struct[0].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_shot_state.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[1]=1;	
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[2]=9;
					client_custom_graphic_shot_state.grapic_data_struct[0].start_x=1730;
					client_custom_graphic_shot_state.grapic_data_struct[0].start_y=400;
					client_custom_graphic_shot_state.grapic_data_struct[0].radius=10;
					client_custom_graphic_shot_state.grapic_data_struct[0].color=5;
					client_custom_graphic_shot_state.grapic_data_struct[0].width=20;
				
					client_custom_graphic_shot_state.grapic_data_struct[1].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_shot_state.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[1]=2;	
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[2]=0;
					client_custom_graphic_shot_state.grapic_data_struct[1].start_x=1790;
					client_custom_graphic_shot_state.grapic_data_struct[1].start_y=400;
					client_custom_graphic_shot_state.grapic_data_struct[1].radius=10;
					client_custom_graphic_shot_state.grapic_data_struct[1].color=5;
					client_custom_graphic_shot_state.grapic_data_struct[1].width=20;
				}
				else 
				{
					client_custom_graphic_shot_state.grapic_data_struct[0].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_shot_state.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[1]=1;	
					client_custom_graphic_shot_state.grapic_data_struct[0].graphic_name[2]=9;
			
					client_custom_graphic_shot_state.grapic_data_struct[1].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_shot_state.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[1]=2;	
					client_custom_graphic_shot_state.grapic_data_struct[1].graphic_name[2]=0;
				}				
				*(ext_client_custom_graphic_double_t*)(&ddata[6])=client_custom_graphic_shot_state;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_graphic_shot_state),DN_REG_ID,tx_buf);
			}
			break;
			case 9:
			{
				ddata[0]=0x0101;	
	      ddata[1]=0x0101>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
				/*************************超级电容电量显示修改*******************************/
			
				client_custom_graphic_vol.grapic_data_struct.operate_type=2;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
			  client_custom_graphic_vol.grapic_data_struct.layer=1;   //图层
				client_custom_graphic_vol.grapic_data_struct.graphic_type=4;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_graphic_vol.grapic_data_struct.graphic_name[0]=0;	
				client_custom_graphic_vol.grapic_data_struct.graphic_name[1]=0;	
				client_custom_graphic_vol.grapic_data_struct.graphic_name[2]=9;				
				client_custom_graphic_vol.grapic_data_struct.start_x=300;
				client_custom_graphic_vol.grapic_data_struct.start_y=580;
				client_custom_graphic_vol.grapic_data_struct.end_x=50;
				client_custom_graphic_vol.grapic_data_struct.end_y=50;
				client_custom_graphic_vol.grapic_data_struct.width=7;
				client_custom_graphic_vol.grapic_data_struct.start_angle=0;
				client_custom_graphic_vol.grapic_data_struct.end_angle=(capacitance_message1.cap_voltage_filte-12)/13*360;
				client_custom_graphic_vol.grapic_data_struct.color=8;
	
				*(ext_client_custom_graphic_single_t*)(&ddata[6])=client_custom_graphic_vol;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_graphic_vol),DN_REG_ID,tx_buf);									
			}	break;
			case 10:
			{
				ddata[0]=0x0103;	
	      ddata[1]=0x0103>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
				/*************************陀螺提示*******************************/
				if(chassis.ctrl_mode==CHASSIS_ROTATE)
				{
					client_custom_graphic_five1.grapic_data_struct[0].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形//中心圆
					client_custom_graphic_five1.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[0].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[0].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[0].graphic_name[2]=1;				
					client_custom_graphic_five1.grapic_data_struct[0].start_x=1760;
					client_custom_graphic_five1.grapic_data_struct[0].start_y=580;
					client_custom_graphic_five1.grapic_data_struct[0].radius=50;
					client_custom_graphic_five1.grapic_data_struct[0].width=1;
					client_custom_graphic_five1.grapic_data_struct[0].color=2;

					client_custom_graphic_five1.grapic_data_struct[1].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[1].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[1].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[1].graphic_name[2]=2;			
					client_custom_graphic_five1.grapic_data_struct[1].start_x=1730;
					client_custom_graphic_five1.grapic_data_struct[1].start_y=610;
					client_custom_graphic_five1.grapic_data_struct[1].radius=10;
					client_custom_graphic_five1.grapic_data_struct[1].width=7;
					client_custom_graphic_five1.grapic_data_struct[1].color=2;
					
					client_custom_graphic_five1.grapic_data_struct[2].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[2].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[2].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[2].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[2].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[2].graphic_name[2]=3;				
					client_custom_graphic_five1.grapic_data_struct[2].start_x=1790;
					client_custom_graphic_five1.grapic_data_struct[2].start_y=610;
					client_custom_graphic_five1.grapic_data_struct[2].radius=10;
					client_custom_graphic_five1.grapic_data_struct[2].width=7;
					client_custom_graphic_five1.grapic_data_struct[2].color=2;
					
					client_custom_graphic_five1.grapic_data_struct[3].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[3].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[3].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[3].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[3].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[3].graphic_name[2]=4;			
					client_custom_graphic_five1.grapic_data_struct[3].start_x=1730;
					client_custom_graphic_five1.grapic_data_struct[3].start_y=550;
					client_custom_graphic_five1.grapic_data_struct[3].radius=10;
					client_custom_graphic_five1.grapic_data_struct[3].width=7;
					client_custom_graphic_five1.grapic_data_struct[3].color=2;
					
					client_custom_graphic_five1.grapic_data_struct[4].operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[4].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[4].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[4].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[4].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[4].graphic_name[2]=5;				
					client_custom_graphic_five1.grapic_data_struct[4].start_x=1790;
					client_custom_graphic_five1.grapic_data_struct[4].start_y=550;
					client_custom_graphic_five1.grapic_data_struct[4].radius=10;
					client_custom_graphic_five1.grapic_data_struct[4].width=7;
					client_custom_graphic_five1.grapic_data_struct[4].color=2;
				}
				else
				{
					client_custom_graphic_five1.grapic_data_struct[0].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[0].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[0].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[0].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[0].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[0].graphic_name[2]=1;				
					client_custom_graphic_five1.grapic_data_struct[0].start_x=1760;
					client_custom_graphic_five1.grapic_data_struct[0].start_y=580;
					client_custom_graphic_five1.grapic_data_struct[0].radius=50;
					client_custom_graphic_five1.grapic_data_struct[0].width=1;
					client_custom_graphic_five1.grapic_data_struct[0].color=2;

					client_custom_graphic_five1.grapic_data_struct[1].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[1].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[1].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[1].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[1].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[1].graphic_name[2]=2;			
					client_custom_graphic_five1.grapic_data_struct[1].start_x=1730;
					client_custom_graphic_five1.grapic_data_struct[1].start_y=610;
					client_custom_graphic_five1.grapic_data_struct[1].radius=10;
					client_custom_graphic_five1.grapic_data_struct[1].width=7;
					client_custom_graphic_five1.grapic_data_struct[1].color=2;
					
					client_custom_graphic_five1.grapic_data_struct[2].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[2].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[2].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[2].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[2].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[2].graphic_name[2]=3;				
					client_custom_graphic_five1.grapic_data_struct[2].start_x=1790;
					client_custom_graphic_five1.grapic_data_struct[2].start_y=610;
					client_custom_graphic_five1.grapic_data_struct[2].radius=10;
					client_custom_graphic_five1.grapic_data_struct[2].width=7;
					client_custom_graphic_five1.grapic_data_struct[2].color=2;
					
					client_custom_graphic_five1.grapic_data_struct[3].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[3].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[3].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[3].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[3].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[3].graphic_name[2]=4;			
					client_custom_graphic_five1.grapic_data_struct[3].start_x=1730;
					client_custom_graphic_five1.grapic_data_struct[3].start_y=550;
					client_custom_graphic_five1.grapic_data_struct[3].radius=10;
					client_custom_graphic_five1.grapic_data_struct[3].width=7;
					client_custom_graphic_five1.grapic_data_struct[3].color=2;
					
					client_custom_graphic_five1.grapic_data_struct[4].operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_graphic_five1.grapic_data_struct[4].layer=1;   //图层
					client_custom_graphic_five1.grapic_data_struct[4].graphic_type=2;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_graphic_five1.grapic_data_struct[4].graphic_name[0]=0;	
					client_custom_graphic_five1.grapic_data_struct[4].graphic_name[1]=2;	
					client_custom_graphic_five1.grapic_data_struct[4].graphic_name[2]=5;						
				}	
				
				*(ext_client_custom_graphic_five_t*)(&ddata[6])=client_custom_graphic_five1;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_graphic_five1),DN_REG_ID,tx_buf);			
			}break;
			case 11:
			{
				ddata[0]=0x0110;
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

				/*************************卡弹提示*******************************/

				if((friction_rotor==friction_on)&&(BUS1_CM1Encoder.filter_rate>-20||BUS1_CM2Encoder.filter_rate<20))
				{
					client_custom_friction_rotor_state.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_friction_rotor_state.grapic_data_struct.layer=1;   //图层
					client_custom_friction_rotor_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_friction_rotor_state.grapic_data_struct.graphic_name[0]=0;	
					client_custom_friction_rotor_state.grapic_data_struct.graphic_name[1]=2;	
					client_custom_friction_rotor_state.grapic_data_struct.graphic_name[2]=6;
					
					client_custom_friction_rotor_state.grapic_data_struct.start_x=195;
					client_custom_friction_rotor_state.grapic_data_struct.start_y=700;
					client_custom_friction_rotor_state.grapic_data_struct.width=2;
					client_custom_friction_rotor_state.grapic_data_struct.start_angle=20;
					client_custom_friction_rotor_state.grapic_data_struct.end_angle=9;
					client_custom_friction_rotor_state.grapic_data_struct.color=2;
					client_custom_friction_rotor_state.data[0]='S';
					client_custom_friction_rotor_state.data[1]='H';
					client_custom_friction_rotor_state.data[2]='O';
					client_custom_friction_rotor_state.data[3]='O';
					client_custom_friction_rotor_state.data[4]='T';
					client_custom_friction_rotor_state.data[5]=':';
					client_custom_friction_rotor_state.data[6]='L';
					client_custom_friction_rotor_state.data[7]='O';
					client_custom_friction_rotor_state.data[8]='C';
					client_custom_friction_rotor_state.data[9]='K';
				}
				else
				{
					client_custom_friction_rotor_state.grapic_data_struct.operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_friction_rotor_state.grapic_data_struct.layer=1;   //图层
					client_custom_friction_rotor_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_friction_rotor_state.grapic_data_struct.graphic_name[0]=0;	
					client_custom_friction_rotor_state.grapic_data_struct.graphic_name[1]=2;	
					client_custom_friction_rotor_state.grapic_data_struct.graphic_name[2]=6;			
				}
			
				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_friction_rotor_state;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_friction_rotor_state),DN_REG_ID,tx_buf);
			}break;
			case 12:
			{
				ddata[0]=0x0110;
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

				/*************************超级电容字符显示*******************************/

				client_custom_cale_character.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
				client_custom_cale_character.grapic_data_struct.layer=1;   //图层
				client_custom_cale_character.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
				client_custom_cale_character.grapic_data_struct.graphic_name[0]=0;	
				client_custom_cale_character.grapic_data_struct.graphic_name[1]=2;	
				client_custom_cale_character.grapic_data_struct.graphic_name[2]=7;
				client_custom_cale_character.grapic_data_struct.start_x=275;
				client_custom_cale_character.grapic_data_struct.start_y=590;
				client_custom_cale_character.grapic_data_struct.width=2;
				client_custom_cale_character.grapic_data_struct.start_angle=20;
				client_custom_cale_character.grapic_data_struct.end_angle=9;
				client_custom_cale_character.grapic_data_struct.color=2;
				client_custom_cale_character.data[0]='S';
				client_custom_cale_character.data[1]='U';
				client_custom_cale_character.data[2]='C';
			
				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_cale_character;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_cale_character),DN_REG_ID,tx_buf);
			}break;
			case 13:
				{
					ddata[0]=0x0110;
					ddata[1]=0x0110>>8;	 //数据内容id  
					//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
					ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
					ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
					ddata[4]=client_custom_ID;
					ddata[5]=client_custom_ID>>8;       //客户端id
					
			/*********************底盘云台分离提示****************************************/

					if(chassis.ctrl_mode == MANUAL_SEPARATE_GIMBAL)
					{
						client_custom_seperate_state.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
						client_custom_seperate_state.grapic_data_struct.layer=1;   //图层
						client_custom_seperate_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
						client_custom_seperate_state.grapic_data_struct.graphic_name[0]=0;	
						client_custom_seperate_state.grapic_data_struct.graphic_name[1]=2;	
						client_custom_seperate_state.grapic_data_struct.graphic_name[2]=8;
						
						client_custom_seperate_state.grapic_data_struct.start_x=195;
						client_custom_seperate_state.grapic_data_struct.start_y=760;
						client_custom_seperate_state.grapic_data_struct.width=2;
						client_custom_seperate_state.grapic_data_struct.start_angle=20;
						client_custom_seperate_state.grapic_data_struct.end_angle=9;
						client_custom_seperate_state.grapic_data_struct.color=2;
						client_custom_seperate_state.data[0]='S';
						client_custom_seperate_state.data[1]='E';
						client_custom_seperate_state.data[2]='P';
						client_custom_seperate_state.data[3]='E';
						client_custom_seperate_state.data[4]='R';
						client_custom_seperate_state.data[5]='A';
						client_custom_seperate_state.data[6]='T';
						client_custom_seperate_state.data[7]='E';
						client_custom_seperate_state.data[5]=':';
						client_custom_seperate_state.data[6]='O';
						client_custom_seperate_state.data[7]='N';
					}
					else
					{
						client_custom_seperate_state.grapic_data_struct.operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
						client_custom_seperate_state.grapic_data_struct.layer=1;   //图层
						client_custom_seperate_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
						client_custom_seperate_state.grapic_data_struct.graphic_name[0]=0;	
						client_custom_seperate_state.grapic_data_struct.graphic_name[1]=2;	
						client_custom_seperate_state.grapic_data_struct.graphic_name[2]=8;			
					}
				
					*(ext_client_custom_character_t*)(&ddata[6])=client_custom_seperate_state;
					data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_seperate_state),DN_REG_ID,tx_buf);
				}
					break;
			case 14:
			{
					ddata[0]=0x0110;
					ddata[1]=0x0110>>8;	 //数据内容id  
					//0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
					ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
					ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
					ddata[4]=client_custom_ID;
					ddata[5]=client_custom_ID>>8;       //客户端id
					
			/*********************发射电机过热显示****************************************/

					if(over_hot_motor)
					{
						client_custom_shot_state.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
						client_custom_shot_state.grapic_data_struct.layer=1;   //图层
						client_custom_shot_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
						client_custom_shot_state.grapic_data_struct.graphic_name[0]=0;	
						client_custom_shot_state.grapic_data_struct.graphic_name[1]=2;	
						client_custom_shot_state.grapic_data_struct.graphic_name[2]=9;
						
						client_custom_shot_state.grapic_data_struct.start_x=1760;
						client_custom_shot_state.grapic_data_struct.start_y=730;
						client_custom_shot_state.grapic_data_struct.width=2;
						client_custom_shot_state.grapic_data_struct.start_angle=20;
						client_custom_shot_state.grapic_data_struct.end_angle=9;
						client_custom_shot_state.grapic_data_struct.color=2;
						client_custom_shot_state.data[0]='M';
						client_custom_shot_state.data[1]='O';
						client_custom_shot_state.data[2]='T';
						client_custom_shot_state.data[3]='O';
						client_custom_shot_state.data[4]='R';
						client_custom_shot_state.data[5]=':';
						client_custom_shot_state.data[6]='H';
						client_custom_shot_state.data[7]='O';
						client_custom_shot_state.data[8]='T';
					}
					else
					{
						client_custom_shot_state.grapic_data_struct.operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
						client_custom_shot_state.grapic_data_struct.layer=1;   //图层
						client_custom_shot_state.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
						client_custom_shot_state.grapic_data_struct.graphic_name[0]=0;	
						client_custom_shot_state.grapic_data_struct.graphic_name[1]=2;	
						client_custom_shot_state.grapic_data_struct.graphic_name[2]=9;			
					}
				
					*(ext_client_custom_character_t*)(&ddata[6])=client_custom_shot_state;
					data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_shot_state),DN_REG_ID,tx_buf);
			}
				break;
			case 15:
			{
				ddata[0]=0x0110;
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id
				
				if(BUS1_CM1Encoder.filter_rate<-10&&BUS1_CM2Encoder.filter_rate>10)
				{
					client_custom_friction_rotor_state1.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_friction_rotor_state1.grapic_data_struct.layer=1;   //图层
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_name[0]=0;	
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_name[1]=3;	
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_name[2]=0;
					
					client_custom_friction_rotor_state1.grapic_data_struct.start_x=195;
					client_custom_friction_rotor_state1.grapic_data_struct.start_y=700;
					client_custom_friction_rotor_state1.grapic_data_struct.width=2;
					client_custom_friction_rotor_state1.grapic_data_struct.start_angle=20;
					client_custom_friction_rotor_state1.grapic_data_struct.end_angle=9;
					client_custom_friction_rotor_state1.grapic_data_struct.color=2;
					client_custom_friction_rotor_state1.data[0]='S';
					client_custom_friction_rotor_state1.data[1]='H';
					client_custom_friction_rotor_state1.data[2]='O';
					client_custom_friction_rotor_state1.data[3]='O';
					client_custom_friction_rotor_state1.data[4]='T';
					client_custom_friction_rotor_state1.data[5]=':';
					client_custom_friction_rotor_state1.data[6]='O';
					client_custom_friction_rotor_state1.data[7]='N';
				}
				else
				{
					client_custom_friction_rotor_state1.grapic_data_struct.operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					client_custom_friction_rotor_state1.grapic_data_struct.layer=1;   //图层
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_name[0]=0;	
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_name[1]=3;	
					client_custom_friction_rotor_state1.grapic_data_struct.graphic_name[2]=0;			
				}
			
				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_friction_rotor_state1;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_friction_rotor_state1),DN_REG_ID,tx_buf);
			}
				break;
			case 16:
			{
				ddata[0]=0x0110;	
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

			/*********************测距距离修改****************************************/

				client_custom_radar_dis.grapic_data_struct.operate_type=2;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
				client_custom_radar_dis.grapic_data_struct.layer=2;   //图层
				client_custom_radar_dis.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_radar_dis.grapic_data_struct.graphic_name[0]=0;	
				client_custom_radar_dis.grapic_data_struct.graphic_name[1]=1;	
				client_custom_radar_dis.grapic_data_struct.graphic_name[2]=1;
				
			  client_custom_radar_dis.grapic_data_struct.start_x=1715;
			  client_custom_radar_dis.grapic_data_struct.start_y=510;
			  client_custom_radar_dis.grapic_data_struct.width=WIDTH;
				client_custom_radar_dis.grapic_data_struct.start_angle=20;
				client_custom_radar_dis.grapic_data_struct.end_angle=2;
				
				sprintf(client_custom_radar_dis.data,"%f", TF02.Dist*0.01);

				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_radar_dis;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_radar_dis),DN_REG_ID,tx_buf);	
			}
				break;
			case 17:
			{
				ddata[0]=0x0110;	
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

			/*********************测距距离修改****************************************/

				client_custom_roll_Angle.grapic_data_struct.operate_type=2;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
				client_custom_roll_Angle.grapic_data_struct.layer=1;   //图层
				client_custom_roll_Angle.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
		    client_custom_roll_Angle.grapic_data_struct.graphic_name[0]=0;	
				client_custom_roll_Angle.grapic_data_struct.graphic_name[1]=1;	
				client_custom_roll_Angle.grapic_data_struct.graphic_name[2]=0;
				
			  client_custom_roll_Angle.grapic_data_struct.start_x=1715;
			  client_custom_roll_Angle.grapic_data_struct.start_y=470;
			  client_custom_roll_Angle.grapic_data_struct.width=WIDTH;
				client_custom_roll_Angle.grapic_data_struct.start_angle=20;
				client_custom_roll_Angle.grapic_data_struct.end_angle=2;
				
				sprintf(client_custom_roll_Angle.data,"%f",roll_Angle);
				
				*(ext_client_custom_character_t*)(&ddata[6])=client_custom_roll_Angle;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(client_custom_roll_Angle),DN_REG_ID,tx_buf);	
			}
			break;
			case 18:
			{
				ddata[0]=0x0110;
	      ddata[1]=0x0110>>8;	 //数据内容id  
        //0x0100  删除图形 0x0101 绘制一个图形 0x0102 绘制二个图形 0x0103 绘制五个图形 0x0104绘制七个图形 0x0110客户端绘制字符图形	
			  ddata[2]=judge_rece_mesg.game_robot_state.robot_id;
	      ddata[3]=judge_rece_mesg.game_robot_state.robot_id>>8;    //机器人id
	      ddata[4]=client_custom_ID;
	      ddata[5]=client_custom_ID>>8;       //客户端id

			/*********************卡弹清除程序执行完成****************************************/

				if(lock_cilent_flag)
				{
					cilent_clean_poke_lock.grapic_data_struct.operate_type=1;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					cilent_clean_poke_lock.grapic_data_struct.layer=1;   //图层
					cilent_clean_poke_lock.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					cilent_clean_poke_lock.grapic_data_struct.graphic_name[0]=0;	
					cilent_clean_poke_lock.grapic_data_struct.graphic_name[1]=2;	
					cilent_clean_poke_lock.grapic_data_struct.graphic_name[2]=9;
					
					cilent_clean_poke_lock.grapic_data_struct.start_x=195;
					cilent_clean_poke_lock.grapic_data_struct.start_y=700;
					cilent_clean_poke_lock.grapic_data_struct.width=2;
					cilent_clean_poke_lock.grapic_data_struct.start_angle=20;
					cilent_clean_poke_lock.grapic_data_struct.end_angle=9;
					cilent_clean_poke_lock.grapic_data_struct.color=2;
					cilent_clean_poke_lock.data[0]='L';
					cilent_clean_poke_lock.data[1]='O';
					cilent_clean_poke_lock.data[2]='C';
					cilent_clean_poke_lock.data[3]='K';
					cilent_clean_poke_lock.data[4]=':';
					cilent_clean_poke_lock.data[5]='C';
					cilent_clean_poke_lock.data[6]='L';
					cilent_clean_poke_lock.data[7]='E';
					cilent_clean_poke_lock.data[8]='A';
					cilent_clean_poke_lock.data[9]='N';
				}
				else
				{
					cilent_clean_poke_lock.grapic_data_struct.operate_type=3;  //1 增加 2修改图形 3删除单个图形 5删除一个图层的图形 6删除所有图形
					cilent_clean_poke_lock.grapic_data_struct.layer=1;   //图层
					cilent_clean_poke_lock.grapic_data_struct.graphic_type=7;  //0 直线 1矩形 2整圆 3椭圆 4圆弧 5浮点数 6整数型 7字符
					cilent_clean_poke_lock.grapic_data_struct.graphic_name[0]=0;	
					cilent_clean_poke_lock.grapic_data_struct.graphic_name[1]=2;	
					cilent_clean_poke_lock.grapic_data_struct.graphic_name[2]=9;			
				}

				*(ext_client_custom_character_t*)(&ddata[6])=cilent_clean_poke_lock;
				data_upload_handle(STUDENT_INTERACTIVE_HEADER_DATA_ID, ddata,6+sizeof(cilent_clean_poke_lock),DN_REG_ID,tx_buf);
			}
				break;
			default:
				break;
		}
	draw_cnt++;
  if(draw_cnt>19 )//在需要刷新的图层刷新
		draw_cnt=0;
}

void delete_Coverage(u8 coverage)
{
	ddata[6]=4;//1增加2修改3删除单个4删除图层5删除所有
	ddata[13]=coverage;//图层0-9
}