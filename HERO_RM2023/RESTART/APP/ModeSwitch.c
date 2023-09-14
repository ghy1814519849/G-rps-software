#include "main.h"

//ģʽ�л�����
void mode_switch_task(void)
{
		get_gimbal_mode();    
    get_chassis_mode();
}

//
void get_gimbal_mode(void)
{
  if (gim.ctrl_mode==GIMBAL_RELAX && GetInputMode() != STOP)     //ģʽ��Ϊֹͣ   ����  ģʽΪ GIMBAL_RELAX   ����ͨģʽʱ����̨��ʼ����
  {
      gim.ctrl_mode = GIMBAL_INIT;
			gimbal_back_param();                                //б�¿������磬��ʼֵ��ֵ����ʱ��̨���ܶ�
	}
	if (GetInputMode() == STOP)                             //��̨�ؿ�
	{
			gim.ctrl_mode = GIMBAL_RELAX;
			autoshoot_mode = NORMAL_SHOOT;
			position_set_flag=0;
			gimbal_back_param();
    
	}
}

//��ȡ���̵�ģʽ
void get_chassis_mode(void)
{
	  //�����̨��ģʽΪ����ʼ��ģʽ  ����  �����̣�ֹͣģʽ  ����  �ؿ�ģʽ     ���̵�ģʽΪ �ؿ�ģʽ
	  if (gim.ctrl_mode == GIMBAL_INIT || GetInputMode() == STOP || gim.ctrl_mode == GIMBAL_RELAX)
	  {
			chassis.ctrl_mode = CHASSIS_RELAX;
	  }
		//������̲�Ϊ����ģʽ ������  ��Ϊ ������̨����ģʽ  ���� ��Ϊ ���̵ߵ�ģʽ  ����  ��Ϊ����ģʽ  ���������õ��̵�ģʽ��Ϊ���̸�����̨��ģʽ
	  else if((chassis.ctrl_mode != CHASSIS_ROTATE)&&(chassis.ctrl_mode != MANUAL_SEPARATE_GIMBAL)&&(chassis.ctrl_mode != CHASSIS_REVERSE)&&(gim.ctrl_mode != GIMBAL_AUTO_ANGLE)&&(autoshoot_mode != AUTO_SHOOT))
	  {
			chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
	  } 
}

//��̨�ɿ�
uint8_t gimbal_is_controllable(void)
{
	//�����̨��ģʽΪ �ؿ�  ����  ���ģʽΪ�ر�   ����  �д��� �������� ��ʱ����0  ���򷵻�1
  if (gim.ctrl_mode == GIMBAL_RELAX || GetInputMode() == STOP || Is_Lost_Error_Set(LOST_ERROR_RC)
      
			
//			||Is_Lost_Error_Set(LOST_ERROR_IMU) 
			)
    return 0;
  else
    return 1;
}

//���̿ɿ�   
uint8_t chassis_is_controllable(void)
{
	//��� ���̵�ģʽΪ �ؿ�  ����  ���Ϊֹͣ   ����  ���ִ���  ����������ʱ����0�����򷵻�1
  if (chassis.ctrl_mode == CHASSIS_RELAX
      ||GetInputMode() == STOP
			||Is_Lost_Error_Set(LOST_ERROR_IMU)
     )
    return 0;
  else
    return 1;
}
