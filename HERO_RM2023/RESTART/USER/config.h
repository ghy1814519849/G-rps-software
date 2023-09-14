#ifndef __SYS_H__
#define __SYS_H__

#define ROLL_MAX 14					//����19��
#define ROLL_MIN -35				//����48��
//��̨��ʼλ��
#define	GMPitchEncoder_Offset 0
#define	GMYawEncoder_Offset   58500
#define	CameraEncoder_Offset	1150
//��ȫ��ֵ
#define Pitch_Safe_angle_max 12
#define Pitch_Safe_angle_min -32

#define REMOTE_SHOOT       1
        //0:ң�����󲦸�ģʽ����	0:С����		1:����	2:����	3:���ϰ���

#define CAP_CTRL              				 1   //��0��1
#define  POWER_LIMT                   1   //1ʱʹ���Ͻ��㷨��0ʱʹ��ֱ���޵�����

//�������mm
#define FOCAL_LENGTH               6.0F //6.0F
//���  	泤mm
#define TARGET_SURFACE_LENGTH      4.6e-3F
//�����mm
#define TARGET_SURFACE_WIDTH        4.6e-3F
//���سߴ�mm
#define IMAGE_LENGTH                4.6e-3F

//����ͷ��ǹ�����ĵİ�װƫ���-YAW����
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		0.0f
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA  0.0f

//����ͷ��ǹ�����ĵľ��� 
#define HEIGHT_BETWEEN_GUN_CAMERA 			9.2f//9.17f

//ͼ�����̨�����ӳ�ʱ�� - �� /
#define YAW_IMAGE_GIMBAL_DELAY_TIME			100e-3f//220e-3f//120e-3fs
#define PIT_IMAGE_GIMBAL_DELAY_TIME     100e-3f//220e-3f

#endif
