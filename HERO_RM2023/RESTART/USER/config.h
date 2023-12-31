#ifndef __SYS_H__
#define __SYS_H__

#define ROLL_MAX 14					//极限19°
#define ROLL_MIN -35				//极限48°
//云台初始位置
#define	GMPitchEncoder_Offset 0
#define	GMYawEncoder_Offset   58500
#define	CameraEncoder_Offset	1150
//安全阈值
#define Pitch_Safe_angle_max 12
#define Pitch_Safe_angle_min -32

#define REMOTE_SHOOT       1
        //0:遥控器左拨杆模式设置	0:小陀螺		1:发射	2:自瞄	3:搬障碍块

#define CAP_CTRL              				 1   //旧0新1
#define  POWER_LIMT                   1   //1时使用上交算法，0时使用直接限电流法

//相机焦距mm
#define FOCAL_LENGTH               6.0F //6.0F
//靶�  	娉m
#define TARGET_SURFACE_LENGTH      4.6e-3F
//靶面宽mm
#define TARGET_SURFACE_WIDTH        4.6e-3F
//像素尺寸mm
#define IMAGE_LENGTH                4.6e-3F

//摄像头和枪管中心的安装偏差角-YAW方向
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		0.0f
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA  0.0f

//摄像头和枪管中心的距离 
#define HEIGHT_BETWEEN_GUN_CAMERA 			9.2f//9.17f

//图像和云台控制延迟时间 - 秒 /
#define YAW_IMAGE_GIMBAL_DELAY_TIME			100e-3f//220e-3f//120e-3fs
#define PIT_IMAGE_GIMBAL_DELAY_TIME     100e-3f//220e-3f

#endif
