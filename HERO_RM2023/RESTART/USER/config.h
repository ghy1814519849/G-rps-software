#ifndef __SYS_H__
#define __SYS_H__

#define ROLL_MAX 14					//¼«ÏŞ19¡ã
#define ROLL_MIN -35				//¼«ÏŞ48¡ã
//ÔÆÌ¨³õÊ¼Î»ÖÃ
#define	GMPitchEncoder_Offset 0
#define	GMYawEncoder_Offset   58500
#define	CameraEncoder_Offset	1150
//°²È«ãĞÖµ
#define Pitch_Safe_angle_max 12
#define Pitch_Safe_angle_min -32

#define REMOTE_SHOOT       1
        //0:Ò£¿ØÆ÷×ó²¦¸ËÄ£Ê½ÉèÖÃ	0:Ğ¡ÍÓÂİ		1:·¢Éä	2:×ÔÃé	3:°áÕÏ°­¿é

#define CAP_CTRL              				 1   //¾É0ĞÂ1
#define  POWER_LIMT                   1   //1Ê±Ê¹ÓÃÉÏ½»Ëã·¨£¬0Ê±Ê¹ÓÃÖ±½ÓÏŞµçÁ÷·¨

//Ïà»ú½¹¾àmm
#define FOCAL_LENGTH               6.0F //6.0F
//°ĞÃ  	æ³¤mm
#define TARGET_SURFACE_LENGTH      4.6e-3F
//°ĞÃæ¿ímm
#define TARGET_SURFACE_WIDTH        4.6e-3F
//ÏñËØ³ß´çmm
#define IMAGE_LENGTH                4.6e-3F

//ÉãÏñÍ·ºÍÇ¹¹ÜÖĞĞÄµÄ°²×°Æ«²î½Ç-YAW·½Ïò
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		0.0f
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA  0.0f

//ÉãÏñÍ·ºÍÇ¹¹ÜÖĞĞÄµÄ¾àÀë 
#define HEIGHT_BETWEEN_GUN_CAMERA 			9.2f//9.17f

//Í¼ÏñºÍÔÆÌ¨¿ØÖÆÑÓ³ÙÊ±¼ä - Ãë /
#define YAW_IMAGE_GIMBAL_DELAY_TIME			100e-3f//220e-3f//120e-3fs
#define PIT_IMAGE_GIMBAL_DELAY_TIME     100e-3f//220e-3f

#endif
