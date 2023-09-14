#include "main.h"

int OutData[4]={0};//串口示波器 全局变量输出数据

RampGen_t qweRamp = RAMP_GEN_DAFAULT;  
float flag_flag=0;

int main(void)
{
 		BSP_Init();
		qweRamp.SetScale(&qweRamp, 500);
    qweRamp.ResetCounter(&qweRamp);
//		ddt_SetID(0x01);
		while(1)
		{
			ddt_SetMotor(200);
			
				OutData[0]=RC_CtrlData.rc.ch0;
				OutData[1]=RC_CtrlData.rc.ch1;
				OutData[2]=RC_CtrlData.rc.ch2;
				OutData[3]=RC_CtrlData.rc.ch3;
				OutPut_Data(OutData);

			if(RC_CtrlData.rc.s1==1)
			{
				qweRamp.SetScale(&qweRamp, 500);
				qweRamp.ResetCounter(&qweRamp);	
			}
			flag_flag=4000*qweRamp.Calc(&qweRamp);

		}
}


