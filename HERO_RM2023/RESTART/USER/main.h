#ifndef __MAIN_H__
#define __MAIN_H__

//STLIB
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_spi.h"
#include "pid.h"
#include "LADRC.h"

//USER
#include "config.h"

//BSP
#include "delay.h"
#include "common.h"
#include "bsp.h"
#include "timer.h"
#include "led.h"
#include "laser.h"
#include "can1.h"
#include "can2.h"
#include "usart.h"
#include "usart2.h"
#include "usart3.h"
#include "usart4.h"
#include "pwm.h"
#include "output2vs.h"
#include "client.h"
#include "TF02.h"
#include "ddt_motor.h"
#include "Camera_Gimbal_Task.h"

//auto_shoot
#include "protobuf-c.h"
//#include "protocol.pb-c.h"
#include "Send.pb-c.h"
#include "Recieve.pb-c.h"

//PROTOCAL
#include "protocal.h"

//CMSIS
#include "arm_math.h"
#include "math.h"

//RMLIB
#include "fifo.h"
#include "LostCounter.h"
#include "ramp.h"

//CRCLIB
#include "crcLib.h"

//TASK
#include "Control_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "Remote_Task.h"
#include "CanBusTask.h"
#include "ModeSwitch.h"
#include "Shoot_Task.h"
#include "AutoAngle_Shoot.h"
#include "AutoShoot_Control_Task.h"
#include "Supervise.h"
#include "Judge.h"
#include "Vehicle_Com_Task.h"
#include "Barriers_Task.h"

//CH100
//#include "ch100_DMA.h"
#include "hi220.h"
#endif 
