#ifndef __LADRC_H
#define __LADRC_H
#include "main.h"



typedef struct 
{
  float h;
	float xhat1;
	float xhat2;
	float xhat3;
	float b0;
	float wc;
	float y;
	float u;
	float u0;
	float w0;
	float beta1;
	float beta2;
	float beta3;
	float ref;
	float kp;
	float kd;
	float r;
	float v1;
	float v2;
	
}LADRC_NUM;


void LADRC_extend_observer(LADRC_NUM* v);
void LADRC_pdcontroler(LADRC_NUM* v);
float LADRC_control_task(LADRC_NUM* v,float ref,float feedback);
void LADRC_Init(void);
void LADRC_TD(void);

extern LADRC_NUM ladrc_num;
extern LADRC_NUM ladrc_angle;







#endif


