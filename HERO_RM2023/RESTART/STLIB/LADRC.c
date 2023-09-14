#include "main.h"


LADRC_NUM ladrc_num,ladrc_angle;

void LADRC_Init(void)
{
	ladrc_num.b0 = 2;
	ladrc_num.h = 0.003;
	ladrc_num.wc = 600;//600
	ladrc_num.w0 = 6;//6
	ladrc_num.r = 20;
	
	ladrc_angle.b0 = 2;
	ladrc_angle.h = 0.003;
	ladrc_angle.wc = 600;//600
	ladrc_angle.w0 = 6;//6
	ladrc_angle.r = 20;
	
	
}

void LADRC_extend_observer(LADRC_NUM* v)
{
//	ladrc_num.beta1 = 3*ladrc_num.w0;
//	ladrc_num.beta2 = 3*ladrc_num.w0*ladrc_num.w0;
//	ladrc_num.beta3 = ladrc_num.w0*ladrc_num.w0*ladrc_num.w0;
//	
//	ladrc_num.xhat1+= (ladrc_num.xhat2 + ladrc_num.beta1*(ladrc_num.y - ladrc_num.xhat1))*ladrc_num.h;
//	ladrc_num.xhat2+= (ladrc_num.xhat3 + ladrc_num.b0*ladrc_num.u + ladrc_num.beta2*(ladrc_num.y - ladrc_num.xhat1))*ladrc_num.h;
//	ladrc_num.xhat3+= (ladrc_num.beta3*(ladrc_num.y - ladrc_num.xhat1))*ladrc_num.h;
	
	v->beta1 = 2*v->w0;
	v->beta2 = v->w0*v->w0;
	
	v->xhat1+= (v->xhat2 + v->b0*v->u + v->beta1*(v->y - v->xhat1))*v->h;
	v->xhat2+= (v->beta2*(v->y - v->xhat1))*v->h;
}

void LADRC_pdcontroler(LADRC_NUM* v)
{
//	ladrc_num.kp = ladrc_num.wc*ladrc_num.wc;
//	ladrc_num.kd = 2*ladrc_num.wc;
//	
//	float e1 = ladrc_num.v1-ladrc_num.xhat1;
//	float e2 = ladrc_num.v2-ladrc_num.xhat2;
//	
//	ladrc_num.u0 = e1*ladrc_num.kp - ladrc_num.kd*e2;
//	ladrc_num.u = (ladrc_num.u0 - ladrc_num.xhat3)/ladrc_num.b0;
//	if(ladrc_num.u>2000)
//		ladrc_num.u=2000;
//	else if(ladrc_num.u<-2000)
//		ladrc_num.u=-2000;
	
	v->kp = v->wc;
//	v->kd = 2*v->wc;
	
	float e1 = v->ref-v->xhat1;
//	float e2 = v->v2-v->xhat2;
	
	v->u0 = e1*v->kp;
	v->u = (v->u0 - v->xhat2)/v->b0;
	if(v->u>10000)
		v->u=10000;
	else if(v->u<-10000)
		v->u=-10000;
}

void LADRC_TD(void)
{
	float fh= -ladrc_num.r*ladrc_num.r*(ladrc_num.v1-ladrc_num.ref)-2*ladrc_num.r*ladrc_num.v2;
	ladrc_num.v1+=ladrc_num.v2*ladrc_num.h;
	ladrc_num.v2+=fh*ladrc_num.h;
}

float LADRC_control_task(LADRC_NUM* v,float ref,float feedback)
{
	v->ref = ref;
	v->y = feedback;
	
	
//	LADRC_TD();
	LADRC_extend_observer(v);
	LADRC_pdcontroler(v);
	
	return v->u;
}


