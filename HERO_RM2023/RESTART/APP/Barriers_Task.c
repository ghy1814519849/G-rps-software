#include "main.h"

barriers_mode_e barriers_mode = BARRIERS_STOP;
u8 barriers_zero_flag = 1;
u8 return_zero_flag = 0;
u8 stop_barriers_flag = 0;
float Remove_Barrier_set_angle[2]={0};
float Remove_Barrier_start_angle[2]={0};
float Remove_Barrier_stop_angle[2]={0};
float Remove_Barrier_end_angle[2]={0};

//搬障碍块任务
void barriers_task(void)
{
	switch(barriers_mode)
	{
		case BARRIERS_INIT:
		{
			if(barriers_zero_flag==1)
			{
				return_zero();
			}
		}
		break;
		case BARRIERS_RETURN:
		{
			Remove_Barrier_set_angle[0] = Remove_Barrier_start_angle[0];
			Remove_Barrier_set_angle[1] = Remove_Barrier_start_angle[1];
		}
		break;
		case BARRIERS_UP:
		{
			Remove_Barrier_set_angle[0]=Remove_Barrier_stop_angle[0];
			Remove_Barrier_set_angle[1]=Remove_Barrier_stop_angle[1];
		}
		break;
		case BARRIERS_DOWN:
		{
			Remove_Barrier_set_angle[0]=Remove_Barrier_end_angle[0];
			Remove_Barrier_set_angle[1]=Remove_Barrier_end_angle[1];
		}
		break;
		case BARRIERS_KEY_UP:
		{
			Remove_Barrier_set_angle[0]=Remove_Barrier_set_angle[0]+10;
			Remove_Barrier_set_angle[1]=Remove_Barrier_set_angle[1]-10;
		}
		break;
		case BARRIERS_KEY_DOWN:
		{
			Remove_Barrier_set_angle[0]=Remove_Barrier_set_angle[0]-10;
			Remove_Barrier_set_angle[1]=Remove_Barrier_set_angle[1]+10;
		}
		break;
		case BARRIERS_STOP:
		{
			Remove_Barrier_set_angle[0]=Remove_Barrier_set_angle[0];//BUS1_CM5Encoder.ecd_angle;
			Remove_Barrier_set_angle[1]=Remove_Barrier_set_angle[1];//BUS1_CM6Encoder.ecd_angle;
		}
		break;
	}
	if(return_zero_flag)
		barriers_moving_calc();
	
}

//实际上B1C5和B1C6是没有数据的，那他读的到底是啥
u8 zero_time=0;
void return_zero(void)//11350
{
//	pid_remove_barrier[0].get=BUS1_CM5_Poke_Encoder.ecd_angle;
//	pid_remove_barrier[0].set=-13000;
//	pid_calc(&pid_remove_barrier[0],pid_remove_barrier[0].get,pid_remove_barrier[0].set);
//	
//	pid_remove_barrier[1].get=BUS1_CM6Encoder.ecd_angle;
//	pid_remove_barrier[1].set=13000;
//	pid_calc(&pid_remove_barrier[1],pid_remove_barrier[1].get,pid_remove_barrier[1].set);
//	
//	pid_remove_barrier_speed[0].get=BUS1_CM5_Poke_Encoder.filter_rate;
//	pid_remove_barrier_speed[0].set=pid_remove_barrier[0].out;
//	pid_calc(&pid_remove_barrier_speed[0],pid_remove_barrier_speed[0].get ,pid_remove_barrier_speed[0].set );
//	
//	pid_remove_barrier_speed[1].get=BUS1_CM6Encoder.filter_rate;
//	pid_remove_barrier_speed[1].set=pid_remove_barrier[1].out;
//	pid_calc(&pid_remove_barrier_speed[1], pid_remove_barrier_speed[1].get, pid_remove_barrier_speed[1].set);
//	
	//如果速度过快，那么计数开始
	if((pid_remove_barrier_speed[0].out==-1200)&&(pid_remove_barrier_speed[1].out==1200))
		zero_time++;
	else
		zero_time=0;
	
	//如果计数为50，则 
	if(zero_time==50)
	{
		return_zero_flag=1;
		zero_time = 0;
		barriers_zero_flag=0;
		barriers_mode = BARRIERS_STOP;
//		Remove_Barrier_end_angle[0]=BUS1_CM5_Poke_Encoder.ecd_angle;
//		Remove_Barrier_end_angle[1]=BUS1_CM6Encoder.ecd_angle;
		Remove_Barrier_start_angle[0]=Remove_Barrier_end_angle[0]+barriers_ecd_angle;
		Remove_Barrier_start_angle[1]=Remove_Barrier_end_angle[1]-barriers_ecd_angle;
		Remove_Barrier_stop_angle[0]=Remove_Barrier_end_angle[0]+5000;
		Remove_Barrier_stop_angle[1]=Remove_Barrier_end_angle[1]-5000;
		Remove_Barrier_set_angle[0] = Remove_Barrier_start_angle[0];
		Remove_Barrier_set_angle[1] = Remove_Barrier_start_angle[1];
		pid_remove_barrier_speed[0].max_out = 7300;
		pid_remove_barrier_speed[1].max_out = 7300;
	}
}

//PID计算过程
void barriers_moving_calc(void)
{
	if(barriers_zero_flag==0)
	{
		//限制电压
		VAL_LIMIT(Remove_Barrier_set_angle[0],Remove_Barrier_end_angle[0]+50,Remove_Barrier_start_angle[0]);
		VAL_LIMIT(Remove_Barrier_set_angle[1],Remove_Barrier_start_angle[1],Remove_Barrier_end_angle[1]-50);
	}
//	pid_remove_barrier[0].get=BUS1_CM5_Poke_Encoder.ecd_angle;
//	pid_remove_barrier[0].set=Remove_Barrier_set_angle[0];
//	pid_calc(&pid_remove_barrier[0],pid_remove_barrier[0].get,pid_remove_barrier[0].set);
//	
//	pid_remove_barrier[1].get=BUS1_CM6Encoder.ecd_angle;
//	pid_remove_barrier[1].set=Remove_Barrier_set_angle[1];
//	pid_calc(&pid_remove_barrier[1],pid_remove_barrier[1].get,pid_remove_barrier[1].set);
//	
//	pid_remove_barrier_speed[0].get=BUS1_CM5_Poke_Encoder.filter_rate;
//	pid_remove_barrier_speed[0].set=pid_remove_barrier[0].out;
//	pid_calc(&pid_remove_barrier_speed[0],pid_remove_barrier_speed[0].get ,pid_remove_barrier_speed[0].set );
//	
//	pid_remove_barrier_speed[1].get=BUS1_CM6Encoder.filter_rate;
//	pid_remove_barrier_speed[1].set=pid_remove_barrier[1].out;
//	pid_calc(&pid_remove_barrier_speed[1], pid_remove_barrier_speed[1].get, pid_remove_barrier_speed[1].set);
}

//PID 初始化
void barriers_param_init(void)
{
	PID_struct_init(&pid_remove_barrier[0],POSITION_PID,100,50, 0, 0.2, 0 , 0 );
	PID_struct_init(&pid_remove_barrier[1],POSITION_PID,100,50, 0, 0.2, 0 , 0 );
		
	PID_struct_init(&pid_remove_barrier_speed[0],POSITION_PID,1200,500, 0, 120 , 0.02 ,0 );
	PID_struct_init(&pid_remove_barrier_speed[1],POSITION_PID,1200,500, 0, 120 , 0.02 ,0 );
}