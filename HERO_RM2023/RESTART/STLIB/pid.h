/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file pid.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief pid parameter initialization, position and delta pid calculate
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
	
#ifndef __pid_H__
#define __pid_H__

#include "stm32f4xx.h"

enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;
	
  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
											 float      output_deadband,

                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} pid_t;


#if 0
#define PID_PARAM_DEFAULT \
{\
  0,\
  0,\
  0,\
  0,\
  0,\
  {0,0,0},\
  0,\
  0,\
  0,\
  0,\
  0,\
  0,\
}\

typedef struct
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3]; //error

  float pout; 
  float iout; 
  float dout; 
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 

  float p_far;
  float p_near;
  float grade_range;
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} grade_pid_t;
#endif

void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
		float    output_deadband,
    float kp,
    float ki,
    float kd				);

float pid_calc(pid_t *pid, float fdb, float ref);
float pid_calc1(pid_t *pid, float get, float set);
void  pid_clr(pid_t *pid);
		
extern pid_t pid_pit_follow;
extern pid_t pid_yaw_follow;
extern pid_t pid_pit_speed_follow;
extern pid_t pid_yaw_speed_follow;
	
extern pid_t pid_yaw_station;
extern pid_t pid_pit_station;
extern pid_t pid_pit_speed_station;
extern pid_t pid_yaw_speed_station;
	
extern pid_t pid_yaw_auto_angle;
extern pid_t pid_pit_auto_angle;
extern pid_t pid_pit_speed_auto_angle;
extern pid_t pid_yaw_speed_auto_angle;
	
	
extern pid_t pid_yaw_follow_chassis_angle ;
extern pid_t pid_yaw_follow_chassis_speed ;
		
extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_speed_safe;
extern pid_t pid_pit_safe;
extern pid_t pid_pit_speed;
extern pid_t pid_yaw_speed;
extern pid_t pid_auto_aim_pit;
extern pid_t pid_auto_aim_yaw;
extern pid_t pid_voltage;
extern pid_t pid_software_limit;
extern pid_t pid_spd[4];


extern pid_t pid_chassis_angle;
extern pid_t pid_trigger;
extern pid_t pid_trigger_speed;
extern pid_t pid_imu_tmp;
extern pid_t pid_rotate[4];
extern pid_t pid_speed_bias;
extern pid_t pid_front_distance;
extern pid_t pid_right_distance;
extern pid_t pid_angle_distance;
extern pid_t pid_spring[2];
extern pid_t pid_shoot_bullet_position_speed_loop;  //用于连续射击前，下拨盘拨动一个角度的pid速度环控制器
extern pid_t pid_shoot_bullet_position_angle_loop_2;  //用于连续射击前，拨动一个角度的pid角度环控制器二级拨盘
extern pid_t pid_shoot_bullet_position_speed_loop_2;   //用于连续射击前，拨动一个角度的pid速度环控制器

extern pid_t pid_remove_barrier[2];
extern pid_t pid_remove_barrier_speed[2];
extern pid_t pid_shoot_current[3];
#endif
