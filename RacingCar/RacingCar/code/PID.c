/*
 * PID.c
 *
 *  Created on: 2023年12月21日
 *      Author: LINEAR
 */


#include "PID.h"
#include "isr.h"


pid_param forward_steer_pid;
pid_param forward_mid_pid;
pid_param forward_turn_pid;
pid_param forward_speed_pid;
pid_param forward_speed_control_pid;

//------------------------------------------------------------------------------------------------------------------
int Start_Flag=0;

float forward_steer_err=0;


float forward_steer_P            =6,forward_steer_D             =7,   forward_steer_I                    =0.005;
float forward_mid_P              =0.060,forward_mid_D           =0.050 ,forward_mid_I                    =0.00005;//0.00005
float forward_turn_P             =0,forward_turn_D              =0,forward_turn_I                        =0;//20.20
float forward_speed_P            =50,forward_speed_D            =0,forward_speed_I                       =0.5;
float forward_speed_control_P    =0.3,forward_speed_control_D   =0.6,forward_speed_control_I               =0;

//-------------------------------------------------------------------------------------------------------------------
void forward_steer_Init(pid_param * pid);
void forward_mid_Init(pid_param * pid);
void forward_turn_Init(pid_param * pid);
void forward_speed_Init(pid_param * pid);
void forward_speed_control_Init(pid_param * pid);

float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


void forward_Init(void)
{

    forward_steer_Init(&forward_steer_pid);
    forward_mid_Init(&forward_mid_pid);
    forward_turn_Init(&forward_turn_pid);
    forward_speed_Init(&forward_speed_pid);
    forward_speed_control_Init(&forward_speed_control_pid);

}

void forward_steer_Init(pid_param * pid)
{
    pid->kp        = forward_steer_P;
    pid->ki        = forward_steer_I;
    pid->kd        = forward_steer_D;

    pid->imax      = 0.1;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}
void forward_mid_Init(pid_param * pid)
{
    pid->kp        = forward_mid_P;
    pid->ki        = forward_mid_I;
    pid->kd        = forward_mid_D;

    pid->imax      =0.000675;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}
void forward_turn_Init(pid_param * pid)
{
    pid->kp        = forward_turn_P;
    pid->ki        = forward_turn_I;
    pid->kd        = forward_turn_D;

    pid->imax      =0.001;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

void forward_speed_Init(pid_param * pid)
{
    pid->kp        = forward_speed_P;
    pid->ki        = forward_speed_I;
    pid->kd        = forward_speed_D;

    pid->imax      =0.05;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}
void forward_speed_control_Init(pid_param * pid)
{
    pid->kp        = forward_speed_control_P;
    pid->ki        = forward_speed_control_I;
    pid->kd        = forward_speed_control_D;

    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}
//-----------------------------------------------------------------
float Steer_Control_PWM_PID(float Steer_Control_PWM_PID_err)
{
    PidLocCtrl(&forward_steer_pid,Steer_Control_PWM_PID_err);
    return forward_steer_pid.out;
}
float LCC_PID(float LCC_PID_err)
{
    PidLocCtrl(&forward_mid_pid,LCC_PID_err);
    return forward_mid_pid.out;
}
float LCC_turn_PID(float LCC_turn_PID_err)
{

    PidLocCtrl(&forward_turn_pid,LCC_turn_PID_err);
    return forward_turn_pid.out;
}
float speed_keep_PID(float speed_keep_err)
{
    PidLocCtrl(&forward_speed_pid,speed_keep_err);
    return forward_speed_pid.out;
}
float forward_speed_control_PID(float forward_speed_control_err)//速度偏差控制，弯道过大将速控制再2.0
{
    PidLocCtrl(&forward_speed_control_pid,forward_speed_control_err);
    return forward_speed_control_pid.out;

}
//-------------------------------------------------------------
float PidLocCtrl(pid_param * pid, float error)
{
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
 // if(Start_Flag==0) pid->integrator=0,error=0,pid->last_error=0,pid->out=0,pid->last_t=0,pid->last_derivative =0, Start_Flag=1;
    return pid->out;
}







