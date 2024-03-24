/*
 * PID.h
 *
 *  Created on: 2023年12月21日
 *      Author: LINEAR
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"
#include "System.h"

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //绉�����骞�

    float                out_p;
    float                out_i;
    float                out_d;
    float                out;

    float                integrator;
    float                last_error;
    float                last_derivative;
    unsigned long        last_t;
}pid_param;


extern  pid_param forward_steer_pid;
extern  pid_param forward_mid_pid;
extern  pid_param forward_turn_pid;
extern  pid_param forward_speed_pid;

extern float forward_steer_P,forward_steer_D,forward_steer_I;
extern float forward_mid_P,forward_mid_D,forward_mid_I;

extern int Start_Flag;
float   constrain_float                        (float amt, float low, float high)      ;

float   PidLocCtrl                             (pid_param * pid, float error)          ;

float   Steer_Control_PWM_PID                  (float Steer_Control_PWM_PID_err)       ;
float   LCC_PID                                (float LCC_PID_err)                     ;
float   LCC_turn_PID                           (float LCC_turn_PID_err)                ;
float   speed_keep_PID                         (float speed_keep_err)                  ;
float   forward_speed_control_PID              (float forward_speed_control_err)       ;

void forward_steer_Init(pid_param * pid);
void forward_mid_Init(pid_param * pid);
void forward_turn_Init(pid_param * pid);
void forward_speed_Init(pid_param * pid);
void forward_speed_control_Init(pid_param * pid);

void forward_Init(void);

#endif
