/*
 * forward.c
 *
 *  Created on: 2024年3月3日
 *      Author: 线条君
 */
#include "forward.h"
#include "PID.h"
#include "PID_LQR.h"


#define turn_LCC_err_control    (15)
#define turn_err_PWM_MAX            (600)
float balance_A_err,turn_err_PWM;
float LCC_MAX=4.5;

float LCC(float err)
{
    balance_A_err=LCC_PID(err);

    if(balance_A_err<=LCC_MAX&&balance_A_err>=-LCC_MAX)
    {
        balance_A_err=balance_A_err+0;
    }
    else
    {
        if(balance_A_err>LCC_MAX)
        {
            balance_A_err=LCC_MAX;
        }

        if(balance_A_err<-LCC_MAX)
        {
            balance_A_err=-LCC_MAX;
        }
    }

   // balance_A_err=(angle_0-err)*(angle_0-err)*(angle_0-err)/8100/90*5;
     return -balance_A_err;

}
void  turn_LCC(float err)
{
    //限制在大于turn_LCC_err_control启动
    if(err>=turn_LCC_err_control||err<=-turn_LCC_err_control)
    {
        if(err>0)
        {
            err=err-turn_LCC_err_control;
        }
        else err=err+turn_LCC_err_control;//负数


        turn_err_PWM=LCC_turn_PID(err);
        //限幅
        if(turn_err_PWM<=turn_err_PWM_MAX&&turn_err_PWM>=-turn_err_PWM_MAX)
        {
            turn_err_PWM=turn_err_PWM+0;
        }
        else
        {
            if(turn_err_PWM>turn_err_PWM_MAX)
            {
                turn_err_PWM=turn_err_PWM_MAX;
            }

            if(turn_err_PWM<-turn_err_PWM_MAX)
            {
                turn_err_PWM=-turn_err_PWM_MAX;
            }
        }

    }
    else turn_err_PWM=0;
    //获取err




}
