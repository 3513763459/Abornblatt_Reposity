/*
 * steer.c
 *
 *  Created on: 2024年3月5日
 *      Author: 线条君
 */

#include "Steer.h"
#include "PID.h"
#include "zf_driver_pwm.h"



#define SERVO_MOTOR_PWM           (ATOM1_CH1_P33_9)  // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ          (200 )            // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MID                 (2440)           // 50HZ中值610，左右变化70,右转数值变小
#define SERVO_DELTA               (230)           //100HZ 中值1220 左右变化130
float Steer_Control_PWM,Steer_Control_PWM_max=350,Steer_Control_err;

void Steer_Init(void)
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MID-SERVO_DELTA);//200HZ
   // pwm_init(ATOM0_CH7_P02_7, 50, 1000);   // ATOM 0模块的通道7 使用P02_7引脚输出PWM  PWM频率50HZ  占空比百分之1000/PWM_DUTY_MAX*100
}

void Steer_Control(float err)
{

  //  pwm_set_duty(SERVO_MOTOR_PWM, 2200);//中间
    //增加是左摆，建议浮动在400以内

       //航向角偏大就加大
    Steer_Control_err=err;
    Steer_Control_PWM=Steer_Control_PWM_PID(Steer_Control_err);

    if(Steer_Control_PWM<=Steer_Control_PWM_max&&Steer_Control_PWM>=-Steer_Control_PWM_max)
    {
        Steer_Control_PWM=Steer_Control_PWM+0;
    }
    else
    {
        if(Steer_Control_PWM>Steer_Control_PWM_max)
        {
            Steer_Control_PWM=Steer_Control_PWM_max;
        }

        if(Steer_Control_PWM<-Steer_Control_PWM_max)
        {
            Steer_Control_PWM=-Steer_Control_PWM_max;
        }
    }
   // constrain_float(Steer_Control_PWM,-Steer_Control_PWM_max,Steer_Control_PWM_max);//限幅

    pwm_set_duty(SERVO_MOTOR_PWM, 2200-Steer_Control_PWM);


}
