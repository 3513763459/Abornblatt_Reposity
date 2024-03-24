/*
 * motor.c
 *
 *  Created on: 2024年3月9日
 *      Author: 线条君
 */

#include "PID.h"
#include "Camera.h"
#include "forward.h"
#include "PID_LQR.h"
#include "Motor.h"

#define array_burst_max    (5000)
#define speed_standard_min (2.5)
#define Motor_dead_zone      (500)//电机死区
#define protection_set        (60)

float speed_forward=0;//目标值2和-2
float pwm_stand,speed1_err_pwm,speed2_err_pwm,speed1_err,speed2_err,speed_standard1,speed_standard2;
float speed_standard_set=3;//3即可上坡
float motor_1_control,motor_2_control;//正负
float motor_P02_7,motor_P02_5,motor_P02_6,motor_P02_4;
/*
 * 前进速度误差控制函数
 * 操控目的：误差在大于30和-30时将速度控制在-2和2以内
 * */
void get_speed_standard_set (void)
{
    speed_standard1=speed_standard_set-forward_speed_control_PID(fabs(Image_DataStruct.Camera_Error))/10;//误差绝对值
    if(speed_standard1<speed_standard_min)speed_standard1=speed_standard_min;//锁住最小值
    speed_standard2=-speed_standard1;

}

void motor_IO_PWM_control(void)
{

              motor_1_control=motor_P02_6-motor_P02_7;
              motor_2_control=motor_P02_5-motor_P02_4;

              if(motor_1_control>=9500)motor_1_control=9500;
              if(motor_2_control>=9500)motor_2_control=9500;

              if(motor_1_control>0)
              {
                  pwm_set_duty(ATOM1_CH7_P02_7, 0);
                  pwm_set_duty(ATOM0_CH6_P02_6, motor_1_control);
              }
              else
              {
                  pwm_set_duty(ATOM1_CH7_P02_7, -motor_1_control);
                  pwm_set_duty(ATOM0_CH6_P02_6, 0);

              }

              if(motor_2_control>0)
              {
                  pwm_set_duty(ATOM1_CH5_P02_5, motor_2_control);
                  pwm_set_duty(ATOM0_CH4_P02_4, 0);
              }

              motor_protection();



}


void motor_control(void)
{

    if(pwm_stand>0)
      {
      motor_P02_7=Motor_dead_zone+pwm_stand+turn_err_PWM;
      motor_P02_5=Motor_dead_zone+pwm_stand+speed1_err_pwm+turn_err_PWM;
      motor_P02_6=speed1_err_pwm;
      motor_P02_4=0;

      motor_IO_PWM_control();

      }
      else
      {
      motor_P02_7=0;
      motor_P02_5=speed1_err_pwm;
      motor_P02_6=Motor_dead_zone-pwm_stand+speed1_err_pwm-turn_err_PWM;
      motor_P02_4=Motor_dead_zone-pwm_stand-turn_err_PWM;

      motor_IO_PWM_control();
      }

}

void Speed_Keep()
{
    get_speed_standard_set ();
    speed1_err=speed_standard1-(speed1-(speed1+speed2)/2);//(speed1+speed2)/2减去编码器横向值(T)
    speed2_err=-speed_standard2+(speed2-(speed1+speed2)/2);//大了输出正值

    speed1_err_pwm=speed_keep_PID(speed1_err);
    speed2_err_pwm=speed_keep_PID(speed2_err);
    if((speed1_err_pwm>array_burst_max&&speed2_err_pwm>array_burst_max)||(speed1_err_pwm<-array_burst_max&&speed2_err_pwm<-array_burst_max))//防止爆数组
    {
        //防止爆数组，重置
        forward_speed_Init(&forward_speed_pid);

    }


}

void Get_motor_stand(float pwm)//右转是负数（摄像头）
{
    pwm_stand=pwm;
}

void motor_protection(void)
{
    if(imu.pitch>protection_set||imu.pitch<-protection_set)
    {
    pwm_set_duty(ATOM1_CH7_P02_7, 0);
    pwm_set_duty(ATOM1_CH5_P02_5, 0);
    pwm_set_duty(ATOM0_CH6_P02_6,0);
    pwm_set_duty(ATOM0_CH4_P02_4, 0);
    }
}

void motor_Init(void)
{
    pwm_init(ATOM1_CH7_P02_7, 17000,0);//向前走向右speed1负67一个电机
    pwm_init(ATOM1_CH5_P02_5, 17000,0);//向后走向右speed2负
    pwm_init(ATOM0_CH6_P02_6, 17000,0);//向后走向左speed1正
    pwm_init(ATOM0_CH4_P02_4, 17000,0);//向前走向左speed2正
    //1,2 2,-2 1500编码器-3

}

