/*
 * steer.c
 *
 *  Created on: 2024��3��5��
 *      Author: ������
 */

#include "Steer.h"
#include "PID.h"
#include "zf_driver_pwm.h"



#define SERVO_MOTOR_PWM           (ATOM1_CH1_P33_9)  // ���������϶����Ӧ����
#define SERVO_MOTOR_FREQ          (200 )            // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300
#define SERVO_MID                 (2440)           // 50HZ��ֵ610�����ұ仯70,��ת��ֵ��С
#define SERVO_DELTA               (230)           //100HZ ��ֵ1220 ���ұ仯130
float Steer_Control_PWM,Steer_Control_PWM_max=350,Steer_Control_err;

void Steer_Init(void)
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MID-SERVO_DELTA);//200HZ
   // pwm_init(ATOM0_CH7_P02_7, 50, 1000);   // ATOM 0ģ���ͨ��7 ʹ��P02_7�������PWM  PWMƵ��50HZ  ռ�ձȰٷ�֮1000/PWM_DUTY_MAX*100
}

void Steer_Control(float err)
{

  //  pwm_set_duty(SERVO_MOTOR_PWM, 2200);//�м�
    //��������ڣ����鸡����400����

       //�����ƫ��ͼӴ�
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
   // constrain_float(Steer_Control_PWM,-Steer_Control_PWM_max,Steer_Control_PWM_max);//�޷�

    pwm_set_duty(SERVO_MOTOR_PWM, 2200-Steer_Control_PWM);


}
