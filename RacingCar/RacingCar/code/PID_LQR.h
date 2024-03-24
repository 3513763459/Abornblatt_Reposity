/*
 * PID_LQR.h
 *
 *  Created on: 2022��1��16��
 *      Author: yi deng
 */

#ifndef CODE_PID_LQR_H_
#define CODE_PID_LQR_H_
#include "Function.h"
#define range(x, a, b)   (min(max(x, a), b))

 extern float speed1,speed2,speed3,speedk,startflag,speed_enco;//
 extern int A,B;
 extern float PWM;
struct pid_motor
{

    float zhili_out;
    float velo_out;
    //ֱ����PD����
    float zhili_p;
    float zhili_d;
    float zhili_i;
    //�ٶȻ�PI����
    float velo_p;
    float velo_i;
    float velo_d;
    float velo_speed;
    float motor_die;
    float ZHONGZHI;
    float ZHONGZHIK;//��̬����ֵ��
    float yak;//��̬����ֵ��
 };

struct pid_MW   //PID�ṹ��
{
      float speedout;
      float angleout;
      float gyro_out;

      //�ٶȻ�����
      float speed_p;
      float speed_I;
      float speed_D;
      //�ǶȻ�����
      float angle_p;
      float angle_d;
      float angle_i;
      //���ٶȻ�����
      float gyro_P;
      float gyro_i;
      float gyro_d;
      //ƽ���
      float balance_A;
      float balancelime;
      float balancelk;
      float turn_increase;
      //ת��
      float turn_pspeedk;
      float turn_dspeedk;
      float turn_p;     //ת���⻷P
      float turn_i;     //ת���⻷i
      float turn_ilimit;
      float turn_iaim;
      float turn_d;
      float yawan_p1;
      float yawan_p2;
      float yawan_aim1;
      float yawan_aim2;
      float yawan_max;
      float yawan_min;
      float turnspeed_p;     //ת���⻷P
      float turnspeed_i;     //ת���⻷i

      float turn_out;   //�⻷���
      float turnspeed_out;   //�⻷���
      float aimgyro;   //�⻷���
};

struct pid_turn       //ƫ��ı�ƽ���ת��PID
{
      float turn_p;     //ת���⻷P
      float turn_out;   //�⻷���
      float turn_gyro_p;//���ٶ��ڻ�P
      float turn_gyro_d;//���ٶ��ڻ�D
      float final_out;  //ת��������ı��ƽ���

};

extern struct pid_motor pid1;
extern struct pid_MW pid2;

void pid2_speed(void);
void pid2_angle(void);
void pid2_gyro(void);
void Turn_loop(void);
void Turnspeed_loop(void);

//void moto_stop();

void pidget(void);
#endif /* CODE_PID_LQR_H_ */
