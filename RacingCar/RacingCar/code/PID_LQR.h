/*
 * PID_LQR.h
 *
 *  Created on: 2022年1月16日
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
    //直立环PD参数
    float zhili_p;
    float zhili_d;
    float zhili_i;
    //速度环PI参数
    float velo_p;
    float velo_i;
    float velo_d;
    float velo_speed;
    float motor_die;
    float ZHONGZHI;
    float ZHONGZHIK;//动态的中值比
    float yak;//动态的中值比
 };

struct pid_MW   //PID结构体
{
      float speedout;
      float angleout;
      float gyro_out;

      //速度环参数
      float speed_p;
      float speed_I;
      float speed_D;
      //角度环参数
      float angle_p;
      float angle_d;
      float angle_i;
      //角速度环参数
      float gyro_P;
      float gyro_i;
      float gyro_d;
      //平衡角
      float balance_A;
      float balancelime;
      float balancelk;
      float turn_increase;
      //转向
      float turn_pspeedk;
      float turn_dspeedk;
      float turn_p;     //转向外环P
      float turn_i;     //转向外环i
      float turn_ilimit;
      float turn_iaim;
      float turn_d;
      float yawan_p1;
      float yawan_p2;
      float yawan_aim1;
      float yawan_aim2;
      float yawan_max;
      float yawan_min;
      float turnspeed_p;     //转向外环P
      float turnspeed_i;     //转向外环i

      float turn_out;   //外环输出
      float turnspeed_out;   //外环输出
      float aimgyro;   //外环输出
};

struct pid_turn       //偏差改变平衡角转向PID
{
      float turn_p;     //转向外环P
      float turn_out;   //外环输出
      float turn_gyro_p;//角速度内环P
      float turn_gyro_d;//角速度内环D
      float final_out;  //转向环输出（改变的平衡角

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
