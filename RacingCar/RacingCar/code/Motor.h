/*
 * motor.h
 *
 *  Created on: 2024年3月9日
 *      Author: 线条君
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#define speed_standard_set_define
void Get_motor_stand(float pwm);
void motor_Init(void);
void motor_control(void);
void get_speed_standard_set (void);
void Speed_Keep(void);
void motor_IO_PWM_control(void);
void motor_protection(void);

extern float  speed1_err,speed2_err,speed1_err_pwm,speed2_err_pwm,speed_standard1,speed_forward,pwm_stand;
extern float speed_standard_set,motor_1_control,motor_2_control;//正负;

#endif /* CODE_MOTOR_H_ */
