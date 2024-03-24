/*
 * steer.h
 *
 *  Created on: 2024年3月5日
 *      Author: 线条君
 */

#ifndef CODE_STEER_H_
#define CODE_STEER_H_
#include "zf_common_headfile.h"
void Steer_Init(void);
void Steer_Control(float err);

extern float Steer_Control_PWM,Steer_Control_err;


#endif /* CODE_STEER_H_ */
