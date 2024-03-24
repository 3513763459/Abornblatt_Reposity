/*
 * Car.h
 *
 *  Created on: 2023��12��9��
 *      Author: LINEAR
 */

#ifndef CODE_CAR_H_
#define CODE_CAR_H_
#include "zf_device_mt9v03x.h"
#define STEER_PERIOD_MS (5) //���,�������ж�
#define SPEED_PERIOD_MS (20) //����ж�
#define GYRO_PERIOD_MS  (2)//�������ж�

void Cpu1_Init(void);
void Cpu0_Init(void);
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
extern uint8 x0_boundry[MT9V03X_H],x1_boundry[MT9V03X_H],y0_boundry[MT9V03X_H],y1_boundry[MT9V03X_H];
extern float rptsx_boundry[MT9V03X_H],rptsy_boundry[MT9V03X_H];

#endif /* CODE_CAR_H_ */
