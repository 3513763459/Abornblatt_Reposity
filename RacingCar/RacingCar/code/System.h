/*
 * System.h
 *
 *  Created on: 2023年12月11日
 *      Author: LINEAR
 */

#ifndef CODE_SYSTEM_H_
#define CODE_SYSTEM_H_
#include "stdbool.h"
#include "stdint.h"

typedef struct{
  float P;
  float I;
  float D;
  float gama;  /*微分先行滤波系数*/

  float KP2;//舵机使用
  float KD2;//舵机使用
  float Steer_d;
} PID_Datatypedef; //结构体要在SystemDatatypedef中被引用，只能在这里

typedef struct {
  float nowspeed;     // pulse表示nowspeed
  float Speed_L;      //左编码器记录的速度
  float Speed_R;
  int MaxSpeed;        //最大速度
  int exceptspeed;    // speed表示expcetspeed
  float Length;       //走过路程

  int straight_speed;       //直道速度
} Speed_DataTypedef;

/**
 * Element flag
 */

typedef struct
{
     uint8_t CD;
     bool CD_flag;//下拐点标志位1:代表找到下拐点

     uint8_t CU;//十字上拐点
     bool CU_flag;//上拐点标志位
     uint8_t CU_adjacent;//相邻图像信息传递

     uint8_t Round_Prev_L;//入圆环前 下拐点
     uint8_t Round_Prev_M;//入圆环前 中拐点
     uint8_t Round_Prev_U;//入圆环前 上拐点
     uint8_t Round_Out_D;//出圆环下拐点
     uint8_t Round_Out_U;//出圆环上拐点
}Element_DataTypedef;

extern Element_DataTypedef Element0_DataStruct;//左路元素
extern Element_DataTypedef Element1_DataStruct;//右路元素
typedef struct
{
     Speed_DataTypedef SpeedDataStruct;
     uint8_t test_x;
     uint8_t test_y;
     uint8_t Element_Flag;//元素标志位,目标:如果确定当前位圆环,不在进行十字判定
     uint8_t CIRCLE_Flag;//圆环状态标志位
}System_DataTypedef;
extern System_DataTypedef System_DataStruct;

#endif /* CODE_SYSTEM_H_ */
