/*
 * Encoder.c
 *存放速度获取以及真实速度的计算
 *  Created on: 2023年12月11日
 *      Author: LINEAR
 */
#include "Encoder.h"
#include "zf_driver_encoder.h"
#define ENCODER_L                 (TIM2_ENCODER)                  // 正交编码器对应使用的编码器接口
#define ENCODER_L_A               (TIM2_ENCODER_CH1_P33_7)      // A 相对应的引脚
#define ENCODER_L_B               (TIM2_ENCODER_CH2_P33_6)     // B 相对应的引脚

#define ENCODER_R                 (TIM4_ENCODER)                  // 正交编码器对应使用的编码器接口
#define ENCODER_R_A               (TIM4_ENCODER_CH1_P02_8)      // A 相对应的引脚
#define ENCODER_R_B               (TIM4_ENCODER_CH2_P00_9)     // B 相对应的引脚



void Encoder_Init(void)
{

    encoder_quad_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);

}

float low_pass = 0.7;//低通滤波系数
void Get_Encoder(Speed_DataTypedef *Q)
{

    register int Encoder_L=0,Encoder_R=0;
    Encoder_L =  encoder_get_count(ENCODER_L);     //往前跑是就是一个编码器为正，一个为负
    Encoder_R = -encoder_get_count(ENCODER_R);
    Q->Speed_L =  Encoder_L* low_pass + Q->Speed_L*(1-low_pass);
    Q->Speed_R = Encoder_R*low_pass + Q->Speed_R*(1-low_pass);
    Q->nowspeed = (Q->Speed_L+Q->Speed_R)/2;
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);


}




