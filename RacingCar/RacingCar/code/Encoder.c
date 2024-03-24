/*
 * Encoder.c
 *����ٶȻ�ȡ�Լ���ʵ�ٶȵļ���
 *  Created on: 2023��12��11��
 *      Author: LINEAR
 */
#include "Encoder.h"
#include "zf_driver_encoder.h"
#define ENCODER_L                 (TIM2_ENCODER)                  // ������������Ӧʹ�õı������ӿ�
#define ENCODER_L_A               (TIM2_ENCODER_CH1_P33_7)      // A ���Ӧ������
#define ENCODER_L_B               (TIM2_ENCODER_CH2_P33_6)     // B ���Ӧ������

#define ENCODER_R                 (TIM4_ENCODER)                  // ������������Ӧʹ�õı������ӿ�
#define ENCODER_R_A               (TIM4_ENCODER_CH1_P02_8)      // A ���Ӧ������
#define ENCODER_R_B               (TIM4_ENCODER_CH2_P00_9)     // B ���Ӧ������



void Encoder_Init(void)
{

    encoder_quad_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);   // ��ʼ��������ģ�������� �������������ģʽ
    encoder_quad_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);

}

float low_pass = 0.7;//��ͨ�˲�ϵ��
void Get_Encoder(Speed_DataTypedef *Q)
{

    register int Encoder_L=0,Encoder_R=0;
    Encoder_L =  encoder_get_count(ENCODER_L);     //��ǰ���Ǿ���һ��������Ϊ����һ��Ϊ��
    Encoder_R = -encoder_get_count(ENCODER_R);
    Q->Speed_L =  Encoder_L* low_pass + Q->Speed_L*(1-low_pass);
    Q->Speed_R = Encoder_R*low_pass + Q->Speed_R*(1-low_pass);
    Q->nowspeed = (Q->Speed_L+Q->Speed_R)/2;
    encoder_clear_count(ENCODER_L);
    encoder_clear_count(ENCODER_R);


}




