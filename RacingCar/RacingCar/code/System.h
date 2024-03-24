/*
 * System.h
 *
 *  Created on: 2023��12��11��
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
  float gama;  /*΢�������˲�ϵ��*/

  float KP2;//���ʹ��
  float KD2;//���ʹ��
  float Steer_d;
} PID_Datatypedef; //�ṹ��Ҫ��SystemDatatypedef�б����ã�ֻ��������

typedef struct {
  float nowspeed;     // pulse��ʾnowspeed
  float Speed_L;      //���������¼���ٶ�
  float Speed_R;
  int MaxSpeed;        //����ٶ�
  int exceptspeed;    // speed��ʾexpcetspeed
  float Length;       //�߹�·��

  int straight_speed;       //ֱ���ٶ�
} Speed_DataTypedef;

/**
 * Element flag
 */

typedef struct
{
     uint8_t CD;
     bool CD_flag;//�¹յ��־λ1:�����ҵ��¹յ�

     uint8_t CU;//ʮ���Ϲյ�
     bool CU_flag;//�Ϲյ��־λ
     uint8_t CU_adjacent;//����ͼ����Ϣ����

     uint8_t Round_Prev_L;//��Բ��ǰ �¹յ�
     uint8_t Round_Prev_M;//��Բ��ǰ �йյ�
     uint8_t Round_Prev_U;//��Բ��ǰ �Ϲյ�
     uint8_t Round_Out_D;//��Բ���¹յ�
     uint8_t Round_Out_U;//��Բ���Ϲյ�
}Element_DataTypedef;

extern Element_DataTypedef Element0_DataStruct;//��·Ԫ��
extern Element_DataTypedef Element1_DataStruct;//��·Ԫ��
typedef struct
{
     Speed_DataTypedef SpeedDataStruct;
     uint8_t test_x;
     uint8_t test_y;
     uint8_t Element_Flag;//Ԫ�ر�־λ,Ŀ��:���ȷ����ǰλԲ��,���ڽ���ʮ���ж�
     uint8_t CIRCLE_Flag;//Բ��״̬��־λ
}System_DataTypedef;
extern System_DataTypedef System_DataStruct;

#endif /* CODE_SYSTEM_H_ */
