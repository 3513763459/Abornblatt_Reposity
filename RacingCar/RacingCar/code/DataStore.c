/*
 * DataStore.c
 *
 *  Created on: 2023��12��11��
 *      Author: LINEAR
 */
#include "DataStore.h"
#include "Camera.h"
#include "Element.h"
#include "System.h"

void Data_Settings(void)
{
    //����ͷ��������
    Image_DataStruct.Camera_Exp=100;
    Image_DataStruct.Diff_Ratio=0.3;//��Ⱥ���ֵ
    System_DataStruct.Element_Flag = ELEMENMT_NONE;
    //��·Ԫ�ر�־λ��ʼ��
    /*ʮ��*/
    Element0_DataStruct.CD=0;
    Element0_DataStruct.CD_flag=0;
    /*Element0_DataStruct.CD_adjacent=ELEMENMT_NONE;*/
    Element0_DataStruct.CU=0;
    Element0_DataStruct.CU_flag=0;
    Element0_DataStruct.CU_adjacent=ELEMENMT_NONE;
    /*��Բ��*/
    Element0_DataStruct.Round_Prev_L=0;
    Element0_DataStruct.Round_Prev_M=0;
    Element0_DataStruct.Round_Prev_U=0;
    //��·Ԫ�ر�־λ��ʼ��
    Element1_DataStruct.CD_flag=0;
    Element1_DataStruct.CD=0;
    /*Element1_DataStruct.CD_adjacent=ELEMENMT_NONE;*/
    Element1_DataStruct.CU=0;
    Element1_DataStruct.CU_flag=0;
    Element1_DataStruct.CU_adjacent=ELEMENMT_NONE;

    //ֱ�� ����յ�б���ж�
    Element_JustStruct.Straight_Slope=0.8;//0.8���ã�
    Element_JustStruct.Flat_Slope = 0.1;

}


