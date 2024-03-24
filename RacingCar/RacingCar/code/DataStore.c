/*
 * DataStore.c
 *
 *  Created on: 2023年12月11日
 *      Author: LINEAR
 */
#include "DataStore.h"
#include "Camera.h"
#include "Element.h"
#include "System.h"

void Data_Settings(void)
{
    //摄像头参数设置
    Image_DataStruct.Camera_Exp=100;
    Image_DataStruct.Diff_Ratio=0.3;//差比和阈值
    System_DataStruct.Element_Flag = ELEMENMT_NONE;
    //左路元素标志位初始化
    /*十字*/
    Element0_DataStruct.CD=0;
    Element0_DataStruct.CD_flag=0;
    /*Element0_DataStruct.CD_adjacent=ELEMENMT_NONE;*/
    Element0_DataStruct.CU=0;
    Element0_DataStruct.CU_flag=0;
    Element0_DataStruct.CU_adjacent=ELEMENMT_NONE;
    /*左圆环*/
    Element0_DataStruct.Round_Prev_L=0;
    Element0_DataStruct.Round_Prev_M=0;
    Element0_DataStruct.Round_Prev_U=0;
    //右路元素标志位初始化
    Element1_DataStruct.CD_flag=0;
    Element1_DataStruct.CD=0;
    /*Element1_DataStruct.CD_adjacent=ELEMENMT_NONE;*/
    Element1_DataStruct.CU=0;
    Element1_DataStruct.CU_flag=0;
    Element1_DataStruct.CU_adjacent=ELEMENMT_NONE;

    //直道 弯道拐点斜率判定
    Element_JustStruct.Straight_Slope=0.8;//0.8更好？
    Element_JustStruct.Flat_Slope = 0.1;

}


