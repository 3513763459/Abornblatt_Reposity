/*
 * Function.h
 *
 *  Created on: 2023年12月9日
 *      Author: LINEAR
 */

#ifndef CODE_FUNCTION_H_
#define CODE_FUNCTION_H_
#include "stdint.h"
#define min(a,b) (a<b?:a,b)
#define max(a,b) (a>b?:a,b)
#define Diff_Ratio(a,b)    (((float)a-(float)b)/((float)a+(float)b))
#define PI                  ( 3.1415926535898 )
void copyArrays(uint8_t(*A)[2],uint8_t(*B)[2],uint8_t Start_A, uint8_t End_A,uint8_t Start_B);
void Patch_Line_Points(uint8_t D,uint8_t U,uint8_t (*ipts)[2]);
float Slope_2Points(int begin,uint8_t end,uint8_t (*ipts)[2]);
float Slope_Calculate(int begin,uint8_t end,uint8_t (*p)[2]);
void OpenCV_DrawLine(uint8_t (* ipts)[2],uint8_t * ipts_num,uint8_t blue,uint8_t green,uint8_t red);
void Intercept_Calculate(uint8_t D,uint8_t (*ipts)[2],float *slope,float *intercept);
float process_curvity(uint8_t (* ipts)[2],uint8_t m,uint8_t separate);//三点曲率计算
#endif /* CODE_FUNCTION_H_ */
