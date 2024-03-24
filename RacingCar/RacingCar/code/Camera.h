/*
 * Camera.h
 *
 *  Created on: 2023年12月9日
 *      Author: LINEAR
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_
#include "zf_device_mt9v03x.h"
#define     POINTS_MAX_LEN          120
#define     TC_IMAGE(img,x,y)       ((img)->data[(y)*(img)->step+(x)])
//定义一个图像，用于后续连续定义图像数组
#define     DEF_IMAGE(ptr,w,h)      {.data=ptr, .width=w, .height=h, .step=w}
#define    BlackBoundry    2
typedef struct image {
    uint8 *data;
    uint16_t width;
    uint16_t height;
    uint16_t step;
} image_t;
extern image_t img_raw;


typedef struct
{
float Camera_Error;//摄像头处理得到的偏差
uint8_t begin_err;//代表从中线数组哪一行开始计算偏差
uint16_t Camera_Exp;      //曝光时间,时间越短,图像越暗
uint8_t Diff_Threshold;//摄像头处理阈值
uint8_t begin_x;
float Diff_Ratio;//摄像头处理差比和阈值 有正有负-1~1之间
}Image_DataTypedef;//这里将存储图像处理用到的需要根据场地调整的变量?
extern Image_DataTypedef Image_DataStruct;

extern uint8 Gray_Image[MT9V03X_H][MT9V03X_W];//把摄像头传进的图像转移到这里,对这张图像做处理
// 原图左右边线
extern uint8_t ipts0[POINTS_MAX_LEN][2];
extern uint8_t ipts1[POINTS_MAX_LEN][2];
extern uint8_t ipts0_num, ipts1_num;
extern float block_size;
extern float clip_value;
//拟合的中线
extern float  rpts[MT9V03X_H][2];
extern uint8 rpts_num;
//图像处理主函数
void ImageProcess(image_t * image);
void Find_Line(image_t * image);
void Image_Black(uint8_t Width,uint8_t Height,uint8_t *p);
//迷宫寻线,局部二值化
void findline_lefthand_adaptive(image_t *img, float block_size, float clip_value, uint8 x, uint8 y, uint8 pts[][2], uint8 *num);
void findline_righthand_adaptive(image_t *img, float block_size, float clip_value, uint8 x, uint8 y, uint8 pts[][2], uint8 *num);

//拟合中线
void Fit_Mid_Line(uint8 ipts0[][2],uint8 *num0,uint8 ipts1[][2],uint8 *num1,float (*Mid)[2],uint8 *Mid_num);

//获取图像偏差
float Get_Error(uint8 width,uint8 begin_err,float (*rpts)[2],uint8 *Mid_num);

#endif /* CODE_CAMERA_H_ */
