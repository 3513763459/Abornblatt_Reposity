/*
 * Camera.h
 *
 *  Created on: 2023��12��9��
 *      Author: LINEAR
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_
#include "zf_device_mt9v03x.h"
#define     POINTS_MAX_LEN          120
#define     TC_IMAGE(img,x,y)       ((img)->data[(y)*(img)->step+(x)])
//����һ��ͼ�����ں�����������ͼ������
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
float Camera_Error;//����ͷ����õ���ƫ��
uint8_t begin_err;//���������������һ�п�ʼ����ƫ��
uint16_t Camera_Exp;      //�ع�ʱ��,ʱ��Խ��,ͼ��Խ��
uint8_t Diff_Threshold;//����ͷ������ֵ
uint8_t begin_x;
float Diff_Ratio;//����ͷ�����Ⱥ���ֵ �����и�-1~1֮��
}Image_DataTypedef;//���ｫ�洢ͼ�����õ�����Ҫ���ݳ��ص����ı���?
extern Image_DataTypedef Image_DataStruct;

extern uint8 Gray_Image[MT9V03X_H][MT9V03X_W];//������ͷ������ͼ��ת�Ƶ�����,������ͼ��������
// ԭͼ���ұ���
extern uint8_t ipts0[POINTS_MAX_LEN][2];
extern uint8_t ipts1[POINTS_MAX_LEN][2];
extern uint8_t ipts0_num, ipts1_num;
extern float block_size;
extern float clip_value;
//��ϵ�����
extern float  rpts[MT9V03X_H][2];
extern uint8 rpts_num;
//ͼ����������
void ImageProcess(image_t * image);
void Find_Line(image_t * image);
void Image_Black(uint8_t Width,uint8_t Height,uint8_t *p);
//�Թ�Ѱ��,�ֲ���ֵ��
void findline_lefthand_adaptive(image_t *img, float block_size, float clip_value, uint8 x, uint8 y, uint8 pts[][2], uint8 *num);
void findline_righthand_adaptive(image_t *img, float block_size, float clip_value, uint8 x, uint8 y, uint8 pts[][2], uint8 *num);

//�������
void Fit_Mid_Line(uint8 ipts0[][2],uint8 *num0,uint8 ipts1[][2],uint8 *num1,float (*Mid)[2],uint8 *Mid_num);

//��ȡͼ��ƫ��
float Get_Error(uint8 width,uint8 begin_err,float (*rpts)[2],uint8 *Mid_num);

#endif /* CODE_CAMERA_H_ */
