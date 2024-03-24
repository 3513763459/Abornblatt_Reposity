/*
 * Camera.c
 *
 *  Created on: 2023��12��9��
 *      Author: LINEAR
 *      ��ע��,��ʱ������ͼ����ʱ��,�޸�MT9V03X_FPS_DEFͼ��֡��
 */
#include "Camera.h"
#include "Function.h"

#define     TC             TC_IMAGE
#define     HANDOFF        20 //��˼�ǵ�Ѱ�������ұ߽�����м��(�۴�ͼ���м�����ɨ��),���ص����<=10��ʱ���л�ѭ�߽�ģʽ,eg:���ҹ����ʱ��,�������߽������ͼ���Ҳ�ƽ�,һ����߽���ͼ���м���ұ�,��ô���Ѳ����߽���

image_t img_raw = DEF_IMAGE(NULL,MT9V03X_W,MT9V03X_H);
uint8 Gray_Image[MT9V03X_H][MT9V03X_W];//������ͷ������ͼ��ת�Ƶ�����,������ͼ��������
Image_DataTypedef Image_DataStruct;//���ｫ�洢ͼ�����õ�����Ҫ���ݳ��ص����ı���?
// ԭͼ���ұ���
uint8_t ipts0[POINTS_MAX_LEN][2];
uint8_t ipts1[POINTS_MAX_LEN][2];
uint8_t ipts0_num, ipts1_num;
float block_size = 5;
float clip_value = 2;

//��ϵ�����
float  rpts[POINTS_MAX_LEN][2];
uint8 rpts_num;

void ImageProcess(image_t * image)    //ͼ����������
{
    memcpy(Gray_Image,mt9v03x_image,MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0;//ʹ����һ֡DMA�����ͼ��ͼ��  ���Կ�ʼ������һ֡

    Image_Black(MT9V03X_W,MT9V03X_H,(uint8 *)image->data);

    Find_Line(image);//����Ѳ��

    //�ҵ�����,�����������
    Fit_Mid_Line(ipts0,&ipts0_num,ipts1,&ipts1_num,rpts,&rpts_num);//�������ұ���,�������,����Ӧ��������Ԫ��ʶ����Ϻ����

    Image_DataStruct.Camera_Error = Get_Error(MT9V03X_W,Image_DataStruct.begin_err,rpts,&rpts_num);
}


void Find_Line(image_t * image)
{

    uint8_t x0 =2, y = image->height-5;//-5��Ϊ�˱�����ֲ���ֵ��3x3��ײ
    uint8_t x1 = image->width -3;
    ipts0_num = sizeof(ipts0) / sizeof(ipts0[0]);//���ٸ��� 120������0~119
    ipts1_num = sizeof(ipts1) / sizeof(ipts1[0]);

//x1 from right find left break point
       for (; x1 > 0;x1--) if (Diff_Ratio(TC_IMAGE(image,x1,y),TC_IMAGE(image,x1-1,y))>Image_DataStruct.Diff_Ratio) break;
//x0 from left find right break point
       for (; x0< image->width - 3;x0++)if (Diff_Ratio(TC_IMAGE(image,x0,y),TC_IMAGE(image,x0+1,y))>Image_DataStruct.Diff_Ratio) {break;}


if(x0>=image->width/2 && x1 >= image->width/2){x1=3;}
if(x1<=image->width/2 && x0 <= image->width/2){x0=image->width-5;}
//x1 should be larger than x0
Image_DataStruct.begin_x =(x0+x1)/2;

for (;Image_DataStruct.begin_x > 0; Image_DataStruct.begin_x--)
if (Diff_Ratio(TC_IMAGE(image, Image_DataStruct.begin_x, y),TC_IMAGE(image, Image_DataStruct.begin_x - 1, y))> Image_DataStruct.Diff_Ratio) break;
findline_lefthand_adaptive(image, block_size, clip_value, Image_DataStruct.begin_x, y, ipts0, &ipts0_num);

for(;Image_DataStruct.begin_x < image->width - 1; Image_DataStruct.begin_x++)
if(Diff_Ratio(TC_IMAGE(image, Image_DataStruct.begin_x, y),TC_IMAGE(image, Image_DataStruct.begin_x + 1, y))> Image_DataStruct.Diff_Ratio) {break;}
findline_righthand_adaptive(image, block_size, clip_value, Image_DataStruct.begin_x, y, ipts1, &ipts1_num);
}

//�ڿ����
inline void Image_Black(uint8_t Width,uint8_t Height,uint8_t *p)
{
for(uint8_t j=0;j<BlackBoundry;j++)
{
for(uint8_t i=0;i<Height;i++)
{
    p[i*Width+j]=30;//��0�мӺڿ�,����MT9V03X��187���Զ����˺ڿ�,���ﲻ�����
    p[i*Width+Width-1-j]=30;//����ͼ���187���ǰ�ɫ,��ԭ������187�кڱ�
}
}
}

/* ǰ�������壺
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2] = {{0,  -1},
                            {1,  0},
                            {0,  1},
                            {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1},
                                {1,  -1},
                                {1,  1},
                                {-1, 1}};
const int dir_frontright[4][2] = {{1,  -1},
                                 {1,  1},
                                 {-1, 1},
                                 {-1, -1}};

//�����Թ�Ѱ��
void findline_lefthand_adaptive(image_t *img, float block_size, float clip_value, uint8 x, uint8 y, uint8 pts[][2], uint8 *num)
{

    int half = block_size / 2; // 3
    uint8_t step = 0, dir = 0, turn = 0;
    while (step < *num && 0 < x && x < img->width - half - 1 && half < y && y < img->height - half - 1 && turn < 4)
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++)
        {
            for (int dx = -half; dx <= half; dx++)
             {
                if ((x + dx)<0)
                    continue;
                else
                      local_thres += TC(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value = TC(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = TC(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres)
        {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres)
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else
         {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

//�����Թ�Ѱ��
void findline_righthand_adaptive(image_t *img, float block_size, float clip_value, uint8 x, uint8 y, uint8 pts[][2], uint8 *num)
{

    int half = block_size / 2;
    uint8_t step = 0, dir = 0, turn = 0;
    while (step < *num && half < x && x < img->width-1 && half < y && y < img->height-half-1 && turn < 4)
      {
          int local_thres = 0;
          for (int dy = -half; dy <= half; dy++)
          {
              for (int dx = -half; dx <= half; dx++)
              {
                  if(x+dx>img->width-1)continue;
                  else
                  local_thres += TC(img, x + dx, y + dy);
              }
          }
          local_thres /= block_size * block_size;
          local_thres -= clip_value;

          int front_value = TC(img, x + dir_front[dir][0], y + dir_front[dir][1]);
          int frontright_value = TC(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
          if (front_value < local_thres)//���ǰ��Ϊ��ɫ
          {
              dir = (dir + 3) % 4;
              turn++;
          } else if (frontright_value < local_thres) //���ǰ��Ϊ��ɫ �� ǰ�ҷ�Ϊ��ɫ,turn=0,dir���ֲ���
          {
              x += dir_front[dir][0];
              y += dir_front[dir][1];
              pts[step][0] = x;
              pts[step][1] = y;
              step++;
              turn = 0;
          }
          else//���ǰ��Ϊ��ɫ  ǰ�ҷ�ҲΪ��ɫ,��ǰ�ҷ�����
           {
              x += dir_frontright[dir][0];
              y += dir_frontright[dir][1];
              dir = (dir + 1) % 4;
              pts[step][0] = x;
              pts[step][1] = y;
              step++;
              turn = 0;
          }
      }
      *num = step;
}

//˼·�������ȡ�������ұ����������ٵĵ�
void Fit_Mid_Line(uint8 ipts0[][2],uint8 *num0,uint8 ipts1[][2],uint8 *num1,float (*Mid)[2],uint8 *Mid_num)
{
   *Mid_num = min(*num0,*num1);//�����ó������߲����ȶ�
   for(uint8 i=0;i<*Mid_num;i++)
   {
       Mid[i][0] = (ipts0[i][0]+ipts1[i][0])/2;
       Mid[i][1] = (ipts0[i][1]+ipts1[i][1])/2;
   }
}

/*��ȡͼ��ƫ��
 * ����1:ͼ����
 * ����2:����������ĵڼ�������ʼ����ƫ��
 * ����3:��������
 * ƫ��С��0,����С��Ҫ���ҹ�
 * */
float Weight[11] ={1 , 2 , 3 , 4 , 5 , 6 , 5 , 4 , 3 , 2 , 1};//Ȩ��11������
inline float Get_Error(uint8 width,uint8 begin_err,float (*rpts)[2],uint8 *Mid_num)
{
   float err=0;static float camera_err_1=0;
   float weight_count=0;
    for (uint8 i=0;i<sizeof(Weight)/sizeof(Weight[0]);i++)//����������ĳһ������ʼ����10�п�ʼ����ƫ��
    {
         err+=(width/2-rpts[i+begin_err][0])*Weight[i];
         weight_count += Weight[i];
    }
    err = err/weight_count;

    /*�������,������һ��*/
    if(err == 0)return camera_err_1;
    camera_err_1=err;

    return err;
}


