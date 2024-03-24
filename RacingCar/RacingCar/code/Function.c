#include "Function.h"
#include "string.h"
#include "math.h"

//�������б�� �ʺϴ�б���ж� �Ƿ�Ϊֱ�����ж�
float Slope_2Points(int begin,uint8_t end,uint8_t (*ipts)[2])
{
    if(begin<0)begin=0;
    float dominator,nominator;
    dominator = ipts[end][0]-ipts[begin][0];//������֮��x����
    nominator = ipts[end][1]-ipts[begin][1];//����֮��y��֮��Ĳ��
    if(dominator==0){dominator=-1;}//��ֹ����
    return (nominator/dominator);
}

//б�����
void Intercept_Calculate(uint8_t D,uint8_t (*ipts)[2],float *slope,float *intercept)
{
    *intercept = ipts[D][1]-*slope*ipts[D][0];
}

//������
void Patch_Line_Points(uint8_t D,uint8_t U,uint8_t (*ipts)[2])
{
    float slope,intercept;
    slope = Slope_2Points(D-1,U+1,ipts);
    //Slope_Calculate(U+2,U+7,ipts,&slope);
    Intercept_Calculate(U+1,ipts,&slope,&intercept);
    uint8_t temp =ipts[D][1];
//    uint8_t cross_step = ipts[U][1]-ipts[D][1]-1;
    for (uint8_t i=D+1;i<U;i++)
    {
        ipts[i][1]=temp;
        temp--;//����yֵ
        ipts[i][0] = (uint8_t)((ipts[i][1]-intercept)/slope);

    }

}

//��С���˷����б�� �ʺ�Сб���ж�
/*
*@param:��ʼ��begein ��ֹ��end(����������) end>begin
ֱ����б����1~2֮��
*@author:C����
*@date:2024-2-1
*/
float Slope_Calculate(int begin,uint8_t end,uint8_t (*p)[2])
{
if(begin<0)begin=0;//���籣��
float xsum=0,ysum=0,xysum=0,x2sum=0;
int i=0;
for(i=begin;i<=end;i++)
{
   xsum +=(float)p[i][0];
   ysum +=(float)p[i][1];
   xysum+=(float)(p[i][0]*p[i][1]);
   x2sum+=(float)(p[i][0]*p[i][0]);
}

if((end-begin+1)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
{
return ((end-begin+1)*xysum-xsum*ysum)/((end-begin+1)*x2sum-xsum*xsum);
}
else
{
return 2;
}

}

//��ά�����Ǩ����,��A�����ݰ�Ǩ��B,����Start_A��End_A
void copyArrays(uint8_t (*A)[2], uint8_t (*B)[2], uint8_t Start_A, uint8_t End_A,uint8_t Start_B)
{
    uint8_t size_to_copy = (End_A - Start_A + 1) * 2;
    memcpy(B[Start_B], A[Start_A], size_to_copy * sizeof(uint8_t));
}

//�����������
/*
*@param:����1:�������� ����2:��ʼ�� ����3:����������
*@author:C����
*@date:2024-2-14
*/
float process_curvity(uint8_t (* ipts)[2],uint8_t m, uint8_t separate)
{
    float K,q1,AB,BC,AC;
    uint8_t x1 = ipts[m][0];
    uint8_t y1 = ipts[m][1];
    uint8_t x2 = ipts[m+separate][0];
    uint8_t y2 = ipts[m+separate][1];
    uint8_t x3 = ipts[m+2*separate][0];
    uint8_t y3 = ipts[m+2*separate][1];
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;    //����ķ��ű�ʾ����
    q1 = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    AB = sqrt(q1);
    q1 = ((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    BC = sqrt(q1);
    q1 = ((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    AC = sqrt(q1);
    if (AB * BC * AC == 0)
    {
        K = 0;
    }
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}
