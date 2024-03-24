#include "Function.h"
#include "string.h"
#include "math.h"

//两点计算斜率 适合大斜率判定 是否为直道的判定
float Slope_2Points(int begin,uint8_t end,uint8_t (*ipts)[2])
{
    if(begin<0)begin=0;
    float dominator,nominator;
    dominator = ipts[end][0]-ipts[begin][0];//求两点之间x轴差距
    nominator = ipts[end][1]-ipts[begin][1];//两点之间y轴之间的差距
    if(dominator==0){dominator=-1;}//防止除飞
    return (nominator/dominator);
}

//斜距计算
void Intercept_Calculate(uint8_t D,uint8_t (*ipts)[2],float *slope,float *intercept)
{
    *intercept = ipts[D][1]-*slope*ipts[D][0];
}

//填充边线
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
        temp--;//更新y值
        ipts[i][0] = (uint8_t)((ipts[i][1]-intercept)/slope);

    }

}

//最小二乘法拟合斜率 适合小斜率判定
/*
*@param:起始于begein 终止于end(包括这两点) end>begin
直道的斜率在1~2之间
*@author:C先生
*@date:2024-2-1
*/
float Slope_Calculate(int begin,uint8_t end,uint8_t (*p)[2])
{
if(begin<0)begin=0;//过界保护
float xsum=0,ysum=0,xysum=0,x2sum=0;
int i=0;
for(i=begin;i<=end;i++)
{
   xsum +=(float)p[i][0];
   ysum +=(float)p[i][1];
   xysum+=(float)(p[i][0]*p[i][1]);
   x2sum+=(float)(p[i][0]*p[i][0]);
}

if((end-begin+1)*x2sum-xsum*xsum) //判断除数是否为零
{
return ((end-begin+1)*xysum-xsum*ysum)/((end-begin+1)*x2sum-xsum*xsum);
}
else
{
return 2;
}

}

//二维数组搬迁函数,把A的数据搬迁到B,包括Start_A到End_A
void copyArrays(uint8_t (*A)[2], uint8_t (*B)[2], uint8_t Start_A, uint8_t End_A,uint8_t Start_B)
{
    uint8_t size_to_copy = (End_A - Start_A + 1) * 2;
    memcpy(B[Start_B], A[Start_A], size_to_copy * sizeof(uint8_t));
}

//三点计算曲率
/*
*@param:参数1:边线数组 参数2:起始点 参数3:三点间隔距离
*@author:C先生
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
    int S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;    //面积的符号表示方向
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
