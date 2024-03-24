#include "Element.h"
#include "Function.h"

#define ELEMENT_DISTANCE  5 //x三步趋势判定 防止毛刺
#define ELEMENT_THRESHOLD 3 //y值比较所用
#define TREND_DISTANCE 10 //两步趋势判定所用距离

Element_TendTypedef Element_DataEnum;//十字预警枚举体,给出拐点即将消失信息
Element_JustTypedef Element_JustStruct;//直道 圆环 十字拐点斜率判定临界值
uint8_t iptsld[POINTS_MAX_LEN][2];
uint8_t iptsrd[POINTS_MAX_LEN][2];
uint8_t iptsld_num, iptsrd_num;

//入圆环前上拐点 上探数组
uint8_t iptslu[POINTS_MAX_LEN][2];//左路
uint8_t iptsru[POINTS_MAX_LEN][2];//右路
uint8_t iptslu_num, iptsru_num;

/**
 * 定义文件域循环变量
 */
uint8_t i,j;

/**
 * 圆环判定过程中需要摆脱上一个阶段,进入下一个阶段,此函数就是圆环各阶段的摆脱判断
 *
 */
void Element_Phase()
{
    static uint8_t Circle_Slope_Last=0;
    switch(System_DataStruct.Element_Flag)
    {
    case LEFT_CIRCLE_IN:
            /**********************************************************左圆环判定阶段**************************************************/
        switch(System_DataStruct.CIRCLE_Flag)
        {
        case CIRCLE_PREV_D_M://存在上下拐点
                        if(ipts0[0][0]==ipts0[2][0] && ipts0[2][0] ==ipts0[4][0])
                        {
                            System_DataStruct.CIRCLE_Flag=CIRCLE_PREV_M;//进入圆环二阶段,等待二阶段的处理,处理函数在下面
                            break;
                        }
            break;
        case CIRCLE_PREV_M://只有中拐点,没有下拐点
                        if(ipts0[0][0]!=ipts0[1][0]&& ipts0[1][0] !=ipts0[2][0])
                        {
                            System_DataStruct.CIRCLE_Flag=CIRCLE_WILL_IN;//进入圆环三阶段,等待三阶段的处理,处理函数在下面
                            break;
                        }
            break;
        case CIRCLE_WILL_IN:

                        if(Slope_2Points(0,TREND_DISTANCE,ipts1)==TREND_DISTANCE)
                        {
                            Circle_Slope_Last=TREND_DISTANCE;
                            break;
                        }
                        if(Circle_Slope_Last==TREND_DISTANCE &&Slope_2Points(0,TREND_DISTANCE,ipts1)!=TREND_DISTANCE)
                        {
                            System_DataStruct.CIRCLE_Flag=CIRCLE_HAVE_IN;
                            break;
                        }
            break;
        case CIRCLE_HAVE_IN:
                        for (i=0;i<ipts1_num-TREND_DISTANCE;i++)
                        {
                            if(ipts1[i][0] < ipts1[i+TREND_DISTANCE][0] && ipts1[i][0] < ipts1[i+2*TREND_DISTANCE][0])//x出现增加趋势,进行锁定,之所以不是确定,是因为出环拐点也会消失
                            {
                               System_DataStruct.CIRCLE_Flag = CIRCLE_OUT_D;
                                printf("%s\n","hahah");
                                break;
                            }
                        }
            break;
        case CIRCLE_OUT_D:
                        for (i=0;i<ipts0_num-ELEMENT_DISTANCE;i++)
                        {
                            //上拐点
                            if((abs(Slope_Calculate(i,i+ELEMENT_DISTANCE,ipts0))<Element_JustStruct.Flat_Slope) && (Slope_2Points(i+ELEMENT_DISTANCE,i+2*ELEMENT_DISTANCE,ipts0) < -Element_JustStruct.Straight_Slope))
                            {
                                System_DataStruct.CIRCLE_Flag = CIRCLE_OUT_U;
                                break;
                            }
                        }
            break;
        case CIRCLE_OUT_U:
                        if(ipts0[0][0]!=ipts0[ELEMENT_THRESHOLD][0] && ipts0[ELEMENT_THRESHOLD][0]!=ipts0[TREND_DISTANCE][0])
                         {
                                 System_DataStruct.CIRCLE_Flag = ELEMENMT_NONE;
                                 break;
                         }
            break;
        default:
            break;
        }

    /**********************************************************左圆环判定阶段**************************************************/
        break;
    default :
        break;
    }

}

/**
 * 知道处于元素哪个阶段,完成拐点索引查找
 * */
void Element_Knee(void)
{
    switch(System_DataStruct.Element_Flag)
    {
    case LEFT_CIRCLE_IN:
                        switch(System_DataStruct.CIRCLE_Flag)//看看是圆环的哪个阶段
                            {
                            case CIRCLE_PREV_D_M : //左圆环一阶段,出现x减小趋势,实锤下拐点
                                    for (i=0;i<ipts0_num-ELEMENT_DISTANCE;i++)
                                        {
                                            if( (ipts0[i][1] <= ipts0[i+ELEMENT_DISTANCE][1]) && (ipts0[i+TREND_DISTANCE][1] - ipts0[i][1] >= ELEMENT_THRESHOLD) )
                                            {
                                                Element0_DataStruct.Round_Prev_L=i;
                                                break;
                                            }
                                        }
                                //存在下拐点,尝试去找中拐点,找到了,两点连线,找不到,从下向上补线
                                iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                                //Plug_in_line(&img_raw,Element0_DataStruct.Round_Prev_L+TREND_DISTANCE,ipts0,iptsld,iptsld_num,0);
                                    for(i=ELEMENT_DISTANCE;i<iptsld_num-2*ELEMENT_DISTANCE;i++)
                                    {//x增加又减小
                                          if(iptsld[i][0]<iptsld[i+ELEMENT_DISTANCE][0] && iptsld[i+ELEMENT_DISTANCE][0]>=iptsld[i+2*ELEMENT_DISTANCE][0])
                                            {
                                                Element0_DataStruct.Round_Prev_M=i+ELEMENT_DISTANCE;
                                                break;
                                            }
                                    }
                                break;
                            case CIRCLE_PREV_M:
                                    for (i=ELEMENT_DISTANCE;i<ipts0_num-2*ELEMENT_DISTANCE;i++)//本目120个点,为0~119,所以需要-1
                                        {//赛道宽度减小又增加
                                          if( (ipts1[i][0]-ipts0[i][0])>(ipts1[i+ELEMENT_THRESHOLD][0]-ipts0[i+ELEMENT_THRESHOLD][0])
                                                && (ipts1[i+ELEMENT_THRESHOLD][0]-ipts0[i+ELEMENT_THRESHOLD][0])<=(ipts1[i+2*ELEMENT_THRESHOLD][0]-ipts0[i+2*ELEMENT_THRESHOLD][0]) )
                                            {
                                                Element0_DataStruct.Round_Prev_M=i-ELEMENT_THRESHOLD;
                                                goto CIRCLE_PREV_M_End;
                                            }
                                        }
                                    //线太短,找不到宽度突变点
                                    iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                                   Plug_in_line(&img_raw,Element0_DataStruct.Round_Prev_L,ipts0,iptsld,iptsld_num,0);
                                        for(i=ELEMENT_DISTANCE;i<iptsld_num-2*ELEMENT_DISTANCE;i++)
                                        {//x增加又减小
                                              if(iptsld[i][0]<iptsld[i+ELEMENT_DISTANCE][0] && iptsld[i+ELEMENT_DISTANCE][0]>=iptsld[i+2*ELEMENT_DISTANCE][0])
                                                {
                                                    Element0_DataStruct.Round_Prev_M=i+ELEMENT_DISTANCE;
                                                    break;
                                                }
                                        }
                                CIRCLE_PREV_M_End:
                                break;
                                break;
                            case CIRCLE_WILL_IN:
                                    iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                                    Plug_in_line(&img_raw,119,ipts0,iptsld,iptsld_num,0);
                                        for(i=ELEMENT_DISTANCE;i<iptsld_num-2*ELEMENT_DISTANCE;i++)
                                        {//
                                              if(iptsld[i][1]<iptsld[i+ELEMENT_DISTANCE][1] && iptsld[i+ELEMENT_DISTANCE][1]>iptsld[i+2*ELEMENT_DISTANCE][1])
                                                {
                                                    Element0_DataStruct.Round_Prev_U=i+ELEMENT_DISTANCE;
                                                    break;
                                                }
                                        }
                                break;
                            default:
                                break;
                            }
        break;
    default:
        break;
    }

}

void Element_Handle(void)
{
    switch(System_DataStruct.Element_Flag)
    {
    case LEFT_CIRCLE_IN:
                        switch(System_DataStruct.CIRCLE_Flag)
                        {
                        case CIRCLE_PREV_D_M:

                            break;
                        default:
                            break;
                        }
        break;
    default:
        break;
    }
}

/**
 * 偏移上探
 * image:图像指针
 * plug:从此处索引进行上探查找
 * ipts:提供索引的数组
 * iptso:存储新一轮边线的数组
 * iptso_num:步数
 * mode:巡线方式 0:左手 1:右手
 */
void Plug_in_line(image_t *image,uint8_t plug,uint8_t (*ipts)[2],uint8_t (*iptso)[2],uint8_t iptso_num,bool mode)
{
if(ipts[plug][0] == BlackBoundry)
{
    i = ipts[plug][0]+BlackBoundry;
}
else if(ipts[plug][0] == image->width-1-BlackBoundry)
{
i = ipts[plug][0]-BlackBoundry;
}
else
{
i = ipts[plug][0];
}
switch(mode)
{
case 0:
    for(j=ipts[plug][1];j>5;j--)
    if (Diff_Ratio(TC_IMAGE(image,i,j),TC_IMAGE(image,i,j-1))> Image_DataStruct.Diff_Ratio) break;
    findline_lefthand_adaptive(image,block_size+2,clip_value+2,i,j,iptso,&iptso_num);
    break;
case 1:
    for(j=ipts[plug][1];j>5;j--)
    if (Diff_Ratio(TC_IMAGE(image,i,j),TC_IMAGE(image,i,j-1))> Image_DataStruct.Diff_Ratio) break;
    findline_righthand_adaptive(image,block_size+2,clip_value+2,i,j,iptso,&iptso_num);
    break;
}
}

/*
*@param:识别右拐点,包括上下返回的是拐点在数组中的索引
* 正常情况是右边线的x坐标轴逐渐减小,但是如果出现增加的现象,就是拐点
* 弯道情况也会出现x减小又增加,为了避免误判,增加y轴差值比较
* 弯道容易误判,所以增加判断x值差值比较，这个是往回比较，左弯道比十字更弯
* !param:往回比较时i-ELEMENT_ISTANCE 所以进入十字中时，最好是在下拐点快要消失，保证i-ELEMENT_ISTANCE>0
*
*@author:Chen
*@date:2024-1-19
*/
void find_RD_point(uint8_t (* ipts1)[2],uint8_t * ipts1_num)
{
        for (i=0;i<*ipts1_num-ELEMENT_DISTANCE;i++)
    {
        if(ipts1[i][0] < ipts1[i+1][0] && ipts1[i][0] < ipts1[i+2][0])//发现x增加趋势,初步判定拐点，趋势两步判断，防止出现坑洼赛道影响判断
        {
           //十字斜率拐点判定
            if((abs(Slope_2Points(i,i+TREND_DISTANCE,ipts1))<Element_JustStruct.Flat_Slope) && (abs(Slope_2Points(i-TREND_DISTANCE,i,ipts1))>Element_JustStruct.Straight_Slope))//相邻10点斜率判断
            {
                Element1_DataStruct.CD=i;
                Element1_DataStruct.CD_flag=1;
                break;
            }
            //十字Y差拐点判定 //完全可以充当出圆环下拐点
            // if( (ipts1[i][1] - ipts1[i+ELEMENT_DISTANCE][1] < ELEMENT_THRESHOLD) && (ipts1[i][0] - ipts1[i-ELEMENT_DISTANCE][0] < ELEMENT_THRESHOLD))
            // {//找到拐点，返回在边线数组中的索引，进入十字一阶段
            //     Element1_DataStruct.CD=i;
            //     Element1_DataStruct.CD_flag=1;
            //     printf("flat: %f straight: %f\n",abs(Slope_Calculate(i,i+TREND_DISTANCE,ipts1)),Slope_2Points(i-TREND_DISTANCE,i,ipts1));
            //     //printf("curity: %f\n", process_curvity(ipts1,i-10,4));//斜入十字的曲率在0.2~0.4之间? 直十字下拐点的曲率在0.35左右
            //     break;
            // }
        }
    }
}

/*
*@param:元素锁定,只进一次,首先进行趋势判断，其次进行y值判断
*@author:
*@date:
*/
void find_LD_point(uint8_t (*ipts0)[2],uint8_t *ipts0_num)
{
    if(System_DataStruct.Element_Flag == LEFT_CIRCLE_IN)//如果已经进入圆环状态,不用继续,直接出来
    {
        return;
    }
    for (i=0;i<*ipts0_num-ELEMENT_DISTANCE;i++)
    {

       //如果出现x减小的趋势,进行拐点判断 趋势三步判断
        if( ipts0[i][0] > ipts0[i+1][0] && ipts0[i][0] > ipts0[i+2][0] && ipts0[i][0] > ipts0[i+TREND_DISTANCE][0])
        {

            //十字斜率拐点判定 缺少斜入十字
             if((ipts0[i][1] - ipts0[i+ELEMENT_DISTANCE][1] < ELEMENT_THRESHOLD) && (abs(Slope_Calculate(i,i+TREND_DISTANCE,ipts0))<Element_JustStruct.Flat_Slope)&& (Slope_2Points(i-TREND_DISTANCE,i,ipts0)<-Element_JustStruct.Straight_Slope))//相邻10点斜率判断
            {
                Element0_DataStruct.CD=i;
                Element0_DataStruct.CD_flag=1;
                break;
            }
            //进入圆环,锁定圆环
             if( (ipts0[i][1] <= ipts0[i+ELEMENT_DISTANCE][1]) && (ipts0[i+TREND_DISTANCE][1] - ipts0[i][1] >= ELEMENT_THRESHOLD)
                && System_DataStruct.Element_Flag ==ELEMENMT_NONE)//使用绝对值反而容易产生误判
            {
                Element0_DataStruct.Round_Prev_L=i;
                System_DataStruct.Element_Flag = LEFT_CIRCLE_IN;//锁定进入左圆环
                System_DataStruct.CIRCLE_Flag = CIRCLE_PREV_D_M;//CIRCLE first phase the fist handle following
                break;
            }

            //十字Y差拐点判定
                // if( (ipts0[i][1] - ipts0[i+ELEMENT_DISTANCE][1] < ELEMENT_THRESHOLD) && (ipts0[i-ELEMENT_DISTANCE][0]-ipts0[i][0]< ELEMENT_THRESHOLD))//y值判断
                // {
                //     Element0_DataStruct.CD=i;
                //     Element0_DataStruct.CD_flag=1;
                //     printf("%f %f\n", Slope_Calculate(i,i+TREND_DISTANCE,ipts0),Slope_2Points(i-TREND_DISTANCE,i,ipts0));
                //     break;
                // }
        }

    }

}

/*
*@param:查找十字上拐点的条件有两个 1:本次下拐点存在 2:上次上拐点存在
上探查找入圆环前中拐点条件 本次下拐点存在
*@author:
*@date:
*/
uint8_t CrossRoad_IN_M=0;
void find_LU_point(image_t * image,uint8_t (*ipts0)[2])
{
    uint8_t translate_x,translate_y;
   /*得到下拐点平移一段距离的索引*/
    if(Element0_DataStruct.CD_flag==1 ||Element0_DataStruct.CU_adjacent == CROSS_U_EXIST)
    {
        translate_x=ipts0[Element0_DataStruct.CD+ELEMENT_DISTANCE][0];
        translate_y=ipts0[Element0_DataStruct.CD+ELEMENT_DISTANCE][1];//下拐点平移一段距离,来到下拐点一侧的平滑界面
    }
    else return;

    //找到下拐点,然后偏移一点,差不多到有拐点右侧的平滑边界, 上探查找上拐点的平滑边界
    for (;translate_y>3;translate_y--)
    {
          if(Diff_Ratio(TC_IMAGE(image,translate_x,translate_y),TC_IMAGE(image,translate_x,translate_y-1))>Image_DataStruct.Diff_Ratio)
            {
                //找到上拐点的边界后,开始巡线
                iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                findline_lefthand_adaptive(image, block_size, clip_value, translate_x, translate_y, iptsld,&iptsld_num);

                //第一上探数组区域
                for (uint8_t m =0;m<iptsld_num-2*ELEMENT_DISTANCE;m++)
                {
/*********************************************ELEMENT BEGIN*********************************************/
                            //十字上拐点
                            if((Element0_DataStruct.CD_flag == 1 || Element0_DataStruct.CU_adjacent == CROSS_U_EXIST) && (abs(Slope_Calculate(m,m+ELEMENT_DISTANCE,iptsld))<Element_JustStruct.Flat_Slope) && (Slope_2Points(m+ELEMENT_DISTANCE,m+2*ELEMENT_DISTANCE,iptsld) < -Element_JustStruct.Straight_Slope))
                            {
                                Element0_DataStruct.CU=m+ELEMENT_DISTANCE;//RU代表着在全新数组中全新的索引,拼接完成数组后,需要对RU进行修改值应该为RD+step,这个step是两点之间赛道应有的点数
                                Element0_DataStruct.CU_flag=1;//进入二阶段,上下拐点识别完毕,那么就是要面临下拐点消失啦！
                                //printf("%f %f\n",Slope_Calculate(m,m+ELEMENT_DISTANCE,iptsld),Slope_2Points(m,m+ELEMENT_DISTANCE,iptsld) );
                                break;
                            }
/*********************************************ELEMENT END*********************************************/
                }

                break;
            }
    }

}


/*
*@param:右上拐点查找,下拐点是上拐点的必要
* 如果需要查找上拐点，那么下拐点是否有必要确定接触了边界？目前采取的策略是下拐点平移进行上探查找
*@author:C先生
*@date:2024-1-20
*/
void find_RU_point(image_t * image,uint8_t (*ipts1)[2])
{
    uint8_t translate_x,translate_y;
    /*得到下拐点平移一段距离的x,y值*/
    if (Element1_DataStruct.CD_flag == 1 || Element1_DataStruct.CU_adjacent == CROSS_U_EXIST)
    {
        translate_x=ipts1[Element1_DataStruct.CD+ELEMENT_DISTANCE][0];
        translate_y=ipts1[Element1_DataStruct.CD+ELEMENT_DISTANCE][1];
    }
    else return;


    //找到下拐点,然后偏移一点,差不多到有拐点右侧的平滑边界, 上探查找上拐点的平滑边界
    for (;translate_y>3;translate_y--)
    {    //使用差比和获取更抗光?
        if(Diff_Ratio(TC_IMAGE(image,translate_x,translate_y),TC_IMAGE(image,translate_x,translate_y-1))>Image_DataStruct.Diff_Ratio)
            {
                //找到上拐点的边界后,开始巡线,将得到的上边线储存在新的边线数组中
                iptsrd_num = sizeof(iptsrd) / sizeof(iptsrd[0]);
                findline_righthand_adaptive(image, block_size, clip_value, translate_x, translate_y, iptsrd,&iptsrd_num);

            //紧接着开始搜索上角点
            //x减小，当出现增加的趋势，可以判断上拐点
                for (uint8_t m =0;m<iptsrd_num-ELEMENT_DISTANCE;m++)
                {
                        if((iptsrd[m][1] - iptsrd[m+ELEMENT_DISTANCE][1] > ELEMENT_THRESHOLD) )
                        {
                            Element1_DataStruct.CU=m;//RU代表着在全新数组中全新的索引,拼接完成数组后,需要对RU进行修改值应该为RD+step,这个step是两点之间赛道应有的点数
                            Element1_DataStruct.CU_flag=1;//说明进入二阶段,上下拐点都识别完毕
                            break;
                        }
                }
            //上探数组的初始位置应该在拐点旁边的平滑区域,所以斜率一定小于0.5(abs),如果不是,那么就不再十字了
            if(abs(Slope_Calculate(Element1_DataStruct.CU-ELEMENT_DISTANCE,Element1_DataStruct.CU,iptsrd))>0.5){Element1_DataStruct.CU=1;Element1_DataStruct.CU_flag=0;}
                break;
            }
    }
}
//十字上下拐点寻找完毕,拼接开始,此时crossflag=2
/*
*@param1:元素结构体指针
*@param2:源边线数组
*@param3:源边线步数
*@param4:上边线数组
*@param5:上边线步数
*@param6: iptscc入圆环前上拐点数组
*@param7: 异路补线数组
*@author:C先生
*@date:2024-1-27
*/
void Trim_DU(Element_DataTypedef * sptr,uint8_t(*ipts)[2],uint8_t * ipts_num,uint8_t (*iptsc)[2])
{
    if (sptr->CU_flag==1)
    {
    uint8_t cross_step = ipts[sptr->CD][1]-iptsc[sptr->CU][1]-1;//得到两点之间有多少行-1是必要的
    uint8_t D_zerofill = POINTS_MAX_LEN-sptr->CD-cross_step-1;//得到下拐点所在的old数组在下拐点之后在加上cross_step个点后 还有多少个点
    uint8_t U_zerofill = POINTS_MAX_LEN-sptr->CU;//得到CU后面的点数(这个需要包括CU)

//!@:会出现上拐点后面的点数小于zerofill
    if(U_zerofill>=D_zerofill)
        {
        copyArrays(iptsc,ipts,sptr->CU,sptr->CU+D_zerofill-1,sptr->CD+1+cross_step);
        *ipts_num=sptr->CD+cross_step+D_zerofill+1;//修改步长，代表的是点数
        }
    else
        {
        copyArrays(iptsc,ipts,sptr->CU,sptr->CU+U_zerofill-1,sptr->CD+1+cross_step);
        *ipts_num=sptr->CD+cross_step+U_zerofill+1;//修改步长,代表的是点数
        }

        sptr->CU=sptr->CD+cross_step+1;//将RU编排进入RD数组内
    }

}


//对于连续图像,面临上下拐点即将消失,需要给出预警信息
/*
*@param:记录上一幅图像信息,并清理标志位
*@author:
*@date:
*/
void Element_Info_Deliver(Element_DataTypedef * sptr)
{
if(sptr->CD_flag==1){sptr->CD_flag=0;}//上幅图像下拐点存在,刷新图间标志量(CD)
else if(sptr->CD_flag ==0){sptr->CD=1;}

if(sptr->CU_flag==1){sptr->CU_adjacent =CROSS_U_EXIST;sptr->CU_flag=0;}
else if(sptr->CU_flag ==0){sptr->CU_adjacent =CROSS_U_DISMISS;sptr->CU=0;}

}

