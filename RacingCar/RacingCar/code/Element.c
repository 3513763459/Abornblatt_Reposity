#include "Element.h"
#include "Function.h"

#define ELEMENT_DISTANCE  5 //x���������ж� ��ֹë��
#define ELEMENT_THRESHOLD 3 //yֵ�Ƚ�����
#define TREND_DISTANCE 10 //���������ж����þ���

Element_TendTypedef Element_DataEnum;//ʮ��Ԥ��ö����,�����յ㼴����ʧ��Ϣ
Element_JustTypedef Element_JustStruct;//ֱ�� Բ�� ʮ�ֹյ�б���ж��ٽ�ֵ
uint8_t iptsld[POINTS_MAX_LEN][2];
uint8_t iptsrd[POINTS_MAX_LEN][2];
uint8_t iptsld_num, iptsrd_num;

//��Բ��ǰ�Ϲյ� ��̽����
uint8_t iptslu[POINTS_MAX_LEN][2];//��·
uint8_t iptsru[POINTS_MAX_LEN][2];//��·
uint8_t iptslu_num, iptsru_num;

/**
 * �����ļ���ѭ������
 */
uint8_t i,j;

/**
 * Բ���ж���������Ҫ������һ���׶�,������һ���׶�,�˺�������Բ�����׶εİ����ж�
 *
 */
void Element_Phase()
{
    static uint8_t Circle_Slope_Last=0;
    switch(System_DataStruct.Element_Flag)
    {
    case LEFT_CIRCLE_IN:
            /**********************************************************��Բ���ж��׶�**************************************************/
        switch(System_DataStruct.CIRCLE_Flag)
        {
        case CIRCLE_PREV_D_M://�������¹յ�
                        if(ipts0[0][0]==ipts0[2][0] && ipts0[2][0] ==ipts0[4][0])
                        {
                            System_DataStruct.CIRCLE_Flag=CIRCLE_PREV_M;//����Բ�����׶�,�ȴ����׶εĴ���,������������
                            break;
                        }
            break;
        case CIRCLE_PREV_M://ֻ���йյ�,û���¹յ�
                        if(ipts0[0][0]!=ipts0[1][0]&& ipts0[1][0] !=ipts0[2][0])
                        {
                            System_DataStruct.CIRCLE_Flag=CIRCLE_WILL_IN;//����Բ�����׶�,�ȴ����׶εĴ���,������������
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
                            if(ipts1[i][0] < ipts1[i+TREND_DISTANCE][0] && ipts1[i][0] < ipts1[i+2*TREND_DISTANCE][0])//x������������,��������,֮���Բ���ȷ��,����Ϊ�����յ�Ҳ����ʧ
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
                            //�Ϲյ�
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

    /**********************************************************��Բ���ж��׶�**************************************************/
        break;
    default :
        break;
    }

}

/**
 * ֪������Ԫ���ĸ��׶�,��ɹյ���������
 * */
void Element_Knee(void)
{
    switch(System_DataStruct.Element_Flag)
    {
    case LEFT_CIRCLE_IN:
                        switch(System_DataStruct.CIRCLE_Flag)//������Բ�����ĸ��׶�
                            {
                            case CIRCLE_PREV_D_M : //��Բ��һ�׶�,����x��С����,ʵ���¹յ�
                                    for (i=0;i<ipts0_num-ELEMENT_DISTANCE;i++)
                                        {
                                            if( (ipts0[i][1] <= ipts0[i+ELEMENT_DISTANCE][1]) && (ipts0[i+TREND_DISTANCE][1] - ipts0[i][1] >= ELEMENT_THRESHOLD) )
                                            {
                                                Element0_DataStruct.Round_Prev_L=i;
                                                break;
                                            }
                                        }
                                //�����¹յ�,����ȥ���йյ�,�ҵ���,��������,�Ҳ���,�������ϲ���
                                iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                                //Plug_in_line(&img_raw,Element0_DataStruct.Round_Prev_L+TREND_DISTANCE,ipts0,iptsld,iptsld_num,0);
                                    for(i=ELEMENT_DISTANCE;i<iptsld_num-2*ELEMENT_DISTANCE;i++)
                                    {//x�����ּ�С
                                          if(iptsld[i][0]<iptsld[i+ELEMENT_DISTANCE][0] && iptsld[i+ELEMENT_DISTANCE][0]>=iptsld[i+2*ELEMENT_DISTANCE][0])
                                            {
                                                Element0_DataStruct.Round_Prev_M=i+ELEMENT_DISTANCE;
                                                break;
                                            }
                                    }
                                break;
                            case CIRCLE_PREV_M:
                                    for (i=ELEMENT_DISTANCE;i<ipts0_num-2*ELEMENT_DISTANCE;i++)//��Ŀ120����,Ϊ0~119,������Ҫ-1
                                        {//������ȼ�С������
                                          if( (ipts1[i][0]-ipts0[i][0])>(ipts1[i+ELEMENT_THRESHOLD][0]-ipts0[i+ELEMENT_THRESHOLD][0])
                                                && (ipts1[i+ELEMENT_THRESHOLD][0]-ipts0[i+ELEMENT_THRESHOLD][0])<=(ipts1[i+2*ELEMENT_THRESHOLD][0]-ipts0[i+2*ELEMENT_THRESHOLD][0]) )
                                            {
                                                Element0_DataStruct.Round_Prev_M=i-ELEMENT_THRESHOLD;
                                                goto CIRCLE_PREV_M_End;
                                            }
                                        }
                                    //��̫��,�Ҳ������ͻ���
                                    iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                                   Plug_in_line(&img_raw,Element0_DataStruct.Round_Prev_L,ipts0,iptsld,iptsld_num,0);
                                        for(i=ELEMENT_DISTANCE;i<iptsld_num-2*ELEMENT_DISTANCE;i++)
                                        {//x�����ּ�С
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
 * ƫ����̽
 * image:ͼ��ָ��
 * plug:�Ӵ˴�����������̽����
 * ipts:�ṩ����������
 * iptso:�洢��һ�ֱ��ߵ�����
 * iptso_num:����
 * mode:Ѳ�߷�ʽ 0:���� 1:����
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
*@param:ʶ���ҹյ�,�������·��ص��ǹյ��������е�����
* ����������ұ��ߵ�x�������𽥼�С,��������������ӵ�����,���ǹյ�
* ������Ҳ�����x��С������,Ϊ�˱�������,����y���ֵ�Ƚ�
* �����������,���������ж�xֵ��ֵ�Ƚϣ���������رȽϣ��������ʮ�ָ���
* !param:���رȽ�ʱi-ELEMENT_ISTANCE ���Խ���ʮ����ʱ����������¹յ��Ҫ��ʧ����֤i-ELEMENT_ISTANCE>0
*
*@author:Chen
*@date:2024-1-19
*/
void find_RD_point(uint8_t (* ipts1)[2],uint8_t * ipts1_num)
{
        for (i=0;i<*ipts1_num-ELEMENT_DISTANCE;i++)
    {
        if(ipts1[i][0] < ipts1[i+1][0] && ipts1[i][0] < ipts1[i+2][0])//����x��������,�����ж��յ㣬���������жϣ���ֹ���ֿ�������Ӱ���ж�
        {
           //ʮ��б�ʹյ��ж�
            if((abs(Slope_2Points(i,i+TREND_DISTANCE,ipts1))<Element_JustStruct.Flat_Slope) && (abs(Slope_2Points(i-TREND_DISTANCE,i,ipts1))>Element_JustStruct.Straight_Slope))//����10��б���ж�
            {
                Element1_DataStruct.CD=i;
                Element1_DataStruct.CD_flag=1;
                break;
            }
            //ʮ��Y��յ��ж� //��ȫ���Գ䵱��Բ���¹յ�
            // if( (ipts1[i][1] - ipts1[i+ELEMENT_DISTANCE][1] < ELEMENT_THRESHOLD) && (ipts1[i][0] - ipts1[i-ELEMENT_DISTANCE][0] < ELEMENT_THRESHOLD))
            // {//�ҵ��յ㣬�����ڱ��������е�����������ʮ��һ�׶�
            //     Element1_DataStruct.CD=i;
            //     Element1_DataStruct.CD_flag=1;
            //     printf("flat: %f straight: %f\n",abs(Slope_Calculate(i,i+TREND_DISTANCE,ipts1)),Slope_2Points(i-TREND_DISTANCE,i,ipts1));
            //     //printf("curity: %f\n", process_curvity(ipts1,i-10,4));//б��ʮ�ֵ�������0.2~0.4֮��? ֱʮ���¹յ��������0.35����
            //     break;
            // }
        }
    }
}

/*
*@param:Ԫ������,ֻ��һ��,���Ƚ��������жϣ���ν���yֵ�ж�
*@author:
*@date:
*/
void find_LD_point(uint8_t (*ipts0)[2],uint8_t *ipts0_num)
{
    if(System_DataStruct.Element_Flag == LEFT_CIRCLE_IN)//����Ѿ�����Բ��״̬,���ü���,ֱ�ӳ���
    {
        return;
    }
    for (i=0;i<*ipts0_num-ELEMENT_DISTANCE;i++)
    {

       //�������x��С������,���йյ��ж� ���������ж�
        if( ipts0[i][0] > ipts0[i+1][0] && ipts0[i][0] > ipts0[i+2][0] && ipts0[i][0] > ipts0[i+TREND_DISTANCE][0])
        {

            //ʮ��б�ʹյ��ж� ȱ��б��ʮ��
             if((ipts0[i][1] - ipts0[i+ELEMENT_DISTANCE][1] < ELEMENT_THRESHOLD) && (abs(Slope_Calculate(i,i+TREND_DISTANCE,ipts0))<Element_JustStruct.Flat_Slope)&& (Slope_2Points(i-TREND_DISTANCE,i,ipts0)<-Element_JustStruct.Straight_Slope))//����10��б���ж�
            {
                Element0_DataStruct.CD=i;
                Element0_DataStruct.CD_flag=1;
                break;
            }
            //����Բ��,����Բ��
             if( (ipts0[i][1] <= ipts0[i+ELEMENT_DISTANCE][1]) && (ipts0[i+TREND_DISTANCE][1] - ipts0[i][1] >= ELEMENT_THRESHOLD)
                && System_DataStruct.Element_Flag ==ELEMENMT_NONE)//ʹ�þ���ֵ�������ײ�������
            {
                Element0_DataStruct.Round_Prev_L=i;
                System_DataStruct.Element_Flag = LEFT_CIRCLE_IN;//����������Բ��
                System_DataStruct.CIRCLE_Flag = CIRCLE_PREV_D_M;//CIRCLE first phase the fist handle following
                break;
            }

            //ʮ��Y��յ��ж�
                // if( (ipts0[i][1] - ipts0[i+ELEMENT_DISTANCE][1] < ELEMENT_THRESHOLD) && (ipts0[i-ELEMENT_DISTANCE][0]-ipts0[i][0]< ELEMENT_THRESHOLD))//yֵ�ж�
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
*@param:����ʮ���Ϲյ������������ 1:�����¹յ���� 2:�ϴ��Ϲյ����
��̽������Բ��ǰ�йյ����� �����¹յ����
*@author:
*@date:
*/
uint8_t CrossRoad_IN_M=0;
void find_LU_point(image_t * image,uint8_t (*ipts0)[2])
{
    uint8_t translate_x,translate_y;
   /*�õ��¹յ�ƽ��һ�ξ��������*/
    if(Element0_DataStruct.CD_flag==1 ||Element0_DataStruct.CU_adjacent == CROSS_U_EXIST)
    {
        translate_x=ipts0[Element0_DataStruct.CD+ELEMENT_DISTANCE][0];
        translate_y=ipts0[Element0_DataStruct.CD+ELEMENT_DISTANCE][1];//�¹յ�ƽ��һ�ξ���,�����¹յ�һ���ƽ������
    }
    else return;

    //�ҵ��¹յ�,Ȼ��ƫ��һ��,��ൽ�йյ��Ҳ��ƽ���߽�, ��̽�����Ϲյ��ƽ���߽�
    for (;translate_y>3;translate_y--)
    {
          if(Diff_Ratio(TC_IMAGE(image,translate_x,translate_y),TC_IMAGE(image,translate_x,translate_y-1))>Image_DataStruct.Diff_Ratio)
            {
                //�ҵ��Ϲյ�ı߽��,��ʼѲ��
                iptsld_num = sizeof(iptsld) / sizeof(iptsld[0]);
                findline_lefthand_adaptive(image, block_size, clip_value, translate_x, translate_y, iptsld,&iptsld_num);

                //��һ��̽��������
                for (uint8_t m =0;m<iptsld_num-2*ELEMENT_DISTANCE;m++)
                {
/*********************************************ELEMENT BEGIN*********************************************/
                            //ʮ���Ϲյ�
                            if((Element0_DataStruct.CD_flag == 1 || Element0_DataStruct.CU_adjacent == CROSS_U_EXIST) && (abs(Slope_Calculate(m,m+ELEMENT_DISTANCE,iptsld))<Element_JustStruct.Flat_Slope) && (Slope_2Points(m+ELEMENT_DISTANCE,m+2*ELEMENT_DISTANCE,iptsld) < -Element_JustStruct.Straight_Slope))
                            {
                                Element0_DataStruct.CU=m+ELEMENT_DISTANCE;//RU��������ȫ��������ȫ�µ�����,ƴ����������,��Ҫ��RU�����޸�ֵӦ��ΪRD+step,���step������֮������Ӧ�еĵ���
                                Element0_DataStruct.CU_flag=1;//������׶�,���¹յ�ʶ�����,��ô����Ҫ�����¹յ���ʧ����
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
*@param:���Ϲյ����,�¹յ����Ϲյ�ı�Ҫ
* �����Ҫ�����Ϲյ㣬��ô�¹յ��Ƿ��б�Ҫȷ���Ӵ��˱߽磿Ŀǰ��ȡ�Ĳ������¹յ�ƽ�ƽ�����̽����
*@author:C����
*@date:2024-1-20
*/
void find_RU_point(image_t * image,uint8_t (*ipts1)[2])
{
    uint8_t translate_x,translate_y;
    /*�õ��¹յ�ƽ��һ�ξ����x,yֵ*/
    if (Element1_DataStruct.CD_flag == 1 || Element1_DataStruct.CU_adjacent == CROSS_U_EXIST)
    {
        translate_x=ipts1[Element1_DataStruct.CD+ELEMENT_DISTANCE][0];
        translate_y=ipts1[Element1_DataStruct.CD+ELEMENT_DISTANCE][1];
    }
    else return;


    //�ҵ��¹յ�,Ȼ��ƫ��һ��,��ൽ�йյ��Ҳ��ƽ���߽�, ��̽�����Ϲյ��ƽ���߽�
    for (;translate_y>3;translate_y--)
    {    //ʹ�ò�Ⱥͻ�ȡ������?
        if(Diff_Ratio(TC_IMAGE(image,translate_x,translate_y),TC_IMAGE(image,translate_x,translate_y-1))>Image_DataStruct.Diff_Ratio)
            {
                //�ҵ��Ϲյ�ı߽��,��ʼѲ��,���õ����ϱ��ߴ������µı���������
                iptsrd_num = sizeof(iptsrd) / sizeof(iptsrd[0]);
                findline_righthand_adaptive(image, block_size, clip_value, translate_x, translate_y, iptsrd,&iptsrd_num);

            //�����ſ�ʼ�����Ͻǵ�
            //x��С�����������ӵ����ƣ������ж��Ϲյ�
                for (uint8_t m =0;m<iptsrd_num-ELEMENT_DISTANCE;m++)
                {
                        if((iptsrd[m][1] - iptsrd[m+ELEMENT_DISTANCE][1] > ELEMENT_THRESHOLD) )
                        {
                            Element1_DataStruct.CU=m;//RU��������ȫ��������ȫ�µ�����,ƴ����������,��Ҫ��RU�����޸�ֵӦ��ΪRD+step,���step������֮������Ӧ�еĵ���
                            Element1_DataStruct.CU_flag=1;//˵��������׶�,���¹յ㶼ʶ�����
                            break;
                        }
                }
            //��̽����ĳ�ʼλ��Ӧ���ڹյ��Աߵ�ƽ������,����б��һ��С��0.5(abs),�������,��ô�Ͳ���ʮ����
            if(abs(Slope_Calculate(Element1_DataStruct.CU-ELEMENT_DISTANCE,Element1_DataStruct.CU,iptsrd))>0.5){Element1_DataStruct.CU=1;Element1_DataStruct.CU_flag=0;}
                break;
            }
    }
}
//ʮ�����¹յ�Ѱ�����,ƴ�ӿ�ʼ,��ʱcrossflag=2
/*
*@param1:Ԫ�ؽṹ��ָ��
*@param2:Դ��������
*@param3:Դ���߲���
*@param4:�ϱ�������
*@param5:�ϱ��߲���
*@param6: iptscc��Բ��ǰ�Ϲյ�����
*@param7: ��·��������
*@author:C����
*@date:2024-1-27
*/
void Trim_DU(Element_DataTypedef * sptr,uint8_t(*ipts)[2],uint8_t * ipts_num,uint8_t (*iptsc)[2])
{
    if (sptr->CU_flag==1)
    {
    uint8_t cross_step = ipts[sptr->CD][1]-iptsc[sptr->CU][1]-1;//�õ�����֮���ж�����-1�Ǳ�Ҫ��
    uint8_t D_zerofill = POINTS_MAX_LEN-sptr->CD-cross_step-1;//�õ��¹յ����ڵ�old�������¹յ�֮���ڼ���cross_step����� ���ж��ٸ���
    uint8_t U_zerofill = POINTS_MAX_LEN-sptr->CU;//�õ�CU����ĵ���(�����Ҫ����CU)

//!@:������Ϲյ����ĵ���С��zerofill
    if(U_zerofill>=D_zerofill)
        {
        copyArrays(iptsc,ipts,sptr->CU,sptr->CU+D_zerofill-1,sptr->CD+1+cross_step);
        *ipts_num=sptr->CD+cross_step+D_zerofill+1;//�޸Ĳ�����������ǵ���
        }
    else
        {
        copyArrays(iptsc,ipts,sptr->CU,sptr->CU+U_zerofill-1,sptr->CD+1+cross_step);
        *ipts_num=sptr->CD+cross_step+U_zerofill+1;//�޸Ĳ���,������ǵ���
        }

        sptr->CU=sptr->CD+cross_step+1;//��RU���Ž���RD������
    }

}


//��������ͼ��,�������¹յ㼴����ʧ,��Ҫ����Ԥ����Ϣ
/*
*@param:��¼��һ��ͼ����Ϣ,�������־λ
*@author:
*@date:
*/
void Element_Info_Deliver(Element_DataTypedef * sptr)
{
if(sptr->CD_flag==1){sptr->CD_flag=0;}//�Ϸ�ͼ���¹յ����,ˢ��ͼ���־��(CD)
else if(sptr->CD_flag ==0){sptr->CD=1;}

if(sptr->CU_flag==1){sptr->CU_adjacent =CROSS_U_EXIST;sptr->CU_flag=0;}
else if(sptr->CU_flag ==0){sptr->CU_adjacent =CROSS_U_DISMISS;sptr->CU=0;}

}

