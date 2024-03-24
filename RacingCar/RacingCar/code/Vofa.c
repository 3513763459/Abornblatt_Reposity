/*
 * Vofa.c
 *��λ��ͨѶ�ļ�
 *
 *  Created on: 2023��12��11��
 *      Author: LINEAR
 */

#include "Vofa.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include "System.h"
#include "zf_device_wireless_uart.h"
#include "Camera.h"

void ips200_showbondary(uint8_t p0[][2],uint8_t num0,uint8_t p1[][2],uint8_t num1)
{
    for(uint8_t i=0;i<num0;i++)
    {

        ips200_draw_point(p0[i][0],p0[i][1],RGB565_RED);
        ips200_draw_point(p1[i][0],p1[i][1],RGB565_RED);


    }
}

void ips200_showfbondary(float p[][2],uint8_t num)
{
    for(uint8_t i=0;i<num;i++)
    {
        if(i>=Image_DataStruct.begin_err && i<=Image_DataStruct.begin_err+10)
        {
            ips200_draw_point((uint8)p[i][0],(uint8)p[i][1],RGB565_GREEN);
        }
        else
        {
            ips200_draw_point((uint8)p[i][0],(uint8)p[i][1],RGB565_RED);
        }

    }
}


void Vofa_datasend(void)
{
       int data[5];
        float tempFloat[5];   //�������ʱ����
        uint8 tempData[24];   //����Ĵ���Buffer��һ��intռ4���ֽڣ������5����4*5=20�����滹���ĸ�У��λ��20+4=24
        //�ٶȻ�speed_type
        data[0] = System_DataStruct.SpeedDataStruct.Speed_L;   //int16ռ2���ֽ�
        data[1] = System_DataStruct.SpeedDataStruct.Speed_R;   //int16ռ2���ֽ�
        data[2] = System_DataStruct.SpeedDataStruct.exceptspeed;   //int16ռ2���ֽ�
        data[3] = 0;    //int16ռ2���ֽ�
        data[4] = 0;    //int16ռ2���ֽ�


        tempFloat[0] = (float)data[0];     //ת�ɸ�����
        tempFloat[1] = (float)data[1];
        tempFloat[2] = (float)data[2];
        tempFloat[3] = (float)data[3];
        tempFloat[4] = (float)data[4];

        memcpy(tempData, (uint8 *)tempFloat, sizeof(tempFloat));  //ͨ��������������������

        tempData[20] = 0x00;//д���β����
        tempData[21] = 0x00;
        tempData[22] = 0x80;
        tempData[23] = 0x7f;

        wireless_uart_send_buffer(tempData, 24);//ͨ�����ڴ������ݣ�ǰ�濪�������飬����ռ��󳤶�

}
