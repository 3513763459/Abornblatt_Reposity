/*
 * Vofa.c
 *上位机通讯文件
 *
 *  Created on: 2023年12月11日
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
        float tempFloat[5];   //定义的临时变量
        uint8 tempData[24];   //定义的传输Buffer，一个int占4个字节，如果传5个，4*5=20，后面还有四个校验位，20+4=24
        //速度环speed_type
        data[0] = System_DataStruct.SpeedDataStruct.Speed_L;   //int16占2个字节
        data[1] = System_DataStruct.SpeedDataStruct.Speed_R;   //int16占2个字节
        data[2] = System_DataStruct.SpeedDataStruct.exceptspeed;   //int16占2个字节
        data[3] = 0;    //int16占2个字节
        data[4] = 0;    //int16占2个字节


        tempFloat[0] = (float)data[0];     //转成浮点数
        tempFloat[1] = (float)data[1];
        tempFloat[2] = (float)data[2];
        tempFloat[3] = (float)data[3];
        tempFloat[4] = (float)data[4];

        memcpy(tempData, (uint8 *)tempFloat, sizeof(tempFloat));  //通过拷贝把数据重新整理

        tempData[20] = 0x00;//写入结尾数据
        tempData[21] = 0x00;
        tempData[22] = 0x80;
        tempData[23] = 0x7f;

        wireless_uart_send_buffer(tempData, 24);//通过串口传输数据，前面开多大的数组，后面占多大长度

}
