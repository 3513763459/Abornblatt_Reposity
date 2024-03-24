/*
 * Car.c
 *
 *  Created on: 2023年12月9日
 *      Author: LINEAR
 */
#include "Car.h"
#include "Camera.h"
#include "DataStore.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"

/*******************************************debug**********************************************************/
uint8 image_copy[MT9V03X_H][MT9V03X_W];
//uint8 *x0_boundry[MT9V03X_H],*x1_boundry[MT9V03X_H],*y0_boundry[MT9V03X_H],*y1_boundry[MT9V03X_H];
//float *rptsx_boundry[MT9V03X_H],*rptsy_boundry[MT9V03X_H];
uint8 x0_boundry[MT9V03X_H],x1_boundry[MT9V03X_H],y0_boundry[MT9V03X_H],y1_boundry[MT9V03X_H];
float rptsx_boundry[MT9V03X_H],rptsy_boundry[MT9V03X_H];
/*******************************************debug**********************************************************/

void Cpu1_Init(void)
{

}

void Cpu0_Init(void)
{
//    //数据初始化
    Data_Settings(); //请首先初始化各类变量,因为摄像头硬件曝光需要
//    Flash_read();

    //初始化摄像头
     mt9v03x_init();
   //  mt9v03x_set_exposure_time(Image_DataStruct.Camera_Exp);
//     //舵机初始化
//     Steer_Init();
//     //编码器初始化
//     Encoder_Init();
//     //电机初始化
//     Motor_Init();//中断由cpu0触发服务,所以函数代码也放在了cpu0
//    //按键初始化
//    key_init(5);

    //陀螺仪dmp初始化
   // mpu_dmp_init();//是有返回值的,0代表成功
/**********************************************debug层****************************************************/

//     for (uint8_t i = 0; i < 120; ++i){x0_boundry[i] = &ipts0[i][0];}//由于我们的x,y都是用ipts[点的个数][0代表x]  ipts[][1代表y]
//     for (uint8_t i = 0; i < 120; ++i){x1_boundry[i] = &ipts1[i][0];}//需要专门进行剥离,所以新建x_boundry[点的个数] 指针数组
//     for (uint8_t i = 0; i < 120; ++i){y0_boundry[i] = &ipts0[i][1];}//将边线数组 ipts[0][0]~ipts[0][119]所代表的x的地址传进x_boundry[0]~x_boundry[119]
//     for (uint8_t i = 0; i < 120; ++i){y1_boundry[i] = &ipts1[i][1];}//这样的化就达成了永久的映射,将x_boundry[0]所代表的首元素地址 传进seekfree_assistant_camera_boundary_config

#define WIFI_SSID_TEST          "chen_wifi"
#define WIFI_PASSWORD_TEST      "12345678"  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL

wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST);
// 逐飞助手初始化 数据传输使用高速WIFI SPI
seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
// 发送总钻风图像信息(仅包含原始图像信息)
seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
//seekfree_assistant_camera_boundary_config(XY_BOUNDARY, 480, *x0_boundry, *x1_boundry, NULL, *y0_boundry, *y1_boundry, NULL);
seekfree_assistant_camera_boundary_config(XY_BOUNDARY,120, x0_boundry,  x1_boundry, NULL,y0_boundry, y1_boundry,NULL);

/**********************************************debug层****************************************************/

    //中断初始化
//    pit_ms_init(CCU60_CH0, STEER_PERIOD_MS);//舵机,编码器中断
//    pit_ms_init(CCU60_CH1, SPEED_PERIOD_MS);//电机中断
//    pit_ms_init(CCU61_CH0, GYRO_PERIOD_MS);//陀螺仪中断
}

