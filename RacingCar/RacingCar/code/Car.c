/*
 * Car.c
 *
 *  Created on: 2023��12��9��
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
//    //���ݳ�ʼ��
    Data_Settings(); //�����ȳ�ʼ���������,��Ϊ����ͷӲ���ع���Ҫ
//    Flash_read();

    //��ʼ������ͷ
     mt9v03x_init();
   //  mt9v03x_set_exposure_time(Image_DataStruct.Camera_Exp);
//     //�����ʼ��
//     Steer_Init();
//     //��������ʼ��
//     Encoder_Init();
//     //�����ʼ��
//     Motor_Init();//�ж���cpu0��������,���Ժ�������Ҳ������cpu0
//    //������ʼ��
//    key_init(5);

    //������dmp��ʼ��
   // mpu_dmp_init();//���з���ֵ��,0����ɹ�
/**********************************************debug��****************************************************/

//     for (uint8_t i = 0; i < 120; ++i){x0_boundry[i] = &ipts0[i][0];}//�������ǵ�x,y������ipts[��ĸ���][0����x]  ipts[][1����y]
//     for (uint8_t i = 0; i < 120; ++i){x1_boundry[i] = &ipts1[i][0];}//��Ҫר�Ž��а���,�����½�x_boundry[��ĸ���] ָ������
//     for (uint8_t i = 0; i < 120; ++i){y0_boundry[i] = &ipts0[i][1];}//���������� ipts[0][0]~ipts[0][119]�������x�ĵ�ַ����x_boundry[0]~x_boundry[119]
//     for (uint8_t i = 0; i < 120; ++i){y1_boundry[i] = &ipts1[i][1];}//�����Ļ��ʹ�������õ�ӳ��,��x_boundry[0]���������Ԫ�ص�ַ ����seekfree_assistant_camera_boundary_config

#define WIFI_SSID_TEST          "chen_wifi"
#define WIFI_PASSWORD_TEST      "12345678"  // �����Ҫ���ӵ�WIFI û����������Ҫ�� ���� �滻Ϊ NULL

wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST);
// ������ֳ�ʼ�� ���ݴ���ʹ�ø���WIFI SPI
seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
// ���������ͼ����Ϣ(������ԭʼͼ����Ϣ)
seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
//seekfree_assistant_camera_boundary_config(XY_BOUNDARY, 480, *x0_boundry, *x1_boundry, NULL, *y0_boundry, *y1_boundry, NULL);
seekfree_assistant_camera_boundary_config(XY_BOUNDARY,120, x0_boundry,  x1_boundry, NULL,y0_boundry, y1_boundry,NULL);

/**********************************************debug��****************************************************/

    //�жϳ�ʼ��
//    pit_ms_init(CCU60_CH0, STEER_PERIOD_MS);//���,�������ж�
//    pit_ms_init(CCU60_CH1, SPEED_PERIOD_MS);//����ж�
//    pit_ms_init(CCU61_CH0, GYRO_PERIOD_MS);//�������ж�
}

