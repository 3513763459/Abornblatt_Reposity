#include "zf_common_headfile.h"
#include "Camera.h"
#include "Car.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    Cpu0_Init();
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
while (TRUE)
{

    if(mt9v03x_finish_flag)
    {
        img_raw.data=Gray_Image[0];
        ImageProcess(&img_raw);
        //mt9v03x_finish_flag = 0;
        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        for (uint8_t i = 0; i < 120; ++i){x0_boundry[i] = ipts0[i][0];}//�������ǵ�x,y������ipts[��ĸ���][0����x]  ipts[][1����y]
        for (uint8_t i = 0; i < 120; ++i){y0_boundry[i] = ipts0[i][1];}//���������� ipts[0][0]~ipts[0][119]�������x�ĵ�ַ����x_boundry[0]~x_boundry[119]

        for (uint8_t i = 0; i < 120; ++i){x1_boundry[i] = ipts1[i][0];}//��Ҫר�Ž��а���,�����½�x_boundry[��ĸ���] ָ������
        for (uint8_t i = 0; i < 120; ++i){y1_boundry[i] = ipts1[i][1];}//�����Ļ��ʹ�������õ�ӳ��,��x_boundry[0]���������Ԫ�ص�ַ ����seekfree_assistant_camera_boundary_config
        for (uint8_t i = 0; i < 120; ++i){rptsx_boundry[i] = rpts[i][0];}//��Ҫר�Ž��а���,�����½�x_boundry[��ĸ���] ָ������
        for (uint8_t i = 0; i < 120; ++i){rptsy_boundry[i] = rpts[i][1];}//�����Ļ��ʹ�������õ�ӳ��,��x_boundry[0]���������Ԫ�ص�ַ ����seekfree_assistant_camera_boundary_config


        seekfree_assistant_camera_send();

    }
}
}

#pragma section all restore

// **************************** �������� ****************************
