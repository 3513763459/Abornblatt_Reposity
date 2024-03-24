#include "zf_common_headfile.h"
#include "Camera.h"
#include "Car.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    Cpu0_Init();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
while (TRUE)
{

    if(mt9v03x_finish_flag)
    {
        img_raw.data=Gray_Image[0];
        ImageProcess(&img_raw);
        //mt9v03x_finish_flag = 0;
        memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
        for (uint8_t i = 0; i < 120; ++i){x0_boundry[i] = ipts0[i][0];}//由于我们的x,y都是用ipts[点的个数][0代表x]  ipts[][1代表y]
        for (uint8_t i = 0; i < 120; ++i){y0_boundry[i] = ipts0[i][1];}//将边线数组 ipts[0][0]~ipts[0][119]所代表的x的地址传进x_boundry[0]~x_boundry[119]

        for (uint8_t i = 0; i < 120; ++i){x1_boundry[i] = ipts1[i][0];}//需要专门进行剥离,所以新建x_boundry[点的个数] 指针数组
        for (uint8_t i = 0; i < 120; ++i){y1_boundry[i] = ipts1[i][1];}//这样的化就达成了永久的映射,将x_boundry[0]所代表的首元素地址 传进seekfree_assistant_camera_boundary_config
        for (uint8_t i = 0; i < 120; ++i){rptsx_boundry[i] = rpts[i][0];}//需要专门进行剥离,所以新建x_boundry[点的个数] 指针数组
        for (uint8_t i = 0; i < 120; ++i){rptsy_boundry[i] = rpts[i][1];}//这样的化就达成了永久的映射,将x_boundry[0]所代表的首元素地址 传进seekfree_assistant_camera_boundary_config


        seekfree_assistant_camera_send();

    }
}
}

#pragma section all restore

// **************************** 代码区域 ****************************
