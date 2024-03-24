/*
 * Flash_Interface.c
 *
 *  Created on: 2024年1月10日
 *      Author: LINEAR
 *  在上电的时候,调参变量从ROM拿值,此时ROM为0(已经被擦除)
    在下载程序,第一次打开单片机,需要在一级菜单先选择 Flash_Init,将RAM的值在给到变量
    请注意,如果你在调参过程中,改变了变量的值,并且Flash_Save,那么在上电后请不要Flash_Init
 */

#include "Flash_Interface.h"
#include "zf_device_ips200.h"
#include "zf_driver_flash.h"
#include "forward.h"
#include "Motor.h"
#include "PID.h"

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (8)                                // 存储数据用的页码 倒数第一个页码



void Flash_save(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))     // 63 0                 // 判断是否有数据
    {
       flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);              // 擦除这一页
    }
    flash_buffer_clear();

    flash_union_buffer[0].float_type = forward_steer_P;
    flash_union_buffer[1].float_type = forward_steer_D;

    flash_union_buffer[2].float_type = forward_mid_P;
    flash_union_buffer[3].float_type = forward_mid_D;

    flash_union_buffer[4].float_type = speed_standard_set;


    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
}
void Flash_read(void)
{
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区

    forward_steer_P=flash_union_buffer[0].float_type ;
    forward_steer_D=flash_union_buffer[1].float_type ;

    forward_mid_P=flash_union_buffer[2].float_type ;
    forward_mid_D=flash_union_buffer[3].float_type;

    speed_standard_set=flash_union_buffer[4].float_type ;
}
void Flash_read_Init(void)
{
    forward_steer_P=6 ;
    forward_steer_D=7 ;

    forward_mid_P=0.060;
    forward_mid_D=0.050;

    speed_standard_set=3.0;

}

//以下为一级菜单=========================================================
unsigned char ui_page0[5][17] =//主页
{
    " nextPage ",
    " STEER_PID",//ui_ServoPid2    舵机动态PD二次型
    " MID_PID",//ui_ServoPid         PID控制
    " SPEED_PID",//ui_Speed       速度控制
    " Flash_Init" //将此时程序RAM的值写到ROM,需要重启
};
unsigned char ui_page1[5][17] =//主页
{
    " nextPage ",
    " DATA_INIT",//ui_ServoPid2    舵机动态PD二次型
    "  ",//ui_ServoPid         PID控制
    "  ",//ui_Speed       速度控制
    " Flash_Init" //将此时程序RAM的值写到ROM,需要重启
};


//以下为二级菜单=========================================================

unsigned char UI_STEER_PID[5][17] =// 二级菜单,舵机界面
{
    " KP0.1+ ",
    " KP0.1-",
    " KD0.1+",
    " KD0.1-",
    " Flash_Save"
};

unsigned char UI_MID_PID[5][17] =// 二级菜单,中值
{
     " KP0.1+ ",
     " KP0.1-",
     " KD0.1+",
     " KD0.1-",
     " Flash_Save"
};
unsigned char UI_SPEED[5][17] =// 二级菜单,速度
{
     " SPEED0.1+ ",
     " SPEED0.1-",
     "  ",
     "  ",
     " Flash_Save"
};



unsigned char ui_Flash[5][17] =// 二级菜单,Flash初始画界面
{
    " are you sure", //
    " Yes"
};

//-------------------------------------------------------------

Interface_Class ui ={
    {0, 0, 0, 0},
    0,  //page
    -1  //enter
};


void interface_display_strings(uint8 strings[10][17])
{
    uint8 i;
    for (i = 0; i < 5; ++i) // i代表表单行数,5行
    {
        if (i == ui.cursor[ui.page])
            {
                strings[i][0] = '>';// 如果这一行是被选中的要进行操作的一行,每一个页面有10行,10个选项
            }
        else
            {
                strings[i][0] = '.';// 没有选中就正常显示 .
            }

        ips200_show_string(0, 220+20*i,(char *)strings[i]);// 强制类型转换将无符号类型转换为有符号类型
    }
}

void interface_display(Interface_Class * UiPointer)
{
  switch(UiPointer->page) // 0~3 一共四页
  {
    case 0: //home页
                switch(UiPointer->enter)// 已经进入二级菜单(Key_Function中),ui.enter已经被赋值为二级菜单在一级菜单的项数,每一页的选项对应相应的菜单
                {
                    case -1: // 没有选择二级菜单，显示第一页 home页
                        interface_display_strings(ui_page0);
                        break;
                    case 0:// 下一页,同时清除enter,不选择二级菜单
                        UiPointer->page++;
                        UiPointer->enter=-1;
                        break;
                    case 1://二级菜单
                        interface_display_strings(UI_STEER_PID);//舵机
                        ips200_show_float(80, 220, forward_steer_P, 2, 3);
                        ips200_show_float(80, 260, forward_steer_D, 2, 3);
                        //使用后还需要初始化一下
                        break;
                    case 2: //中心点
                        interface_display_strings(UI_MID_PID);
                        ips200_show_float(80, 220, forward_mid_P, 2, 4);
                        ips200_show_float(80, 260, forward_mid_D, 2, 4);
                        break;
                    case 3: //编码器速度
                        interface_display_strings(UI_SPEED);
                        ips200_show_float(120, 220, speed_standard_set, 2, 2);

                        break;
                    case 4:
                        interface_display_strings(ui_Flash);
                        break;
                    default:
                        break;
                }
        break;
    case 1:
                    switch(UiPointer->enter)// 已经进入二级菜单(Key_Function中),ui.enter已经被赋值为二级菜单在一级菜单的项数,每一页的选项对应相应的菜单
                    {
                        case -1: // 没有选择二级菜单，显示第一页 1页
                            interface_display_strings(ui_page1);
                            break;
                        case 0:// 下一页,同时清除enter,不选择二级菜单
                            UiPointer->page++;
                            UiPointer->enter=-1;
                            break;
                        case 1:
                            interface_display_strings(ui_Flash);//借用

                            break;
                        case 2: //舵机PID二级界面
                          //  interface_display_strings(ui_gyro1);

                            break;
                        case 3: //速度界面

                            break;
                        case 4:

                            break;
                        default:
                            break;
                    }
        break;
    case 2:
        break;
    case 3:
        break;
    default:
        break;

  }
}

