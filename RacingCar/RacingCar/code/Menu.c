/*
 * Menu.c
 *
 *  Created on: 2023年12月18日
 *      Author: LINEAR
 *界面
按键读取函数放在5ms中断,在main文件存放显示界面函数,这是整体思想
ui.page表示在第几页 ui.cursor[ui.page]表示在第几行 ui.enter只有在进入二级菜单的情况下才会被赋值,在一级菜单ui.enter=-1
ui.cursor[ui.page]=0~9 每一个页面有10个选项，ui.cursor[ui.page]表示每一个页面的第几个选项
S3 在一级菜单第一行执行功能上一页，            在一级菜单的非一行不执行功能                              在二级菜单执行功能返回上一级菜单
S4 在一级菜单第一行执行功能下一页，            在一级菜单的非一行执行进入对应二级菜单            在二级菜单执行功能 执行二级菜单提示任务(加减等)
S5 短按上一行(每页有好几行)
S6 短按下一行
 */
#include "Menu.h"
#include "zf_driver_gpio.h"
#include "zf_device_key.h"
#include "Flash_Interface.h"
#include "PID.h"
#include "zf_device_ips200.h"
#include "DataStore.h"
#include "Motor.h"

#define  S3 P20_6
#define  S4 P20_7
#define  S5 P11_2
#define  S6 P11_3
key_index_enum key_index_array[KEY_NUMBER] = {KEY_1,KEY_2,KEY_3,KEY_4};

int whichKey=-1;
int loop=0;// 循环变量,0:KEY_1 1:KEY_2 3:KEY_4

// 按键状态读取,下列是短按状态后的赋值
// 0 S3
// 1 S4
// 2 S5
// 3 S6
void KeyRead(void)
{
    key_scanner();

    // 在四个按键中循环
    for(loop=0;loop<KEY_NUMBER;loop++)
    {
        switch(key_get_state(key_index_array[loop]))//获取按键状态
        {
        case KEY_SHORT_PRESS://短按
                whichKey=loop;
            key_clear_state(key_index_array[loop]);//清除对应按键的状态
            break;
        default://按键释放状态
            break;
        }
    }
}

void KeyFunction(void)
{
    switch(whichKey)
    {
        case 2: //S5短按 上一个,每一个页面有10个选项
            ui.cursor[ui.page]--; //页面行数--
            if(ui.cursor[ui.page] < 0)
            {
                ui.cursor[ui.page]=4;//减到头就重置
            }
            whichKey=-1;//刷新键值
            break;
        case 3: // S6 短按 下一个
            ui.cursor[ui.page]++;
            if(ui.cursor[ui.page] > 4)
            {
                ui.cursor[ui.page] = 0;
            }
            whichKey=-1;
            break;
/*
 * 在菜单的第一行，执行功能下一页，不管是第一级菜单还是第二级菜单
 * 在一级菜单的其余行，执行功能下一级菜单
 *在二级菜单的其余行，执行功能 */
        case 1:   //S4的短按功能 下一页 非一行执行功能进入对应二级菜单
            if(ui.cursor[ui.page] == 0  && ui.enter == -1) //检测所在行数,
            {
                ips200_clear();
                ui.page++;
                if(ui.page > 3)
                {
                    ui.page = 0;
                }
                ui.cursor[ui.page] = 0;
            }
            else
            {
                    switch(ui.page) // 四个一级菜单, 一级菜单的其余行执行二级菜单的任务
                            {
                                case 0:// 一级菜单 第一页,注意一旦进入二级菜单，唯一与一级菜单有区别的就是ui.enter!=-1,在一级菜单执行进入二级菜单命令，就要确保当前是在一级菜单
                                    if(ui.cursor[ui.page]>0 && ui.enter==-1){ui.enter = ui.cursor[ui.page];ui.cursor[ui.page]=0;ips200_clear();}  // 进入对应二级菜单后,默认在第一行
                                    else //处在二级菜单当中，执行二级菜单对应的任务
                                    {      switch(ui.enter) //搜寻是在第几行的二级菜单
                                            {
                                                case 1: //第一页，第二行的二级菜单，单名 steer
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 0://执行二级菜单提示的对应功能
                                                                    forward_steer_P+=0.02;
                                                                    break;
                                                                case 1:
                                                                    forward_steer_P-=0.02;
                                                                    break;
                                                                case 2:
                                                                    forward_steer_D+=0.02;
                                                                    break;
                                                                case 3:
                                                                    forward_steer_D-=0.02;
                                                                    break;

                                                                case 4:
                                                                    Flash_save();
                                                                    forward_Init();
                                                                    ips200_show_string(120, 300, "ok");
                                                                    break;

                                                                default:
                                                                    break;
                                                            }
                                                    break;
                                                case 2: // 第一页 第三行的二级菜单，单名PID
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 0://执行二级菜单提示的对应功能
                                                                    forward_mid_P+=0.002;
                                                                    break;
                                                                case 1:
                                                                    forward_mid_P-=0.002;
                                                                    break;
                                                                case 2:
                                                                    forward_mid_D+=0.002;
                                                                    break;
                                                                case 3:
                                                                    forward_mid_D-=0.002;
                                                                    break;

                                                                case 4:
                                                                  Flash_save();
                                                                  forward_Init();
                                                                  ips200_show_string(120, 300, "ok");
                                                                  break;

                                                                default:
                                                                    break;
                                                            }
                                                    break;
                                                case 3:// 第一页 第四行的二级菜单，单名SPEED
                                                        switch(ui.cursor[ui.page])
                                                        {
                                                            case 0://执行二级菜单提示的对应功能
                                                                speed_standard_set+=0.1;
                                                                break;
                                                            case 1:
                                                                speed_standard_set-=0.1;
                                                                break;

                                                            case 4:
                                                            Flash_save();
                                                            forward_Init();
                                                            ips200_show_string(120, 300, "ok");
                                                            break;

                                                            default:
                                                                break;
                                                        }
                                                    break;
                                                    case 4:
                                                        switch(ui.cursor[ui.page])
                                                        {

                                                            case 1://"Yes"
                                                                Data_Settings();
                                                                Flash_read();
                                                                forward_Init();
                                                                ips200_show_string(120, 300, "ok");
                                                                break;
                                                            default:
                                                                break;
                                                        }
                                                        break;
                                                default:
                                                    break;

                                            }

                                    }
                                    break;

                                case 1: //一级菜单第二页
                                    if(ui.cursor[ui.page]>0 && ui.enter==-1){ui.enter = ui.cursor[ui.page];ui.cursor[ui.page]=0;ips200_clear();}  // 进入对应二级菜单后,默认在第一行
                                    else //处在二级菜单当中，执行二级菜单对应的任务
                                    {      switch(ui.enter) //搜寻是在第几行的二级菜单
                                            {
                                                case 1: //第2页，第二行的二级菜单，单名 Element
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 1://"Yes"
                                                                    Flash_read_Init();
                                                                    forward_Init();
                                                                    ips200_show_string(120, 300, "ok");
                                                                    break;
                                                                default:
                                                                    break;
                                                            }
                                                    break;
                                                case 2: // 第2页 第三行的二级菜单，单名PID
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 0://执行二级菜单提示的对应功能

                                                                    break;
                                                                case 1:

                                                                    break;
                                                                case 2:

                                                                    break;
                                                                case 3:

                                                                    break;
                                                                default:
                                                                    break;
                                                            }
                                                    break;
                                                case 3:// 第2页 第四行的二级菜单，单名
                                                        switch(ui.cursor[ui.page])
                                                        {
                                                            case 0://执行二级菜单提示的对应功能
                                                                break;
                                                            case 1:
                                                                break;
                                                            default:
                                                                break;
                                                        }
                                                    break;
                                                default:
                                                    break;

                                            }

                                    }
                                    break;
                                case 2: //一级菜单第三页
                                    break;
                                case 3: //一级菜单第四页
                                    break;
                                default:
                                    break;

                            }
            }
            whichKey=-1;
            break;
        case 0://S3短按 一级菜单首行执行上一页,二级菜单执行返回到对应一级菜单
            if(ui.cursor[ui.page] == 0 && ui.enter == -1)// 意思是如果光标在第一行>nextpage这一行才执行下一页的命令
            {
                ips200_clear();
                ui.page--;
                if(ui.page < 0)
                {
                    ui.page = 3;
                }
                ui.cursor[ui.page] = 0;
            }
            else
            {
                 switch(ui.page)// 搜寻进入二级菜单对应的一级菜单的页数，因为在进入一级菜单的时候，ui.page没有清零
                 {
                    case 0: // 表示是在第一页进入的二级菜单，现在在第一页一级菜单进行搜寻是在哪一行进入的二级菜单
                        if(ui.enter>0){ui.enter=-1;ips200_clear();};
                        break;
                    case 1:
                        if(ui.enter>0){ui.enter=-1;ips200_clear();};
                        break;
                    case 2:// 表示是在第三页进入的二级菜单，现在在第三页一级菜单进行搜寻是在哪一行进入的二级菜单
                        if(ui.enter>0){ui.enter=-1;ips200_clear();};
                        break;
                    case 3:
                        if(ui.enter>0){ui.enter=-1;ips200_clear();};
                        break;
                    default:
                        break;
                 }
            }
            whichKey=-1;
            break;
        default:
            break;
    }
}
