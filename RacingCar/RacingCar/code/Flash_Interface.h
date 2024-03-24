/*
 * Flash_Interface.h
 *
 *  Created on: 2024年1月10日
 *      Author: LINEAR
 */

#ifndef CODE_FLASH_INTERFACE_H_
#define CODE_FLASH_INTERFACE_H_
#include "Platform_Types.h"
extern unsigned char ui_SAVE[5][17];
extern unsigned char ui_UnSave[5][17];

typedef struct
{
    int  cursor[4],//ui.cursor[ui.page]=0~9 每一个页面有10个选项，ui.cursor[ui.page]表示每一个页面的第几个选项
    page,
    enter; // 用来控制二级菜单
} Interface_Class;
extern Interface_Class ui;

/*void interface_init(Interface_Class*);*/
void interface_display(Interface_Class *);
void interface_display_strings(uint8 strings[5][17]);//参数为页面显示的参数数组 17是每个参数最大的宽度,10代表每一页有10个可调参数
void Flash_save(void);
void Flash_read(void);
void Flash_read_Init(void);

#endif /* CODE_FLASH_INTERFACE_H_ */
