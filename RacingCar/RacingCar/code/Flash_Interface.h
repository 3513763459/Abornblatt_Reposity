/*
 * Flash_Interface.h
 *
 *  Created on: 2024��1��10��
 *      Author: LINEAR
 */

#ifndef CODE_FLASH_INTERFACE_H_
#define CODE_FLASH_INTERFACE_H_
#include "Platform_Types.h"
extern unsigned char ui_SAVE[5][17];
extern unsigned char ui_UnSave[5][17];

typedef struct
{
    int  cursor[4],//ui.cursor[ui.page]=0~9 ÿһ��ҳ����10��ѡ�ui.cursor[ui.page]��ʾÿһ��ҳ��ĵڼ���ѡ��
    page,
    enter; // �������ƶ����˵�
} Interface_Class;
extern Interface_Class ui;

/*void interface_init(Interface_Class*);*/
void interface_display(Interface_Class *);
void interface_display_strings(uint8 strings[5][17]);//����Ϊҳ����ʾ�Ĳ������� 17��ÿ���������Ŀ��,10����ÿһҳ��10���ɵ�����
void Flash_save(void);
void Flash_read(void);
void Flash_read_Init(void);

#endif /* CODE_FLASH_INTERFACE_H_ */
