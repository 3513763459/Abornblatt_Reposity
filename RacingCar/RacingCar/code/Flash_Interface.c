/*
 * Flash_Interface.c
 *
 *  Created on: 2024��1��10��
 *      Author: LINEAR
 *  ���ϵ��ʱ��,���α�����ROM��ֵ,��ʱROMΪ0(�Ѿ�������)
    �����س���,��һ�δ򿪵�Ƭ��,��Ҫ��һ���˵���ѡ�� Flash_Init,��RAM��ֵ�ڸ�������
    ��ע��,������ڵ��ι�����,�ı��˱�����ֵ,����Flash_Save,��ô���ϵ���벻ҪFlash_Init
 */

#include "Flash_Interface.h"
#include "zf_device_ips200.h"
#include "zf_driver_flash.h"
#include "forward.h"
#include "Motor.h"
#include "PID.h"

#define FLASH_SECTION_INDEX       (0)                                 // �洢�����õ�����
#define FLASH_PAGE_INDEX          (8)                                // �洢�����õ�ҳ�� ������һ��ҳ��



void Flash_save(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))     // 63 0                 // �ж��Ƿ�������
    {
       flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);              // ������һҳ
    }
    flash_buffer_clear();

    flash_union_buffer[0].float_type = forward_steer_P;
    flash_union_buffer[1].float_type = forward_steer_D;

    flash_union_buffer[2].float_type = forward_mid_P;
    flash_union_buffer[3].float_type = forward_mid_D;

    flash_union_buffer[4].float_type = speed_standard_set;


    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // ��ָ�� Flash ������ҳ��д�뻺��������
}
void Flash_read(void)
{
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // �����ݴ� flash ��ȡ��������

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

//����Ϊһ���˵�=========================================================
unsigned char ui_page0[5][17] =//��ҳ
{
    " nextPage ",
    " STEER_PID",//ui_ServoPid2    �����̬PD������
    " MID_PID",//ui_ServoPid         PID����
    " SPEED_PID",//ui_Speed       �ٶȿ���
    " Flash_Init" //����ʱ����RAM��ֵд��ROM,��Ҫ����
};
unsigned char ui_page1[5][17] =//��ҳ
{
    " nextPage ",
    " DATA_INIT",//ui_ServoPid2    �����̬PD������
    "  ",//ui_ServoPid         PID����
    "  ",//ui_Speed       �ٶȿ���
    " Flash_Init" //����ʱ����RAM��ֵд��ROM,��Ҫ����
};


//����Ϊ�����˵�=========================================================

unsigned char UI_STEER_PID[5][17] =// �����˵�,�������
{
    " KP0.1+ ",
    " KP0.1-",
    " KD0.1+",
    " KD0.1-",
    " Flash_Save"
};

unsigned char UI_MID_PID[5][17] =// �����˵�,��ֵ
{
     " KP0.1+ ",
     " KP0.1-",
     " KD0.1+",
     " KD0.1-",
     " Flash_Save"
};
unsigned char UI_SPEED[5][17] =// �����˵�,�ٶ�
{
     " SPEED0.1+ ",
     " SPEED0.1-",
     "  ",
     "  ",
     " Flash_Save"
};



unsigned char ui_Flash[5][17] =// �����˵�,Flash��ʼ������
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
    for (i = 0; i < 5; ++i) // i���������,5��
    {
        if (i == ui.cursor[ui.page])
            {
                strings[i][0] = '>';// �����һ���Ǳ�ѡ�е�Ҫ���в�����һ��,ÿһ��ҳ����10��,10��ѡ��
            }
        else
            {
                strings[i][0] = '.';// û��ѡ�о�������ʾ .
            }

        ips200_show_string(0, 220+20*i,(char *)strings[i]);// ǿ������ת�����޷�������ת��Ϊ�з�������
    }
}

void interface_display(Interface_Class * UiPointer)
{
  switch(UiPointer->page) // 0~3 һ����ҳ
  {
    case 0: //homeҳ
                switch(UiPointer->enter)// �Ѿ���������˵�(Key_Function��),ui.enter�Ѿ�����ֵΪ�����˵���һ���˵�������,ÿһҳ��ѡ���Ӧ��Ӧ�Ĳ˵�
                {
                    case -1: // û��ѡ������˵�����ʾ��һҳ homeҳ
                        interface_display_strings(ui_page0);
                        break;
                    case 0:// ��һҳ,ͬʱ���enter,��ѡ������˵�
                        UiPointer->page++;
                        UiPointer->enter=-1;
                        break;
                    case 1://�����˵�
                        interface_display_strings(UI_STEER_PID);//���
                        ips200_show_float(80, 220, forward_steer_P, 2, 3);
                        ips200_show_float(80, 260, forward_steer_D, 2, 3);
                        //ʹ�ú���Ҫ��ʼ��һ��
                        break;
                    case 2: //���ĵ�
                        interface_display_strings(UI_MID_PID);
                        ips200_show_float(80, 220, forward_mid_P, 2, 4);
                        ips200_show_float(80, 260, forward_mid_D, 2, 4);
                        break;
                    case 3: //�������ٶ�
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
                    switch(UiPointer->enter)// �Ѿ���������˵�(Key_Function��),ui.enter�Ѿ�����ֵΪ�����˵���һ���˵�������,ÿһҳ��ѡ���Ӧ��Ӧ�Ĳ˵�
                    {
                        case -1: // û��ѡ������˵�����ʾ��һҳ 1ҳ
                            interface_display_strings(ui_page1);
                            break;
                        case 0:// ��һҳ,ͬʱ���enter,��ѡ������˵�
                            UiPointer->page++;
                            UiPointer->enter=-1;
                            break;
                        case 1:
                            interface_display_strings(ui_Flash);//����

                            break;
                        case 2: //���PID��������
                          //  interface_display_strings(ui_gyro1);

                            break;
                        case 3: //�ٶȽ���

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

