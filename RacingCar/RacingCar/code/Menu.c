/*
 * Menu.c
 *
 *  Created on: 2023��12��18��
 *      Author: LINEAR
 *����
������ȡ��������5ms�ж�,��main�ļ������ʾ���溯��,��������˼��
ui.page��ʾ�ڵڼ�ҳ ui.cursor[ui.page]��ʾ�ڵڼ��� ui.enterֻ���ڽ�������˵�������²Żᱻ��ֵ,��һ���˵�ui.enter=-1
ui.cursor[ui.page]=0~9 ÿһ��ҳ����10��ѡ�ui.cursor[ui.page]��ʾÿһ��ҳ��ĵڼ���ѡ��
S3 ��һ���˵���һ��ִ�й�����һҳ��            ��һ���˵��ķ�һ�в�ִ�й���                              �ڶ����˵�ִ�й��ܷ�����һ���˵�
S4 ��һ���˵���һ��ִ�й�����һҳ��            ��һ���˵��ķ�һ��ִ�н����Ӧ�����˵�            �ڶ����˵�ִ�й��� ִ�ж����˵���ʾ����(�Ӽ���)
S5 �̰���һ��(ÿҳ�кü���)
S6 �̰���һ��
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
int loop=0;// ѭ������,0:KEY_1 1:KEY_2 3:KEY_4

// ����״̬��ȡ,�����Ƕ̰�״̬��ĸ�ֵ
// 0 S3
// 1 S4
// 2 S5
// 3 S6
void KeyRead(void)
{
    key_scanner();

    // ���ĸ�������ѭ��
    for(loop=0;loop<KEY_NUMBER;loop++)
    {
        switch(key_get_state(key_index_array[loop]))//��ȡ����״̬
        {
        case KEY_SHORT_PRESS://�̰�
                whichKey=loop;
            key_clear_state(key_index_array[loop]);//�����Ӧ������״̬
            break;
        default://�����ͷ�״̬
            break;
        }
    }
}

void KeyFunction(void)
{
    switch(whichKey)
    {
        case 2: //S5�̰� ��һ��,ÿһ��ҳ����10��ѡ��
            ui.cursor[ui.page]--; //ҳ������--
            if(ui.cursor[ui.page] < 0)
            {
                ui.cursor[ui.page]=4;//����ͷ������
            }
            whichKey=-1;//ˢ�¼�ֵ
            break;
        case 3: // S6 �̰� ��һ��
            ui.cursor[ui.page]++;
            if(ui.cursor[ui.page] > 4)
            {
                ui.cursor[ui.page] = 0;
            }
            whichKey=-1;
            break;
/*
 * �ڲ˵��ĵ�һ�У�ִ�й�����һҳ�������ǵ�һ���˵����ǵڶ����˵�
 * ��һ���˵��������У�ִ�й�����һ���˵�
 *�ڶ����˵��������У�ִ�й��� */
        case 1:   //S4�Ķ̰����� ��һҳ ��һ��ִ�й��ܽ����Ӧ�����˵�
            if(ui.cursor[ui.page] == 0  && ui.enter == -1) //�����������,
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
                    switch(ui.page) // �ĸ�һ���˵�, һ���˵���������ִ�ж����˵�������
                            {
                                case 0:// һ���˵� ��һҳ,ע��һ����������˵���Ψһ��һ���˵�������ľ���ui.enter!=-1,��һ���˵�ִ�н�������˵������Ҫȷ����ǰ����һ���˵�
                                    if(ui.cursor[ui.page]>0 && ui.enter==-1){ui.enter = ui.cursor[ui.page];ui.cursor[ui.page]=0;ips200_clear();}  // �����Ӧ�����˵���,Ĭ���ڵ�һ��
                                    else //���ڶ����˵����У�ִ�ж����˵���Ӧ������
                                    {      switch(ui.enter) //��Ѱ���ڵڼ��еĶ����˵�
                                            {
                                                case 1: //��һҳ���ڶ��еĶ����˵������� steer
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 0://ִ�ж����˵���ʾ�Ķ�Ӧ����
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
                                                case 2: // ��һҳ �����еĶ����˵�������PID
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 0://ִ�ж����˵���ʾ�Ķ�Ӧ����
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
                                                case 3:// ��һҳ �����еĶ����˵�������SPEED
                                                        switch(ui.cursor[ui.page])
                                                        {
                                                            case 0://ִ�ж����˵���ʾ�Ķ�Ӧ����
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

                                case 1: //һ���˵��ڶ�ҳ
                                    if(ui.cursor[ui.page]>0 && ui.enter==-1){ui.enter = ui.cursor[ui.page];ui.cursor[ui.page]=0;ips200_clear();}  // �����Ӧ�����˵���,Ĭ���ڵ�һ��
                                    else //���ڶ����˵����У�ִ�ж����˵���Ӧ������
                                    {      switch(ui.enter) //��Ѱ���ڵڼ��еĶ����˵�
                                            {
                                                case 1: //��2ҳ���ڶ��еĶ����˵������� Element
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
                                                case 2: // ��2ҳ �����еĶ����˵�������PID
                                                            switch(ui.cursor[ui.page])
                                                            {
                                                                case 0://ִ�ж����˵���ʾ�Ķ�Ӧ����

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
                                                case 3:// ��2ҳ �����еĶ����˵�������
                                                        switch(ui.cursor[ui.page])
                                                        {
                                                            case 0://ִ�ж����˵���ʾ�Ķ�Ӧ����
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
                                case 2: //һ���˵�����ҳ
                                    break;
                                case 3: //һ���˵�����ҳ
                                    break;
                                default:
                                    break;

                            }
            }
            whichKey=-1;
            break;
        case 0://S3�̰� һ���˵�����ִ����һҳ,�����˵�ִ�з��ص���Ӧһ���˵�
            if(ui.cursor[ui.page] == 0 && ui.enter == -1)// ��˼���������ڵ�һ��>nextpage��һ�в�ִ����һҳ������
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
                 switch(ui.page)// ��Ѱ��������˵���Ӧ��һ���˵���ҳ������Ϊ�ڽ���һ���˵���ʱ��ui.pageû������
                 {
                    case 0: // ��ʾ���ڵ�һҳ����Ķ����˵��������ڵ�һҳһ���˵�������Ѱ������һ�н���Ķ����˵�
                        if(ui.enter>0){ui.enter=-1;ips200_clear();};
                        break;
                    case 1:
                        if(ui.enter>0){ui.enter=-1;ips200_clear();};
                        break;
                    case 2:// ��ʾ���ڵ���ҳ����Ķ����˵��������ڵ���ҳһ���˵�������Ѱ������һ�н���Ķ����˵�
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
