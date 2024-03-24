/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_imu660ra
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.8.0
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   // Ӳ�� SPI ����
*                   SCL/SPC           �鿴 zf_device_imu660ra.h �� IMU660RA_SPC_PIN �궨��
*                   SDA/DSI           �鿴 zf_device_imu660ra.h �� IMU660RA_SDI_PIN �궨��
*                   SA0/SDO           �鿴 zf_device_imu660ra.h �� IMU660RA_SDO_PIN �궨��
*                   CS                �鿴 zf_device_imu660ra.h �� IMU660RA_CS_PIN �궨��
*                   VCC               3.3V��Դ
*                   GND               ��Դ��
*                   ������������
*
*                   // ��� IIC ����
*                   SCL/SPC           �鿴 zf_device_imu660ra.h �� IMU660RA_SCL_PIN �궨��
*                   SDA/DSI           �鿴 zf_device_imu660ra.h �� IMU660RA_SDA_PIN �궨��
*                   VCC               3.3V��Դ
*                   GND               ��Դ��
*                   ������������
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_config.h"
#include "zf_device_imu660ra.h"



float Atmpe_Y,Atmpe_X;
_Matrix Mat = {
  { { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f } },
  { { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f } }
};
_Attitude att = {0};
imu660_data imu = {
        .deg_s = {0.0f, 0.0f, 0.0f},
        .rad_s = {0.0f, 0.0f, 0.0f},
        .acc_g = {0.0f, 0.0f, 0.0f},
        .att_acc_factor = 0.0f,
        .att_gryo_factor = 0.0f,
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f
    };
//_Matrix Mat = {0};
//_Attitude att = {0};
//
//imu660_data imu = {0} ;

int16 imu660ra_gyro_x = 0, imu660ra_gyro_y = 0, imu660ra_gyro_z = 0;                           // ��������������      gyro (������)
int16 imu660ra_acc_x = 0, imu660ra_acc_y = 0, imu660ra_acc_z = 0;                              // ������ٶȼ�����     acc (accelerometer ���ٶȼ�)
int16 gyro_x = 0, gyro_y = 0, gyro_z = 0;                           // �����������˲�����      gyro (������)
int16 acc_x = 0, acc_y = 0, acc_z = 0;                              // ������ٶȼ��˲�����     acc (accelerometer ���ٶȼ�)



#define MahonyPERIOD        1.0f            //��̬�������ڣ�ms��
#define kp      0.5f                        //proportional gain governs rate of convergence to accelerometer/magnetometer
#define ki      0.0001f                     //integral gain governs rate of convergenceof gyroscope biases

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      //scaled integral error
float imuyaw,imuyawlast,iyaw,imux;

/*
 * ��������get_iir_factor
 * ����  ����ȡIIR�˲������˲�����
 * ����  ��out_factor�˲������׵�ַ��Time����ִ�����ڣ�Cut_Off�˲���ֹƵ��
 * ����  ��
 */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
    *out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}
/**
  * @brief   IIR��ͨ�˲���
  * @param   *acc_in ������������ָ�����
  * @param   *acc_out �����������ָ�����
  * @param   lpf_factor �˲�����
  * @retval  x
  */
float iir_lpf(float in,float out,float lpf_factor)
{
    out = out + lpf_factor * (in - out);
  return out;
}
#if IMU660RA_USE_SOFT_IIC
static soft_iic_info_struct imu660ra_iic_struct;

#define imu660ra_write_register(reg, data)        (soft_iic_write_8bit_register (&imu660ra_iic_struct, (reg), (data)))
#define imu660ra_write_registers(reg, data, len)  (soft_iic_write_8bit_registers(&imu660ra_iic_struct, (reg), (data), (len)))
#define imu660ra_read_register(reg)               (soft_iic_read_8bit_register  (&imu660ra_iic_struct, (reg)))
#define imu660ra_read_registers(reg, data, len)   (soft_iic_read_8bit_registers (&imu660ra_iic_struct, (reg), (data), (len)))
#else
//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // �رո߼�ʡ��ģʽ
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_write_register(uint8 reg, uint8 data)
{
    IMU660RA_CS(0);
    spi_write_8bit_register(IMU660RA_SPI, reg | IMU660RA_SPI_W, data);
    IMU660RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA д����
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_write_registers(uint8 reg, const uint8 *data, uint32 len)
{
    IMU660RA_CS(0);
    spi_write_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_W, data, len);
    IMU660RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8           ����
// ʹ��ʾ��     imu660ra_read_register(IMU660RA_CHIP_ID);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_read_register(uint8 reg)
{
    uint8 data[2];
    IMU660RA_CS(0);
    spi_read_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_R, data, 2);
    IMU660RA_CS(1);
    return data[1];
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA ������
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_read_registers(uint8 reg, uint8 *data, uint32 len)
{
    uint8 temp_data[7];
    IMU660RA_CS(0);
    spi_read_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_R, temp_data, len + 1);
    IMU660RA_CS(1);
    for(int i = 0; i < len; i ++)
    {
        *(data ++) = temp_data[i + 1];
    }
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU660RA �Լ�
// ����˵��     void
// ���ز���     uint8           1-�Լ�ʧ�� 0-�Լ�ɹ�
// ʹ��ʾ��     imu660ra_self_check();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    do
    {
        if(timeout_count ++ > IMU660RA_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
        dat = imu660ra_read_register(IMU660RA_CHIP_ID);
        system_delay_ms(1);
    }while(0x24 != dat);                                                    // ��ȡ�豸ID�Ƿ����0X24���������0X24����Ϊû��⵽�豸
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ IMU660RA ���ٶȼ�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     imu660ra_get_acc();                                             // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ     ʹ�� SPI �Ĳɼ�ʱ��Ϊ69us
//            ʹ�� IIC �Ĳɼ�ʱ��Ϊ126us        �ɼ����ٶȼƵ�ʱ����ɼ������ǵ�ʱ��һ�µ�ԭ���Ƕ�ֻ�Ƕ�ȡ�Ĵ�������
//-------------------------------------------------------------------------------------------------------------------

void imu660ra_get_acc (void)
{
    uint8 dat[6];

    imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
    imu660ra_acc_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660ra_acc_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660ra_acc_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}



//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ IMU660RA ����������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     imu660ra_get_gyro();                                            // ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
// ��ע��Ϣ     ʹ�� SPI �Ĳɼ�ʱ��Ϊ69us
//            ʹ�� IIC �Ĳɼ�ʱ��Ϊ126us
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_get_gyro (void)
{
    uint8 dat[6];

    imu660ra_read_registers(IMU660RA_GYRO_ADDRESS, dat, 6);
    imu660ra_gyro_x = (int16)(((uint16)dat[1]<<8 | dat[0]));
    imu660ra_gyro_y = (int16)(((uint16)dat[3]<<8 | dat[2]));
    imu660ra_gyro_z = (int16)(((uint16)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// �������     �� IMU660RA ���ٶȼ�����ת��Ϊʵ����������
// ����˵��     gyro_value              // ������ļ��ٶȼ�����
// ���ز���     void
// ʹ��ʾ��     float data = imu660ra_acc_transition(imu660ra_acc_x);  //��λΪ g(m/s^2)
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
float imu660ra_acc_transition (int16 acc_value)
{
    float acc_data = 0;
    switch(IMU660RA_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break; // 0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        case 0x01: acc_data = (float)acc_value / 8192; break;  // 0x01 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        case 0x02: acc_data = (float)acc_value / 4096; break;  // 0x02 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        case 0x03: acc_data = (float)acc_value / 2048; break;  // 0x03 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �� IMU660RA ����������ת��Ϊʵ����������
// ����˵��     gyro_value              // �����������������
// ���ز���     void
// ʹ��ʾ��     float data = imu660ra_gyro_transition(imu660ra_gyro_x);  // ��λΪ��/s
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
float imu660ra_gyro_transition (int16 gyro_value)
{
    float gyro_data = 0;
    switch(IMU660RA_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 16.4f;  break;  //  0x00 ����������Ϊ:��2000dps      ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x01: gyro_data = (float)gyro_value / 32.8f;  break;  //  0x01 ����������Ϊ:��1000dps      ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x02: gyro_data = (float)gyro_value / 65.6f;  break;  //  0x02 ����������Ϊ:��500 dps      ��ȡ�������������ݳ���65.6          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x03: gyro_data = (float)gyro_value / 131.2f; break;  //  0x03 ����������Ϊ:��250 dps      ��ȡ�������������ݳ���131.2         ����ת��Ϊ������λ�����ݣ���λΪ����/s
        case 0x04: gyro_data = (float)gyro_value / 262.4f; break;  //  0x04 ����������Ϊ:��125 dps      ��ȡ�������������ݳ���262.4         ����ת��Ϊ������λ�����ݣ���λΪ����/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� IMU660RA
// ����˵��     void
// ���ز���     uint8           1-��ʼ��ʧ�� 0-��ʼ���ɹ�
// ʹ��ʾ��     imu660ra_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660ra_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(20);                                                    // �ȴ��豸�ϵ�ɹ�
#if IMU660RA_USE_SOFT_IIC
    soft_iic_init(&imu660ra_iic_struct, IMU660RA_DEV_ADDR, IMU660RA_SOFT_IIC_DELAY, IMU660RA_SCL_PIN, IMU660RA_SDA_PIN);        // ���� IMU660RA �� IIC�˿�
#else
    spi_init(IMU660RA_SPI, SPI_MODE0, IMU660RA_SPI_SPEED, IMU660RA_SPC_PIN, IMU660RA_SDI_PIN, IMU660RA_SDO_PIN, SPI_CS_NULL);   // ���� IMU660RA �� SPI�˿�
    gpio_init(IMU660RA_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);                                                                  // ���� IMU660RA �� CS�˿�
    imu660ra_read_register(IMU660RA_CHIP_ID);                                                                                   // ��ȡһ���豸ID�����豸����ΪSPIģʽ
#endif
    do{
        if(imu660ra_self_check())                                           // IMU660RA�Լ�
        {
            // �������������˶�����Ϣ ������ʾ����λ��������
            // ��ô���� imu660ra �Լ������ʱ�˳���
            // ���һ�½�����û������ ���û������ܾ��ǻ���
            zf_log(0, "imu660ra self check error.");
            return_state = 1;
            break;
        }
        imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // �رո߼�ʡ��ģʽ
        system_delay_ms(1);
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x00);                  // ��ʼ��ģ����г�ʼ������
        imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));                       // ��������ļ�
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x01);                  // ��ʼ�����ý���
        system_delay_ms(20);
        if(imu660ra_read_register(IMU660RA_INT_STA) == 0)                   // ����Ƿ��������
        {
            // �������������˶�����Ϣ ������ʾ����λ��������
            // ��ô���� imu660ra ���ó�ʼ���ļ�������
            // ���һ�½�����û������ ���û������ܾ��ǻ���
            zf_log(0, "imu660ra init error.");
            return_state = 1;
            break;
        }
        imu660ra_write_register(IMU660RA_PWR_CTRL, 0x0E);                   // ��������ģʽ  ʹ�������ǡ����ٶȡ��¶ȴ�����
        imu660ra_write_register(IMU660RA_ACC_CONF, 0xA7);                   // ���ٶȲɼ����� ����ģʽ �����ɼ� 50Hz ����Ƶ��
        imu660ra_write_register(IMU660RA_GYR_CONF, 0xA9);                   // �����ǲɼ����� ����ģʽ �����ɼ� 200Hz����Ƶ��
        imu660ra_write_register(IMU660RA_ACC_RANGE, IMU660RA_ACC_SAMPLE);   // ���ٶ��������� ��������Ϊ:��8g
        imu660ra_write_register(IMU660RA_GYR_RANGE, IMU660RA_GYR_SAMPLE);   // �������������� ��������Ϊ:��2000dps

        // IMU660RA_GYR_RANGE�Ĵ���
        // ����Ϊ:0x00 ���ٶȼ�����Ϊ:��2g          ��ȡ���ļ��ٶȼ����� ����16384      ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        // ����Ϊ:0x01 ���ٶȼ�����Ϊ:��4g          ��ȡ���ļ��ٶȼ����� ����8192       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        // ����Ϊ:0x02 ���ٶȼ�����Ϊ:��8g          ��ȡ���ļ��ٶȼ����� ����4096       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)
        // ����Ϊ:0x03 ���ٶȼ�����Ϊ:��16g         ��ȡ���ļ��ٶȼ����� ����2048       ����ת��Ϊ������λ�����ݣ���λ��g(m/s^2)

        // IMU660RA_ACC_SAMPLE�Ĵ���
        // ����Ϊ:0x00 ����������Ϊ:��2000dps      ��ȡ�������������ݳ���16.4          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x01 ����������Ϊ:��1000dps      ��ȡ�������������ݳ���32.8          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x02 ����������Ϊ:��500 dps      ��ȡ�������������ݳ���65.6          ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x03 ����������Ϊ:��250 dps      ��ȡ�������������ݳ���131.2         ����ת��Ϊ������λ�����ݣ���λΪ����/s
        // ����Ϊ:0x04 ����������Ϊ:��125 dps      ��ȡ�������������ݳ���262.4         ����ת��Ϊ������λ�����ݣ���λΪ����/s
    }while(0);
    return return_state;
}

// Fast inverse square-root
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*
 * ��������mahony_update
 * ����  ����̬����
 * ����  ���������������ݣ���λ������/�룩�����ٶ��������ݣ���λ��g��
 * ����  ��
 */
//Gyroscope units are radians/second, accelerometer  units are irrelevant as the vector is normalised.
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;


  if(ax*ay*az==0)
      return;
    gx=gx*(PI / 180.0f);
    gy=gy*(PI / 180.0f);
    gz=gz*(PI / 180.0f);
    //[ax,ay,az]�ǻ�������ϵ�¼��ٶȼƲ�õ���������(��ֱ����)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //VectorA = MatrixC * VectorB
    //VectorA ���ο���������ת���ڻ����µ�ֵ
    //MatrixC ����������ϵת��������ϵ����ת����
    //VectorB ���ο�����������0,0,1��
    //[vx,vy,vz]�ǵ�������ϵ����������[0,0,1]����DCM��ת����(C(n->b))����õ��Ļ�������ϵ�е���������(��ֱ����)

  vx = Mat.DCM_T[0][2];
  vy = Mat.DCM_T[1][2];
  vz = Mat.DCM_T[2][2];

  //��������ϵ��������˵õ�������������e���ǲ����õ���v����Ԥ��õ��� v^֮��������ת�������v������[ax,ay,az]��,v^����[vx,vy,vz]��
  //����������������DCM�������Ҿ���(����DCM�����е���Ԫ��)�������������þ��ǽ�bϵ��n��ȷ��ת��ֱ���غϡ�
  //ʵ����������������ֻ��bϵ��nϵ��XOYƽ���غ�����������z����ת��ƫ�������ٶȼ��޿��κΣ�
  //���ǣ����ڼ��ٶȼ��޷���֪z���ϵ���ת�˶������Ի���Ҫ�õشż�����һ��������
  //���������Ĳ���õ��Ľ��������������ģ������֮��н����ҵĳ˻�a��v=|a||v|sin��,
  //���ٶȼƲ����õ�������������Ԥ��õ��Ļ������������Ѿ�������λ����������ǵ�ģ��1��
  //Ҳ����˵���������Ĳ���������sin���йأ����ǶȺ�Сʱ�����������Խ����ڽǶȳ����ȡ�

    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

  //������������л���
    exInt = exInt + ex*ki;
    eyInt = eyInt + ey*ki;
    ezInt = ezInt + ez*ki;

  //��̬���������ٶ��ϣ��������ٶȻ���Ư�ƣ�ͨ������Kp��Ki�������������Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
    gx = gx + kp*ex + exInt;
    gy = gy + kp*ey + eyInt;
    gz = gz + kp*ez + ezInt;

  //һ�����������������Ԫ��
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)* MahonyPERIOD * 0.0005f;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)* MahonyPERIOD * 0.0005f;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)* MahonyPERIOD * 0.0005f;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)* MahonyPERIOD * 0.0005f;

  //��������������Ԫ�����й�һ�������õ������徭����ת����µ���Ԫ����
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

  //��Ԫ��תŷ����
    imu.pitch =   atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3) * (180.0f / PI);
    imu.roll  = - asin(2.0f*(q0*q2 - q1*q3)) * (180.0f / PI);
    imuyaw   = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) *  (180.0f / PI);

    iyaw = imuyaw-imuyawlast;
    if (iyaw < -300) {
        imux += 360;
    }
    else if (iyaw > 300) {
        imux += -360;
    }
    imuyawlast = imuyaw;
    imuyaw +=  imux;
    imu.yaw=imuyaw;

    if (imuyaw > 360) {
           imuyaw -= 360;
           imu.yaw=imuyaw;
       } else if (imuyaw < 0) {
           imuyaw += 360;
           imu.yaw=imuyaw;
       }
    float i =imuyaw - imuyawlast;
    int imuflag=0;
    if(imu.yaw<-170)
    {
        imuflag=1;
    }


    if(i >180)
    {
         i -= 360;

    }
    if (i < -180) {
        i += 360;
    }
    else if (i > 180) {

    }
   imuyaw += i;

    imu.yaw=imuyaw;
  //z����ٶȻ��ֵ�ƫ����
   //imu.yaw += imu.deg_s.z  * MahonyPERIOD * 0.001f;
}
/*
 * ��������rotation_matrix
 * ����  ����ת���󣺻�������ϵ -> ��������ϵ
 * ����  ��
 * ����  ��
 */
void rotation_matrix(void)
{
  Mat.DCM[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
  Mat.DCM[0][1] = 2.0f * (q1*q2 -q0*q3);
  Mat.DCM[0][2] = 2.0f * (q1*q3 +q0*q2);

  Mat.DCM[1][0] = 2.0f * (q1*q2 +q0*q3);
  Mat.DCM[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
  Mat.DCM[1][2] = 2.0f * (q2*q3 -q0*q1);

  Mat.DCM[2][0] = 2.0f * (q1*q3 -q0*q2);
  Mat.DCM[2][1] = 2.0f * (q2*q3 +q0*q1);
  Mat.DCM[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
}
/*
 * ��������rotation_matrix_T
 * ����  ����ת�����ת�þ��󣺵�������ϵ -> ��������ϵ
 * ����  ��
 * ����  ��
 */
void rotation_matrix_T(void)
{
  Mat.DCM_T[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
  Mat.DCM_T[0][1] = 2.0f * (q1*q2 +q0*q3);
  Mat.DCM_T[0][2] = 2.0f * (q1*q3 -q0*q2);

  Mat.DCM_T[1][0] = 2.0f * (q1*q2 -q0*q3);
  Mat.DCM_T[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
  Mat.DCM_T[1][2] = 2.0f * (q2*q3 +q0*q1);

  Mat.DCM_T[2][0] = 2.0f * (q1*q3 +q0*q2);
  Mat.DCM_T[2][1] = 2.0f * (q2*q3 -q0*q1);
  Mat.DCM_T[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;
}
/*
 * ��������Matrix_ready
 * ����  ���������׼����Ϊ��̬����ʹ��
 * ����  ��
 * ����  ��
 */
void Matrix_ready(void)
{
  rotation_matrix();                      //��ת�������
  rotation_matrix_T();                    //��ת�������������
}

/* ��ȡIIR��ͨ�˲�
 *
 *
 *
 *
 * */
void IIR_imu(void)
{
    get_iir_factor(&imu.att_acc_factor,0.001f,10);
    get_iir_factor(&imu.att_gryo_factor,0.001f,10);

}


void IMU(void)
{
  /*��ȡX��Y�Ľ��ٶȺͼ��ٶ�*/
    imu660ra_get_acc();
    imu660ra_get_gyro();
    /*�˲��㷨*/
    acc_x = iir_lpf(imu660ra_acc_x,acc_x,imu.att_acc_factor);
    acc_y = iir_lpf(imu660ra_acc_y,acc_y,imu.att_acc_factor);
    acc_z = iir_lpf(imu660ra_acc_z,acc_z,imu.att_acc_factor);
    gyro_x =iir_lpf(imu660ra_gyro_x,gyro_x,imu.att_gryo_factor);
    gyro_y =iir_lpf(imu660ra_gyro_y,gyro_y,imu.att_gryo_factor);
    gyro_z =iir_lpf(imu660ra_gyro_z,gyro_z,imu.att_gryo_factor);
    /*���ݴ洢*/
    imu.acc_g.x = imu660ra_acc_transition(acc_x);
    imu.acc_g.y = imu660ra_acc_transition(acc_y);
    imu.acc_g.z = imu660ra_acc_transition(acc_z);
    imu.deg_s.x = imu660ra_gyro_transition(gyro_x);
    imu.deg_s.y = imu660ra_gyro_transition(gyro_y);
    imu.deg_s.z = imu660ra_gyro_transition(gyro_z);
    /*��̬����*/
   /* Atmpe_X=(atan(imu.acc_g.y/imu.acc_g.z))*180/3.14;
    Atmpe_Y=(atan(imu.acc_g.x/imu.acc_g.z))*180/3.14;
    hubulvbo_Y(imu.acc_g.y,imu.deg_s.y);
    imu.pitch = 0.2 * imu.acc_g.x + (1 - 0.2) * (Atmpe_X + imu.deg_s.x * 0.001);
    imu.yaw += imu.deg_s.z * 0.001f;
   */
    mahony_update(imu.deg_s.x,imu.deg_s.y,imu.deg_s.z,imu.acc_g.x,imu.acc_g.y,imu.acc_g.z);
    Matrix_ready();

}
