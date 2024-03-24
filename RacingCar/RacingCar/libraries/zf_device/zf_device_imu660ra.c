/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_imu660ra
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   // 硬件 SPI 引脚
*                   SCL/SPC           查看 zf_device_imu660ra.h 中 IMU660RA_SPC_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660ra.h 中 IMU660RA_SDI_PIN 宏定义
*                   SA0/SDO           查看 zf_device_imu660ra.h 中 IMU660RA_SDO_PIN 宏定义
*                   CS                查看 zf_device_imu660ra.h 中 IMU660RA_CS_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*
*                   // 软件 IIC 引脚
*                   SCL/SPC           查看 zf_device_imu660ra.h 中 IMU660RA_SCL_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660ra.h 中 IMU660RA_SDA_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
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

int16 imu660ra_gyro_x = 0, imu660ra_gyro_y = 0, imu660ra_gyro_z = 0;                           // 三轴陀螺仪数据      gyro (陀螺仪)
int16 imu660ra_acc_x = 0, imu660ra_acc_y = 0, imu660ra_acc_z = 0;                              // 三轴加速度计数据     acc (accelerometer 加速度计)
int16 gyro_x = 0, gyro_y = 0, gyro_z = 0;                           // 三轴陀螺仪滤波数据      gyro (陀螺仪)
int16 acc_x = 0, acc_y = 0, acc_z = 0;                              // 三轴加速度计滤波数据     acc (accelerometer 加速度计)



#define MahonyPERIOD        1.0f            //姿态解算周期（ms）
#define kp      0.5f                        //proportional gain governs rate of convergence to accelerometer/magnetometer
#define ki      0.0001f                     //integral gain governs rate of convergenceof gyroscope biases

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //quaternion elements representing theestimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;      //scaled integral error
float imuyaw,imuyawlast,iyaw,imux;

/*
 * 函数名：get_iir_factor
 * 描述  ：求取IIR滤波器的滤波因子
 * 输入  ：out_factor滤波因子首地址，Time任务执行周期，Cut_Off滤波截止频率
 * 返回  ：
 */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
    *out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}
/**
  * @brief   IIR低通滤波器
  * @param   *acc_in 输入三轴数据指针变量
  * @param   *acc_out 输出三轴数据指针变量
  * @param   lpf_factor 滤波因数
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
// 函数简介     IMU660RA 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // 关闭高级省电模式
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_write_register(uint8 reg, uint8 data)
{
    IMU660RA_CS(0);
    spi_write_8bit_register(IMU660RA_SPI, reg | IMU660RA_SPI_W, data);
    IMU660RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 写数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 使用示例     imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu660ra_write_registers(uint8 reg, const uint8 *data, uint32 len)
{
    IMU660RA_CS(0);
    spi_write_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_W, data, len);
    IMU660RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU660RA 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8           数据
// 使用示例     imu660ra_read_register(IMU660RA_CHIP_ID);
// 备注信息     内部调用
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
// 函数简介     IMU660RA 读数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 使用示例     imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
// 备注信息     内部调用
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
// 函数简介     IMU660RA 自检
// 参数说明     void
// 返回参数     uint8           1-自检失败 0-自检成功
// 使用示例     imu660ra_self_check();
// 备注信息     内部调用
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
    }while(0x24 != dat);                                                    // 读取设备ID是否等于0X24，如果不是0X24则认为没检测到设备
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 IMU660RA 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660ra_get_acc();                                             // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//            使用 IIC 的采集时间为126us        采集加速度计的时间与采集陀螺仪的时间一致的原因是都只是读取寄存器数据
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
// 函数简介     获取 IMU660RA 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     imu660ra_get_gyro();                                            // 执行该函数后，直接查看对应的变量即可
// 备注信息     使用 SPI 的采集时间为69us
//            使用 IIC 的采集时间为126us
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
// 函数简介     将 IMU660RA 加速度计数据转换为实际物理数据
// 参数说明     gyro_value              // 任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = imu660ra_acc_transition(imu660ra_acc_x);  //单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu660ra_acc_transition (int16 acc_value)
{
    float acc_data = 0;
    switch(IMU660RA_ACC_SAMPLE)
    {
        case 0x00: acc_data = (float)acc_value / 16384; break; // 0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x01: acc_data = (float)acc_value / 8192; break;  // 0x01 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x02: acc_data = (float)acc_value / 4096; break;  // 0x02 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        case 0x03: acc_data = (float)acc_value / 2048; break;  // 0x03 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)
        default: break;
    }
    return acc_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RA 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value              // 任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = imu660ra_gyro_transition(imu660ra_gyro_x);  // 单位为°/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
float imu660ra_gyro_transition (int16 gyro_value)
{
    float gyro_data = 0;
    switch(IMU660RA_GYR_SAMPLE)
    {
        case 0x00: gyro_data = (float)gyro_value / 16.4f;  break;  //  0x00 陀螺仪量程为:±2000dps      获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s
        case 0x01: gyro_data = (float)gyro_value / 32.8f;  break;  //  0x01 陀螺仪量程为:±1000dps      获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
        case 0x02: gyro_data = (float)gyro_value / 65.6f;  break;  //  0x02 陀螺仪量程为:±500 dps      获取到的陀螺仪数据除以65.6          可以转化为带物理单位的数据，单位为：°/s
        case 0x03: gyro_data = (float)gyro_value / 131.2f; break;  //  0x03 陀螺仪量程为:±250 dps      获取到的陀螺仪数据除以131.2         可以转化为带物理单位的数据，单位为：°/s
        case 0x04: gyro_data = (float)gyro_value / 262.4f; break;  //  0x04 陀螺仪量程为:±125 dps      获取到的陀螺仪数据除以262.4         可以转化为带物理单位的数据，单位为：°/s
        default: break;
    }
    return gyro_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RA
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     imu660ra_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660ra_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(20);                                                    // 等待设备上电成功
#if IMU660RA_USE_SOFT_IIC
    soft_iic_init(&imu660ra_iic_struct, IMU660RA_DEV_ADDR, IMU660RA_SOFT_IIC_DELAY, IMU660RA_SCL_PIN, IMU660RA_SDA_PIN);        // 配置 IMU660RA 的 IIC端口
#else
    spi_init(IMU660RA_SPI, SPI_MODE0, IMU660RA_SPI_SPEED, IMU660RA_SPC_PIN, IMU660RA_SDI_PIN, IMU660RA_SDO_PIN, SPI_CS_NULL);   // 配置 IMU660RA 的 SPI端口
    gpio_init(IMU660RA_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);                                                                  // 配置 IMU660RA 的 CS端口
    imu660ra_read_register(IMU660RA_CHIP_ID);                                                                                   // 读取一下设备ID，将设备设置为SPI模式
#endif
    do{
        if(imu660ra_self_check())                                           // IMU660RA自检
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 imu660ra 自检出错并超时退出了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            zf_log(0, "imu660ra self check error.");
            return_state = 1;
            break;
        }
        imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // 关闭高级省电模式
        system_delay_ms(1);
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x00);                  // 开始对模块进行初始化配置
        imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));                       // 输出配置文件
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x01);                  // 初始化配置结束
        system_delay_ms(20);
        if(imu660ra_read_register(IMU660RA_INT_STA) == 0)                   // 检查是否配置完成
        {
            // 如果程序在输出了断言信息 并且提示出错位置在这里
            // 那么就是 imu660ra 配置初始化文件出错了
            // 检查一下接线有没有问题 如果没问题可能就是坏了
            zf_log(0, "imu660ra init error.");
            return_state = 1;
            break;
        }
        imu660ra_write_register(IMU660RA_PWR_CTRL, 0x0E);                   // 开启性能模式  使能陀螺仪、加速度、温度传感器
        imu660ra_write_register(IMU660RA_ACC_CONF, 0xA7);                   // 加速度采集配置 性能模式 正常采集 50Hz 采样频率
        imu660ra_write_register(IMU660RA_GYR_CONF, 0xA9);                   // 陀螺仪采集配置 性能模式 正常采集 200Hz采样频率
        imu660ra_write_register(IMU660RA_ACC_RANGE, IMU660RA_ACC_SAMPLE);   // 加速度量程配置 配置量程为:±8g
        imu660ra_write_register(IMU660RA_GYR_RANGE, IMU660RA_GYR_SAMPLE);   // 陀螺仪量程配置 配置量程为:±2000dps

        // IMU660RA_GYR_RANGE寄存器
        // 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x01 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x02 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x03 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

        // IMU660RA_ACC_SAMPLE寄存器
        // 设置为:0x00 陀螺仪量程为:±2000dps      获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x01 陀螺仪量程为:±1000dps      获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x02 陀螺仪量程为:±500 dps      获取到的陀螺仪数据除以65.6          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x03 陀螺仪量程为:±250 dps      获取到的陀螺仪数据除以131.2         可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x04 陀螺仪量程为:±125 dps      获取到的陀螺仪数据除以262.4         可以转化为带物理单位的数据，单位为：°/s
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
 * 函数名：mahony_update
 * 描述  ：姿态解算
 * 输入  ：陀螺仪三轴数据（单位：弧度/秒），加速度三轴数据（单位：g）
 * 返回  ：
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
    //[ax,ay,az]是机体坐标系下加速度计测得的重力向量(竖直向下)
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //VectorA = MatrixC * VectorB
    //VectorA ：参考重力向量转到在机体下的值
    //MatrixC ：地理坐标系转机体坐标系的旋转矩阵
    //VectorB ：参考重力向量（0,0,1）
    //[vx,vy,vz]是地理坐标系重力分向量[0,0,1]经过DCM旋转矩阵(C(n->b))计算得到的机体坐标系中的重力向量(竖直向下)

  vx = Mat.DCM_T[0][2];
  vy = Mat.DCM_T[1][2];
  vz = Mat.DCM_T[2][2];

  //机体坐标系下向量叉乘得到误差向量，误差e就是测量得到的vˉ和预测得到的 v^之间的相对旋转。这里的vˉ就是[ax,ay,az]’,v^就是[vx,vy,vz]’
  //利用这个误差来修正DCM方向余弦矩阵(修正DCM矩阵中的四元素)，这个矩阵的作用就是将b系和n正确的转化直到重合。
  //实际上这种修正方法只把b系和n系的XOY平面重合起来，对于z轴旋转的偏航，加速度计无可奈何，
  //但是，由于加速度计无法感知z轴上的旋转运动，所以还需要用地磁计来进一步补偿。
  //两个向量的叉积得到的结果是两个向量的模与他们之间夹角正弦的乘积a×v=|a||v|sinθ,
  //加速度计测量得到的重力向量和预测得到的机体重力向量已经经过单位化，因而他们的模是1，
  //也就是说它们向量的叉积结果仅与sinθ有关，当角度很小时，叉积结果可以近似于角度成正比。

    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

  //对误差向量进行积分
    exInt = exInt + ex*ki;
    eyInt = eyInt + ey*ki;
    ezInt = ezInt + ez*ki;

  //姿态误差补偿到角速度上，修正角速度积分漂移，通过调节Kp、Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度。
    gx = gx + kp*ex + exInt;
    gy = gy + kp*ey + eyInt;
    gz = gz + kp*ez + ezInt;

  //一阶龙格库塔法更新四元数
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)* MahonyPERIOD * 0.0005f;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)* MahonyPERIOD * 0.0005f;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)* MahonyPERIOD * 0.0005f;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)* MahonyPERIOD * 0.0005f;

  //把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

  //四元素转欧拉角
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
  //z轴角速度积分的偏航角
   //imu.yaw += imu.deg_s.z  * MahonyPERIOD * 0.001f;
}
/*
 * 函数名：rotation_matrix
 * 描述  ：旋转矩阵：机体坐标系 -> 地理坐标系
 * 输入  ：
 * 返回  ：
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
 * 函数名：rotation_matrix_T
 * 描述  ：旋转矩阵的转置矩阵：地理坐标系 -> 机体坐标系
 * 输入  ：
 * 返回  ：
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
 * 函数名：Matrix_ready
 * 描述  ：矩阵更新准备，为姿态解算使用
 * 输入  ：
 * 返回  ：
 */
void Matrix_ready(void)
{
  rotation_matrix();                      //旋转矩阵更新
  rotation_matrix_T();                    //旋转矩阵的逆矩阵更新
}

/* 获取IIR低通滤波
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
  /*获取X、Y的角速度和加速度*/
    imu660ra_get_acc();
    imu660ra_get_gyro();
    /*滤波算法*/
    acc_x = iir_lpf(imu660ra_acc_x,acc_x,imu.att_acc_factor);
    acc_y = iir_lpf(imu660ra_acc_y,acc_y,imu.att_acc_factor);
    acc_z = iir_lpf(imu660ra_acc_z,acc_z,imu.att_acc_factor);
    gyro_x =iir_lpf(imu660ra_gyro_x,gyro_x,imu.att_gryo_factor);
    gyro_y =iir_lpf(imu660ra_gyro_y,gyro_y,imu.att_gryo_factor);
    gyro_z =iir_lpf(imu660ra_gyro_z,gyro_z,imu.att_gryo_factor);
    /*数据存储*/
    imu.acc_g.x = imu660ra_acc_transition(acc_x);
    imu.acc_g.y = imu660ra_acc_transition(acc_y);
    imu.acc_g.z = imu660ra_acc_transition(acc_z);
    imu.deg_s.x = imu660ra_gyro_transition(gyro_x);
    imu.deg_s.y = imu660ra_gyro_transition(gyro_y);
    imu.deg_s.z = imu660ra_gyro_transition(gyro_z);
    /*姿态解算*/
   /* Atmpe_X=(atan(imu.acc_g.y/imu.acc_g.z))*180/3.14;
    Atmpe_Y=(atan(imu.acc_g.x/imu.acc_g.z))*180/3.14;
    hubulvbo_Y(imu.acc_g.y,imu.deg_s.y);
    imu.pitch = 0.2 * imu.acc_g.x + (1 - 0.2) * (Atmpe_X + imu.deg_s.x * 0.001);
    imu.yaw += imu.deg_s.z * 0.001f;
   */
    mahony_update(imu.deg_s.x,imu.deg_s.y,imu.deg_s.z,imu.acc_g.x,imu.acc_g.y,imu.acc_g.z);
    Matrix_ready();

}
