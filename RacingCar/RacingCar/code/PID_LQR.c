//--------------------------------------------
// **PID�㷨  C�ļ�
//BY ��ղ���
//--------------------------------------------
#include "PID_LQR.h"
#include "forward.h"
#include "Motor.h"
#include "Camera.h"

float speed1,speed2,speed3,startflag=0,PWM;
int A=-10,B=10;
float zhengspeed=0,wangzhi1=0,wangzhi2=0,k1=0,k2=0,speedk=5,speed_enco=0;//2.5
float zhili_out;

int facheflag=0;
/*
 *          .speed_p = -0.75,
         .speed_I =-0.008,
         .speed_D = 0,

         .angle_p =2,
         .angle_d =0,
         .angle_i =0.01,

         .gyro_P  = 15,
         .gyro_i  =0.8,
         .gyro_d = 0,
         */
/*
 * ��ʬֱ��
 *          .speed_p = -0.75,
         .speed_I =-0.009,
         .speed_D = -0.11,

         .angle_p =2,
         .angle_d =0,
         .angle_i =0.01,

         .gyro_P  = 15,
         .gyro_i  =0.8,
         .gyro_d = 2.9,
         */
struct pid_MW pid2 = {
         .speed_p = -0.75,
         .speed_I =-0.009,
         .speed_D = -0.11,

         .angle_p =2,
         .angle_d =0,
         .angle_i =0.01,

         .gyro_P  = 15,
         .gyro_i  =0.8,
         .gyro_d = 2.9,

         .turn_pspeedk=0,
         .turn_dspeedk=0,
         .turn_p = 0,
         .turn_i = 0,
         .turn_ilimit = 0,
         .turn_iaim=0,
         .turn_d = 0,//0

         .yawan_p1=0,
         .yawan_p2=0.0,
         .yawan_aim1=0,
         .yawan_aim2=0,
         .yawan_max=0.00,
         .yawan_min=0.00,

         .turnspeed_p = 0,
         .turnspeed_i = 0,

         .aimgyro=0,
         .balance_A=-3.95,//ƽ��ǣ���е��ֵ+���� �����������С-3 2.65 -3.9

         .balancelime=0,
         .balancelk=0.00,
};

struct pid_motor pid1 =
{
    //ֱ�����Ƕ�PD����
    .zhili_p =205,//���ٶȻ�705
    .zhili_i = 0,
    .zhili_d = 32.12,//���ٶȻ�16.75
    //�ٶȻ�PI����
     .velo_p = 0.025,//1.2
     .velo_d = 0.000,//1.2
     .velo_i = 0.000,//1
     .velo_speed =0,//1
     .motor_die=0,
     .ZHONGZHI=2.3 ,//
     .ZHONGZHIK=0.00,//
 };
//--------------------------------------------------------------
//  @brief     �ٶȻ�PID
//  @param     Ecount      ת�٣�������ֵ��
//  @return    void        û���
//  @note      �������Ϊ������ڽṹ�� pid2 �п��Բο���Ӧע�͡�
//--------------------------------------------------------------
void pid2_speed()
{
    static float error,error_last,pid2_error_i = 0;
    //�õ�PI ֵ

    error = (speed1+speed2)/2-speed_enco;
    pid2_error_i += error;

    pid2_error_i = range(pid2_error_i, -300, 300);
    //PID����
    pid2.speedout = pid2.speed_p*error + pid2.speed_I*(pid2_error_i)+pid2.speed_D*(error-error_last);
    error_last = error;
    //�޷�+�õ��Ƕ�


}

//--------------------------------------------------------------
//  @brief     �ǶȻ�PD
//  @param     void        û����
//  @return    void        Ҳû����
//  @note      �������Ϊ������ڽṹ�� pid2 �п��Բο���Ӧע�͡�
//--------------------------------------------------------------
volatile float  error_p1, angle_p_new, angle_d_new, increase_sin;
void pid2_angle(void)
{
    static float error_p1_last,error_pi;
    //�õ�PIֵ�����޷�Iֵ
    error_p1 =(pid2.speedout + pid2.balance_A-imu.pitch+LCC(Image_DataStruct.Camera_Error));//�ı��е�LCC
    //PID���㣬D���������ǵ�λ�����Ľ��ٶ�
    pid2.angleout = angle_p_new*error_p1 + pid2.angle_d*(error_p1 - error_p1_last)+error_pi*pid2.angle_i;
    error_p1_last = error_p1;
    error_pi+=error_p1;
    error_pi = range( error_pi, -100, 100);//�޷�
    //��ƫ����б�P
    increase_sin = 5*sin(error_p1/3.1415926);
    if(error_p1<0)
    {
        angle_p_new = pid2.angle_p*(1-increase_sin);
    }
    else
    {
        angle_p_new = pid2.angle_p*(1+increase_sin);
    }
    //�޷�����PWMֵ
    pid2.angleout = range(pid2.angleout, -240, 240);//�޷�
}

//--------------------------------------------------------------
//  @brief     ���ٶȻ�PI
//  @param     void        û����
//  @return    void        Ҳû����
//  @note      �������Ϊ������ڽṹ�� pid2 �п��Բο���Ӧע�͡�
//--------------------------------------------------------------
volatile float pid2_gyro_error_i = 0;  //���ٶȻ�����

void pid2_gyro(void)
{
    static float error_p;
    static float error_p_last,pwm;
    //�õ�PI,���޷�
    error_p = pid2.angleout - imu.deg_s.x;
    pid2_gyro_error_i += error_p;
    pid2_gyro_error_i = range(pid2_gyro_error_i, -3000, 3000);
    //PID����
    pid2.gyro_out = pid2.gyro_P*error_p + pid2.gyro_i*pid2_gyro_error_i+ pid2.gyro_d*(error_p - error_p_last);
    pid2.gyro_out = range(pid2.gyro_out, -8000, 8000);//�޷�8000
    error_p_last = error_p;
    pwm=pid2.gyro_out;
    PWM=pwm;

    Get_motor_stand(PWM);


    turn_LCC(Image_DataStruct.Camera_Error);
}


//--------------------------------------------------------------
//  @brief     PID�����ܺ���
//  @param     void        û����
//  @return    void        Ҳû����
//  @note      �������Ϊ������ڽṹ�� pid1 �п��Բο���Ӧע�͡�
//--------------------------------------------------------------

int WDNM=0,WCNM=0,WRNM=0;

void pidget()//2ms
{
    if(facheflag==0&&imu.pitch<20&&imu.pitch>-20)//������־λ�ͽǶȱ���
    {
        WDNM++;
        WCNM++;
        WRNM++;
        if(WDNM==1)
        {
            WDNM=0;
            pid2_gyro();//���ٶ�
        }
         if(WCNM==2)
         {
             WCNM=0;
             pid2_angle();//�Ƕ�
         }
        if(WRNM==5)
        {
            WRNM=0;
            pid2_speed();//�ٶ�
        }
    }
    else
    {//ͣ������

        pwm_set_duty(ATOM1_CH7_P02_7, 0);
        pwm_set_duty(ATOM1_CH5_P02_5, 0);
        pwm_set_duty(ATOM0_CH6_P02_6,0);
        pwm_set_duty(ATOM0_CH4_P02_4, 0);
    }

}







