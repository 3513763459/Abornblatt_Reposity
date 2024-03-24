//--------------------------------------------
// **PID算法  C文件
//BY 秋菊不苦
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
 * 僵尸直立
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
         .balance_A=-3.95,//平衡角，机械中值+向左 陀螺仪向左减小-3 2.65 -3.9

         .balancelime=0,
         .balancelk=0.00,
};

struct pid_motor pid1 =
{
    //直立环角度PD参数
    .zhili_p =205,//无速度环705
    .zhili_i = 0,
    .zhili_d = 32.12,//无速度环16.75
    //速度环PI参数
     .velo_p = 0.025,//1.2
     .velo_d = 0.000,//1.2
     .velo_i = 0.000,//1
     .velo_speed =0,//1
     .motor_die=0,
     .ZHONGZHI=2.3 ,//
     .ZHONGZHIK=0.00,//
 };
//--------------------------------------------------------------
//  @brief     速度环PID
//  @param     Ecount      转速（编码器值）
//  @return    void        没求得
//  @note      最后所得为输出，在结构体 pid2 中可以参考相应注释。
//--------------------------------------------------------------
void pid2_speed()
{
    static float error,error_last,pid2_error_i = 0;
    //得到PI 值

    error = (speed1+speed2)/2-speed_enco;
    pid2_error_i += error;

    pid2_error_i = range(pid2_error_i, -300, 300);
    //PID运算
    pid2.speedout = pid2.speed_p*error + pid2.speed_I*(pid2_error_i)+pid2.speed_D*(error-error_last);
    error_last = error;
    //限幅+得到角度


}

//--------------------------------------------------------------
//  @brief     角度环PD
//  @param     void        没有诶
//  @return    void        也没有诶
//  @note      最后所得为输出，在结构体 pid2 中可以参考相应注释。
//--------------------------------------------------------------
volatile float  error_p1, angle_p_new, angle_d_new, increase_sin;
void pid2_angle(void)
{
    static float error_p1_last,error_pi;
    //得到PI值，并限幅I值
    error_p1 =(pid2.speedout + pid2.balance_A-imu.pitch+LCC(Image_DataStruct.Camera_Error));//改变中点LCC
    //PID运算，D采用陀螺仪单位换算后的角速度
    pid2.angleout = angle_p_new*error_p1 + pid2.angle_d*(error_p1 - error_p1_last)+error_pi*pid2.angle_i;
    error_p1_last = error_p1;
    error_pi+=error_p1;
    error_pi = range( error_pi, -100, 100);//限幅
    //对偏差进行变P
    increase_sin = 5*sin(error_p1/3.1415926);
    if(error_p1<0)
    {
        angle_p_new = pid2.angle_p*(1-increase_sin);
    }
    else
    {
        angle_p_new = pid2.angle_p*(1+increase_sin);
    }
    //限幅，得PWM值
    pid2.angleout = range(pid2.angleout, -240, 240);//限幅
}

//--------------------------------------------------------------
//  @brief     角速度环PI
//  @param     void        没有诶
//  @return    void        也没有诶
//  @note      最后所得为输出，在结构体 pid2 中可以参考相应注释。
//--------------------------------------------------------------
volatile float pid2_gyro_error_i = 0;  //角速度环积分

void pid2_gyro(void)
{
    static float error_p;
    static float error_p_last,pwm;
    //得到PI,并限幅
    error_p = pid2.angleout - imu.deg_s.x;
    pid2_gyro_error_i += error_p;
    pid2_gyro_error_i = range(pid2_gyro_error_i, -3000, 3000);
    //PID运算
    pid2.gyro_out = pid2.gyro_P*error_p + pid2.gyro_i*pid2_gyro_error_i+ pid2.gyro_d*(error_p - error_p_last);
    pid2.gyro_out = range(pid2.gyro_out, -8000, 8000);//限幅8000
    error_p_last = error_p;
    pwm=pid2.gyro_out;
    PWM=pwm;

    Get_motor_stand(PWM);


    turn_LCC(Image_DataStruct.Camera_Error);
}


//--------------------------------------------------------------
//  @brief     PID控制总函数
//  @param     void        没有诶
//  @return    void        也没有诶
//  @note      最后所得为输出，在结构体 pid1 中可以参考相应注释。
//--------------------------------------------------------------

int WDNM=0,WCNM=0,WRNM=0;

void pidget()//2ms
{
    if(facheflag==0&&imu.pitch<20&&imu.pitch>-20)//发车标志位和角度保护
    {
        WDNM++;
        WCNM++;
        WRNM++;
        if(WDNM==1)
        {
            WDNM=0;
            pid2_gyro();//角速度
        }
         if(WCNM==2)
         {
             WCNM=0;
             pid2_angle();//角度
         }
        if(WRNM==5)
        {
            WRNM=0;
            pid2_speed();//速度
        }
    }
    else
    {//停车保护

        pwm_set_duty(ATOM1_CH7_P02_7, 0);
        pwm_set_duty(ATOM1_CH5_P02_5, 0);
        pwm_set_duty(ATOM0_CH6_P02_6,0);
        pwm_set_duty(ATOM0_CH4_P02_4, 0);
    }

}







