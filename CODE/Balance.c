/*
 * Balance.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */


#include "headfile.h"
#include "Balance.h"
#include "tuoluoyi.h"

/*
#include "BrushlessMotor_ADC.h"
#include "BrushlessMotor_HALL.h"
#include "BrushlessMotor.h"
#include "Filter.h"
*/

//变量定义
#define MOTOR_DEADNUM 0
#define MOTOR3_A   ATOM0_CH4_P02_4  //定义3电机正转PWM引脚
#define MOTOR3_B   ATOM0_CH5_P02_5  //定义3电机反转PWM引脚
#define SD_limit   150//舵机限幅
#define SD_middle  770//舵机中值
int16 encoder_flywheel = 0;//动量轮编码器
int16 limit_speed = 10000;//动量轮限速
uint8 Flag = 1;//是否前进
//float flywheel_balance_P = 1800;//直道
//float flywheel_balance_I = 0.7;
//float flywheel_balance_D =35;
float flywheel_balance_P = 2500;
float flywheel_balance_I = 1;
float flywheel_balance_D =55;
float flywheel_speed_P = 10;
float flywheel_speed_I = 0;
float SD_B_P = 0;
float SD_B_I = 0;
float SD_B_D = 0;
float error_my = 0;//动量轮PID角度差
//float flywheel_speed_D = 0;
int16 flywheel_duty = 0;//动量轮速度PWM
float Angle = 0;//角度
float Angle_Zero = 0;//角度零点
float Integration = 0;//舵机PID积分
int16 SD_duty = 770;//舵机PWM
int16 SD_error = 0;//舵机反馈值





//编码器值获取
void encoder_Flywheel_get()
{
    encoder_flywheel = gpt12_get(GPT12_T4);
    gpt12_clear(GPT12_T4);
}






//动量轮PID控制平衡
float flywheel_balance(float angle , float gyro)
{
    float PWM = 0;

    static float errors;
    error_my = angle - Angle_Zero;
    errors += error_my;
    if(errors > 30)
    {
        errors = 30;
    }
    if(errors < -30)
    {
        errors = -30;
    }
    PWM = error_my * flywheel_balance_P + errors * flywheel_balance_I + gyro * flywheel_balance_D;
    return PWM;
}


//动量轮速度闭环
float flywheel_speed(int16 encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = encoder;                                  //速度滤波
    Encoder *= 0.7;                                           //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                             //一阶低通滤波器
    Encoder_Integral += Encoder;                              //积分出位移
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;    //积分限幅
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;    //积分限幅
    Velocity = Encoder * flywheel_speed_P + Encoder_Integral * flywheel_speed_I/100;
                                                              //获取最终数值
    if(Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;//停止时参数清零
    return Velocity;
}


//动量轮控制(有刷电机)
void flywheel_control_Brush()
{
    int16 PWM , PWM_error;
    Flag = 1;
    encoder_Flywheel_get();
    Angle = roll;
    Angle_Zero = 0;
    Angle_Zero = Angle_Zero + (SD_duty - 770)/100.0;//偏
    PWM = flywheel_balance(Angle , icm_gyro_x);
    PWM_error = flywheel_speed(encoder_flywheel);
    flywheel_duty = (PWM - PWM_error);
    //限幅
    if(flywheel_duty < -limit_speed)
    {
        flywheel_duty = -limit_speed;
    }
    else if(flywheel_duty > limit_speed)
    {
        flywheel_duty = limit_speed;
    }
    //死区校正
    else if(flywheel_duty > 0)
    {
        flywheel_duty += MOTOR_DEADNUM;
    }
    else if(flywheel_duty < 0)
    {
        flywheel_duty -= MOTOR_DEADNUM;
    }
    if((flywheel_duty < 1100)&&(flywheel_duty > -1100))
    {
        flywheel_duty = 0;
    }
    if(Angle < -25 || Angle > 25)
    {
        flywheel_duty = 0;
        Integration = 0;
        Flag = 0;
    }
    if(0<=flywheel_duty) //电机1   正转 设置占空比为 百分之 (1000/GTM_ATOM0_PWM_DUTY_MAX*100)
    {
        pwm_duty(MOTOR3_A, flywheel_duty);
        pwm_duty(MOTOR3_B, 0);
    }
    else                //电机1   反转
    {
        pwm_duty(MOTOR3_A, 0);
        pwm_duty(MOTOR3_B, -flywheel_duty);
    }



}

int16 SBB_Get_BalancePID(float Angle,float Gyro)
{
    float  Bias;
    int16 SBB_BalancePID;
    Bias = Angle - Angle_Zero;     // 求出平衡的角度中值和此时横滚角的偏差
    Integration += Bias;           // 积分
    if(Integration<-380)      Integration=-380; //限幅
    else if(Integration>380)  Integration= 380; //限幅
    //===计算平衡控制的舵机PWM  PID控制 kp是P系数 ki式I系数 kd是D系数
    SBB_BalancePID = SD_B_P * Bias + SD_B_I*Integration + SD_B_D*Gyro;
    return SBB_BalancePID;
}

void SD_Balance()
{
    SD_error = SBB_Get_BalancePID(roll , icm_gyro_x);
    SD_duty = 770 + SD_error;
    if(SD_duty >= SD_middle + SD_limit)
    {
        SD_duty = SD_middle + SD_limit;
    }
    if(SD_duty <= SD_middle - SD_limit)
    {
        SD_duty = SD_middle - SD_limit;
    }
    pwm_duty(ATOM0_CH1_P33_9 , SD_duty);
}

void Balance_Control()
{
    flywheel_control_Brush();
//    SD_Balance();
}


/*LLLL

/*
//动量轮控制(无刷电机)
//使用无刷时需将PWM中断开启
void flywheel_control_Brushless()
{
    //角度获取
    Angle = Angle_Get();
    motor_set_dir();
    //通过速度输出引脚输出当前速度
    motor_speed_out();
    motor_control.set_speed = flywheel_balance(Angle , icm_acc_x);
    //进行PI闭环计算
    if(motor_control.dir == FORWARD)
        duty = (int16)closed_loop_pi_calc((float)(motor_control.set_speed - speed_filter.data_average));
    else
        duty = (int16)closed_loop_pi_calc((float)(motor_control.set_speed + speed_filter.data_average));
}

*/









