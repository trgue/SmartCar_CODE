/*
 * Balance.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */


#include "headfile.h"
#include "Balance.h"
#include "ICM20602_Angle_Get.h"
#include "BrushlessMotor_ADC.h"
#include "BrushlessMotor_HALL.h"
#include "BrushlessMotor.h"
#include "Filter.h"


//变量定义
#define MOTOR_DEADNUM 500
int16 encoder_flywheel = 0;
uint8 Flag = 0;
float flywheel_balance_P = 0;
float flywheel_balance_I = 0;
float flywheel_balance_D = 0;
float flywheel_speed_P = 0;
float flywheel_speed_I = 0;
//float flywheel_speed_D = 0;
int16 flywheel_duty = 0;
float Angle = 0;
float Angle_Zero = 0;



//编码器值获取
void encoder_Flywheel_get()
{
    encoder_flywheel = gpt12_get(GPT12_T2);
    gpt12_clear(GPT12_T2);
}






//动量轮PID控制平衡
float flywheel_balance(float angle , float gyro)
{
    float PWM = 0;
    float error = 0;
    static float errors;
    error = angle - Angle_Zero;
    errors += error;
    if(errors > 30)
    {
        errors = 30;
    }
    if(errors < -30)
    {
        errors = -30;
    }
    PWM = error * flywheel_balance_P + errors * flywheel_balance_I + gyro * flywheel_balance_D;
    return PWM;
}


//动量轮速度闭环
float flywheel_speed(int encoder)
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
    encoder_Flywheel_get();
    Angle = Angle_Get();
    PWM = flywheel_balance(Angle , icm_acc_x);
    PWM_error = flywheel_speed(encoder_flywheel);
    flywheel_duty = PWM - PWM_error;
    //限幅
    if(flywheel_duty < -8000)
    {
        flywheel_duty = -8000;
    }
    else if(flywheel_duty > 8000)
    {
        flywheel_duty = 8000;
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
    if((flywheel_duty < 1000)&&(flywheel_duty > 1000))
    {
        flywheel_duty = 0;
    }

}


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











