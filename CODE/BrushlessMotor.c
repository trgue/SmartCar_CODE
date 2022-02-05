/*
 * BrushlessMotor.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */



#include "headfile.h"
#include "BrushlessMotor_HALL.h"
#include "BrushlessMotor_PID.h"
#include "BrushlessMotor_PWM_Input.h"
#include "bldc_config.h"
#include "CCU6_PWM.h"
#include "Filter.h"
#include "BrushlessMotor.h"


int16 duty = 0;           //PWM初值

motor_struct motor_control;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机参数初始化
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    #if BLDC_BRAKE_ENABLE==1
        motor_control.brake_flag = 1;   //刹车使能
    #else
        motor_control.brake_flag = 0;   //刹车关闭
    #endif
    motor_control.dir = FORWARD;                    //设置默认的方向
    #if BLDC_CLOSE_LOOP_ENABLE
    motor_control.set_speed = 0;
    motor_control.max_speed = BLDC_MAX_SPEED;       //设置最大正转速度
    motor_control.min_speed = -BLDC_MAX_SPEED;      //设置最大反转速度
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机信息输出初始化
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void motor_information_out_init(void)
{
    gtm_pwm_init(MOTOR_SPEED_OUT_PIN, 50, 0);       //初始化电机速度输出引脚，输出为频率变化的信号，例如电机转速为每分钟5000转，则引脚上的频率为5000Hz。
    gpio_init(MOTOR_DIR_OUT_PIN, GPO, 0, PUSHPULL); //初始化电机方向输出引脚（用户端接收）

    gpio_init(MOTOR_SWITCH, GPI, 1, PULLUP);        //初始化电机开关控制引脚
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启A相上桥，B相下桥     其他关闭
//  @param      duty    ：  PWM占空比
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_q1q4_open(uint16 duty)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x0b;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, duty);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启A相上桥，C相下桥     其他关闭
//  @param      duty    ：  PWM占空比
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_q1q6_open(uint16 duty)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x23;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, duty);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启B相上桥，A相下桥     其他关闭
//  @param      duty    ：  PWM占空比
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_q3q2_open(uint16 duty)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x0e;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, duty);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启B相上桥，C相下桥     其他关闭
//  @param      duty    ：  PWM占空比
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_q3q6_open(uint16 duty)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x2c;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, duty);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启C相上桥，A相下桥     其他关闭
//  @param      duty    ：  PWM占空比
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_q5q2_open(uint16 duty)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x32;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, duty);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启C相上桥，B相下桥     其他关闭
//  @param      duty    ：  PWM占空比
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_q5q4_open(uint16 duty)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x38;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, duty);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      关闭所有上桥，开启所有下桥，使电机在线圈的自感电动势的作用下自己完成刹车
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void mos_close(void)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x2C;
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}

void motor_set_dir(void)
{
    //获取方向控制引脚电平并切换对应方向
    if(!gpio_get(MOTOR_DIR_IN_PIN))
    {
        motor_control.dir = FORWARD;
    }
    else
    {
        motor_control.dir = REVERSE;
    }
    //输出电机实际运行的方向信息
    motor_dir_out();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机转动方向输出
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void motor_dir_out(void)
{
    gpio_set(MOTOR_DIR_OUT_PIN, motor_control.dir);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机速度输出
//  @param      void
//  @return     void
//  @since      每次换相的时候翻转IO，外部控制器用编码器接口直接采集即可
//              速度引脚连接外部控制器编码器接口的A通道 方向引脚连接B通道
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_out(void)
{
    if(speed_filter.data_average)
    {
        pwm_frequency(MOTOR_SPEED_OUT_PIN, myabs(speed_filter.data_average), 5000);     //频率更改函数并未在开源库中写入 需要使用的同学自行将此函数更新到开源库
    }
    else
    {
        pwm_frequency(MOTOR_SPEED_OUT_PIN, 1000, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电机换相控制
//  @param      void
//  @return     void
//  @since      根据计算出来的目标相位进行相位输出
//-------------------------------------------------------------------------------------------------------------------
void motor_commutation(int8 except_hall)
{

    #if BLDC_CLOSE_LOOP_ENABLE
        if(motor_control.set_speed){
            switch(except_hall)
               {
                   case 1:     mos_q1q6_open(duty);    break;
                   case 2:     mos_q1q4_open(duty);    break;
                   case 3:     mos_q5q4_open(duty);    break;
                   case 4:     mos_q5q2_open(duty);    break;
                   case 5:     mos_q3q2_open(duty);    break;
                   case 6:     mos_q3q6_open(duty);    break;
                   default:    mos_close();            break;
               }
        }
        else {
            mos_close();
        }
    #else
        if(duty){
            switch(except_hall)
               {
                   case 1:     mos_q1q6_open(duty);    break;
                   case 2:     mos_q1q4_open(duty);    break;
                   case 3:     mos_q5q4_open(duty);    break;
                   case 4:     mos_q5q2_open(duty);    break;
                   case 5:     mos_q3q2_open(duty);    break;
                   case 6:     mos_q3q6_open(duty);    break;
                   default:    mos_close();            break;
               }
        }
        else {
            mos_close();
        }
    #endif


}
