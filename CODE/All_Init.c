/*
 * All_Init.c
 *
 *  Created on: 2022年1月23日
 *      Author: guoguo
 */



//头文件包含
#include "All_Init.h"
#include "ui.h"
#include "BrushlessMotor_ADC.h"
#include "BrushlessMotor_HALL.h"
#include "Filter.h"
#include "BrushlessMotor_PWM_Input.h"
#include "BrushlessMotor.h"
#include "CCU6_PWM.h"
#include "headfile.h"



//所有初始化函数
void All_Init()
{
    uart_init(UART_1, 9600, UART1_TX_P11_12, UART1_RX_P11_10);//串口初始化
//    lcd_init();//lcd屏幕初始化
    UI_Init();//UI初始化
//    gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);//编码器初始化
//    icm20602_init_spi();//icm20602初始化
//    gtm_pwm_init(ATOM0_CH5_P02_5, 17000,  0);//ATOM 0模块的通道4 使用P02_4引脚输出PWM  PWM频率50HZ  占空比百分之0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX宏定义在zf_gtm_pwm.h
//    gtm_pwm_init(ATOM0_CH4_P02_4, 17000,  0);
//    adc_collection_init();//初始化adc通道，adc用于采集电源电压、母线电流、相电流的
//    hall_init();//霍尔初始化
//    move_filter_init(&speed_filter);//滑动平均滤波初始化
//    motor_information_out_init();//初始化输出速度与方向信息的引脚
//    pwm_input_init();//输入信号捕获初始化
//    motor_init();//电机初始化
//    closed_loop_pi_init();////无刷电机PID参数初始化
//    ccu6_pwm_init();//初始化定时器,用于输出互补PWM
//    pit_interrupt_ms(CCU6_0, PIT_CH0, 5);//初始化定时器,用于计算占空比
    mt9v03x_init();//摄像头初始化

}


