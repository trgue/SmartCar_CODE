/*
 * All_Init.c
 *
 *  Created on: 2022��1��23��
 *      Author: guoguo
 */



//ͷ�ļ�����
#include "All_Init.h"
#include "ui.h"
#include "BrushlessMotor_ADC.h"
#include "BrushlessMotor_HALL.h"
#include "Filter.h"
#include "BrushlessMotor_PWM_Input.h"
#include "BrushlessMotor.h"
#include "CCU6_PWM.h"
#include "headfile.h"



//���г�ʼ������
void All_Init()
{
    uart_init(UART_1, 9600, UART1_TX_P11_12, UART1_RX_P11_10);//���ڳ�ʼ��
//    lcd_init();//lcd��Ļ��ʼ��
    UI_Init();//UI��ʼ��
//    gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);//��������ʼ��
//    icm20602_init_spi();//icm20602��ʼ��
//    gtm_pwm_init(ATOM0_CH5_P02_5, 17000,  0);//ATOM 0ģ���ͨ��4 ʹ��P02_4�������PWM  PWMƵ��50HZ  ռ�ձȰٷ�֮0/GTM_ATOM0_PWM_DUTY_MAX*100  GTM_ATOM0_PWM_DUTY_MAX�궨����zf_gtm_pwm.h
//    gtm_pwm_init(ATOM0_CH4_P02_4, 17000,  0);
//    adc_collection_init();//��ʼ��adcͨ����adc���ڲɼ���Դ��ѹ��ĸ�ߵ������������
//    hall_init();//������ʼ��
//    move_filter_init(&speed_filter);//����ƽ���˲���ʼ��
//    motor_information_out_init();//��ʼ������ٶ��뷽����Ϣ������
//    pwm_input_init();//�����źŲ����ʼ��
//    motor_init();//�����ʼ��
//    closed_loop_pi_init();////��ˢ���PID������ʼ��
//    ccu6_pwm_init();//��ʼ����ʱ��,�����������PWM
//    pit_interrupt_ms(CCU6_0, PIT_CH0, 5);//��ʼ����ʱ��,���ڼ���ռ�ձ�
    mt9v03x_init();//����ͷ��ʼ��

}


