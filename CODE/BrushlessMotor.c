/*
 * BrushlessMotor.c
 *
 *  Created on: 2022��1��29��
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


int16 duty = 0;           //PWM��ֵ

motor_struct motor_control;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���������ʼ��
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    #if BLDC_BRAKE_ENABLE==1
        motor_control.brake_flag = 1;   //ɲ��ʹ��
    #else
        motor_control.brake_flag = 0;   //ɲ���ر�
    #endif
    motor_control.dir = FORWARD;                    //����Ĭ�ϵķ���
    #if BLDC_CLOSE_LOOP_ENABLE
    motor_control.set_speed = 0;
    motor_control.max_speed = BLDC_MAX_SPEED;       //���������ת�ٶ�
    motor_control.min_speed = -BLDC_MAX_SPEED;      //�������ת�ٶ�
    #endif
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �����Ϣ�����ʼ��
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void motor_information_out_init(void)
{
    gtm_pwm_init(MOTOR_SPEED_OUT_PIN, 50, 0);       //��ʼ������ٶ�������ţ����ΪƵ�ʱ仯���źţ�������ת��Ϊÿ����5000ת���������ϵ�Ƶ��Ϊ5000Hz��
    gpio_init(MOTOR_DIR_OUT_PIN, GPO, 0, PUSHPULL); //��ʼ���������������ţ��û��˽��գ�

    gpio_init(MOTOR_SWITCH, GPI, 1, PULLUP);        //��ʼ��������ؿ�������
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����A�����ţ�B������     �����ر�
//  @param      duty    ��  PWMռ�ձ�
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
//  @brief      ����A�����ţ�C������     �����ر�
//  @param      duty    ��  PWMռ�ձ�
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
//  @brief      ����B�����ţ�A������     �����ر�
//  @param      duty    ��  PWMռ�ձ�
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
//  @brief      ����B�����ţ�C������     �����ر�
//  @param      duty    ��  PWMռ�ձ�
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
//  @brief      ����C�����ţ�A������     �����ر�
//  @param      duty    ��  PWMռ�ձ�
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
//  @brief      ����C�����ţ�B������     �����ر�
//  @param      duty    ��  PWMռ�ձ�
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
//  @brief      �ر��������ţ������������ţ�ʹ�������Ȧ���Ըе綯�Ƶ��������Լ����ɲ��
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
    //��ȡ����������ŵ�ƽ���л���Ӧ����
    if(!gpio_get(MOTOR_DIR_IN_PIN))
    {
        motor_control.dir = FORWARD;
    }
    else
    {
        motor_control.dir = REVERSE;
    }
    //������ʵ�����еķ�����Ϣ
    motor_dir_out();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ת���������
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void motor_dir_out(void)
{
    gpio_set(MOTOR_DIR_OUT_PIN, motor_control.dir);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ٶ����
//  @param      void
//  @return     void
//  @since      ÿ�λ����ʱ��תIO���ⲿ�������ñ������ӿ�ֱ�Ӳɼ�����
//              �ٶ����������ⲿ�������������ӿڵ�Aͨ�� ������������Bͨ��
//-------------------------------------------------------------------------------------------------------------------
void motor_speed_out(void)
{
    if(speed_filter.data_average)
    {
        pwm_frequency(MOTOR_SPEED_OUT_PIN, myabs(speed_filter.data_average), 5000);     //Ƶ�ʸ��ĺ�����δ�ڿ�Դ����д�� ��Ҫʹ�õ�ͬѧ���н��˺������µ���Դ��
    }
    else
    {
        pwm_frequency(MOTOR_SPEED_OUT_PIN, 1000, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����������
//  @param      void
//  @return     void
//  @since      ���ݼ��������Ŀ����λ������λ���
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
