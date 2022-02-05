/*
 * BrushlessMotor_PID.c
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */


#include "headfile.h"
#include "CCU6_PWM.h"
#include "BrushlessMotor_PID.h"

closed_loop_struct closed_loop;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI�ջ�����
//  @param      read_speed  ��ǰ�ٶ�
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
int32 closed_loop_pi_calc(int32 read_error)
{

    closed_loop.error = read_error;

    closed_loop.pout = closed_loop.error * (closed_loop.kp + (float)myabs(closed_loop.error/1000)/1800);

    //����ϵ�����������ж�̬����
    closed_loop.iout += closed_loop.error * (closed_loop.ki + (float)myabs(closed_loop.error/1000)/38000);

    //�����޷�
    closed_loop.iout = (float)limit_ab(closed_loop.out,0,closed_loop.out_max);

//    //���Ŀ���ٶ�Ϊ0���ߵ�����ر����������
//    if((0 == motor_control.set_speed )|| (ccu61_get_trap_flag()))
//    {
//        closed_loop.iout = 0;
//    }

    closed_loop.out = closed_loop.iout + closed_loop.pout;

    //����޷�
    closed_loop.out = limit_ab(closed_loop.out,0,closed_loop.out_max);

    return closed_loop.out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI�ջ������ʼ��
//  @param      void
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
void closed_loop_pi_init(void)
{
    closed_loop.out_max = PWM_PRIOD_LOAD;
    closed_loop.kp = 0.001;
    closed_loop.ki = 0.00001;
    closed_loop.out = 0;
    closed_loop.real_speed = 0;
}





