/*
 * BrushlessMotor_PID.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */


#include "headfile.h"
#include "CCU6_PWM.h"
#include "BrushlessMotor_PID.h"

closed_loop_struct closed_loop;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI闭环计算
//  @param      read_speed  当前速度
//  @return     void
//  @since
//-------------------------------------------------------------------------------------------------------------------
int32 closed_loop_pi_calc(int32 read_error)
{

    closed_loop.error = read_error;

    closed_loop.pout = closed_loop.error * (closed_loop.kp + (float)myabs(closed_loop.error/1000)/1800);

    //积分系数根据误差进行动态调节
    closed_loop.iout += closed_loop.error * (closed_loop.ki + (float)myabs(closed_loop.error/1000)/38000);

    //积分限幅
    closed_loop.iout = (float)limit_ab(closed_loop.out,0,closed_loop.out_max);

//    //如果目标速度为0或者电机被关闭则清除积分
//    if((0 == motor_control.set_speed )|| (ccu61_get_trap_flag()))
//    {
//        closed_loop.iout = 0;
//    }

    closed_loop.out = closed_loop.iout + closed_loop.pout;

    //输出限幅
    closed_loop.out = limit_ab(closed_loop.out,0,closed_loop.out_max);

    return closed_loop.out;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PI闭环计算初始化
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





