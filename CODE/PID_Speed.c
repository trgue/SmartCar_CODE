/*
 * PID_Speed.c
 *
 *  Created on: 2022年1月23日
 *      Author: guoguo
 */



//头文件包含
#include "headfile.h"
#include "PID_Speed.h"



//变量定义
float PID_Speed_P = 21;
float PID_Speed_I = 0.1;
float PID_Speed_D = 0;
int16 PID_num = 0;
int16 PID_Speed_err0 = 0;
int16 PID_Speed_err1 = 0;
int16 PID_Speed_err2 = 0;
short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // 电机所用参数


//获取通过PID算法计算后获得的值
int16 PID_dat (int16 Encoder , int16 Target)
{
    static int16 Pwm;
    Motor_Bias = Encoder - Target;            // 计算偏差
    Pwm += PID_Speed_P * (Motor_Bias - Motor_Last_Bias) + PID_Speed_I * Motor_Bias;
    // ==增量式PI控制器
    if(Pwm > 7000) Pwm = 7000;               // 限幅
    else if(Pwm < -7000)Pwm = -7000;         // 限幅
    Motor_Last_Bias = Motor_Bias;            // 保存上一次偏差
    return Pwm;                              // 增量输出
}








