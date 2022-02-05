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
float PID_Speed_P = 0;
float PID_Speed_I = 0;
float PID_Speed_D = 0;
int16 PID_num = 0;
int16 PID_Speed_err0 = 0;
int16 PID_Speed_err1 = 0;
int16 PID_Speed_err2 = 0;



//获取通过PID算法计算后获得的值
int16 PID_dat (int16 Rdat , int16 SetNum)
{
    PID_Speed_err0 = SetNum - Rdat;
    PID_num = PID_Speed_P * (PID_Speed_err0 - PID_Speed_err1) + PID_Speed_I * PID_Speed_err0 + PID_Speed_D * (PID_Speed_err0 - 2 * PID_Speed_err1 + PID_Speed_err2);
    PID_Speed_err1 = PID_Speed_err0;
    PID_Speed_err2 = PID_Speed_err1;
    return PID_num;
}








