/*
 * PID_Speed.c
 *
 *  Created on: 2022��1��23��
 *      Author: guoguo
 */



//ͷ�ļ�����
#include "headfile.h"
#include "PID_Speed.h"



//��������
float PID_Speed_P = 21;
float PID_Speed_I = 0.1;
float PID_Speed_D = 0;
int16 PID_num = 0;
int16 PID_Speed_err0 = 0;
int16 PID_Speed_err1 = 0;
int16 PID_Speed_err2 = 0;
short  Motor_Bias, Motor_Last_Bias, Motor_Integration; // ������ò���


//��ȡͨ��PID�㷨������õ�ֵ
int16 PID_dat (int16 Encoder , int16 Target)
{
    static int16 Pwm;
    Motor_Bias = Encoder - Target;            // ����ƫ��
    Pwm += PID_Speed_P * (Motor_Bias - Motor_Last_Bias) + PID_Speed_I * Motor_Bias;
    // ==����ʽPI������
    if(Pwm > 7000) Pwm = 7000;               // �޷�
    else if(Pwm < -7000)Pwm = -7000;         // �޷�
    Motor_Last_Bias = Motor_Bias;            // ������һ��ƫ��
    return Pwm;                              // �������
}








