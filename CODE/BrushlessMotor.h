/*
 * BrushlessMotor.h
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */

#ifndef CODE_BRUSHLESSMOTOR_H_
#define CODE_BRUSHLESSMOTOR_H_

#include "common.h"

#define MOTOR_SPEED_OUT_PIN ATOM2_CH1_P33_5 //����ٶ��������
#define MOTOR_DIR_OUT_PIN   P33_11          //������з����������
#define MOTOR_SWITCH        P11_6           //���ʹ�ܿ���


typedef enum
{
    FORWARD,    //��ת
    REVERSE,    //��ת
}MOTOR_DIR_enum;

typedef enum
{
    MOTOR_DISABLE,  //�����ر�
    MOTOR_ENABLE,   //����ʹ��
}MOTOR_EN_STATUS_enum;

typedef struct
{
    MOTOR_EN_STATUS_enum en_status; //ָʾ���ʹ��״̬
    uint8 brake_flag;   //ָʾ��ǰɲ���Ƿ���Ч    1������ɲ��  0����������
    MOTOR_DIR_enum  dir;//�����ת���� FORWARD����ת  REVERSE����ת     BRAKE��ɲ��
    int32 set_speed;    //���õ��ٶ�
    int32 max_speed;    //�ٶ����ֵ
    int32 min_speed;    //�ٶ���Сֵ
}motor_struct;

extern motor_struct motor_control;
extern int16    duty;           //PWMռ�ձȳ�ֵ

void motor_init(void);
void motor_information_out_init(void);
void motor_set_dir(void);
void motor_dir_out(void);
void motor_speed_out(void);
void motor_commutation(int8 except_hall);



#endif /* CODE_BRUSHLESSMOTOR_H_ */