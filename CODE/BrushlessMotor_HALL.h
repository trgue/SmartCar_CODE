/*
 * BrushlessMotor_Hall.h
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */

#ifndef CODE_BRUSHLESSMOTOR_HALL_H_
#define CODE_BRUSHLESSMOTOR_HALL_H_


#include "common.h"

//�������ֵ��������
#define HALLA_PIN   P15_6
#define HALLB_PIN   P15_7
#define HALLC_PIN   P15_8

//������������
#define POLEPAIRS           1

#define PWM_PIT_NUM          60*FPWM

#define COMMUTATION_TIMEOUT 5000

#define MOTOR_RUN   1
#define MOTOR_STOP  0
#define DELAY_MAX   20  //��ʱ�������ʱ��

extern uint32   commutation_delay;
extern int8     hall_value_now;     //��ǰ������ֵ
extern int8     next_hall_value;    //��һ�λ�����ֵ
extern uint16   commutation_time;
extern uint16   commutation_delay_ratio;
extern uint8 motor_state;

void hall_init(void);
void read_hall_value(void);
void scan_hall_status(void);


#endif /* CODE_BRUSHLESSMOTOR_HALL_H_ */
