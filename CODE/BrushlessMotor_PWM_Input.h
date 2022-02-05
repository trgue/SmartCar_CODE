/*
 * BrushlessMotor_PWM_Input.h
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */

#ifndef CODE_BRUSHLESSMOTOR_PWM_INPUT_H_
#define CODE_BRUSHLESSMOTOR_PWM_INPUT_H_


#include "ifxGtm_Tim.h"
#include "IFXGTM_TIM_IN.h"
#include "common.h"


#define GTM_PWM_IN_PRIORITY 80          //输入捕获中断优先级

#define MOTOR_DIR_IN_PIN    P11_3
#define MOTOR_PWM_IN_PIN    P11_2

extern uint16 pwm_in_duty;
extern IfxGtm_Tim_In driver;

void pwm_input_init(void);


#endif /* CODE_BRUSHLESSMOTOR_PWM_INPUT_H_ */
