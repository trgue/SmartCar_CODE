/*
 * CCU6_PWM.h
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */

#ifndef CODE_CCU6_PWM_H_
#define CODE_CCU6_PWM_H_

#include "ifxccu6_regdef.h"
#include "common.h"

#define FCY             ((uint32)100000000)     //ϵͳʱ��
#define FPWM            ((uint16)20000)         //PWMƵ��
#define PWM_PRIOD_LOAD  (uint16)(FCY/FPWM/2)    //PWM����װ��ֵ
#define DEADTIME_LOAD   (50)                    //����װ��ֵ



extern Ifx_CCU6 *ccu6SFR;




void ccu6_pwm_init(void);
uint8 ccu61_get_trap_flag(void);



#endif /* CODE_CCU6_PWM_H_ */
