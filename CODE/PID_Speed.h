/*
 * PID_Speed.h
 *
 *  Created on: 2022��1��23��
 *      Author: guoguo
 */

#ifndef CODE_PID_SPEED_H_
#define CODE_PID_SPEED_H_

#include "headfile.h"

int16 PID_dat (int16 Encoder , int16 Target);

extern float PID_Speed_P;
extern float PID_Speed_I;
extern float PID_Speed_D;



#endif /* CODE_PID_SPEED_H_ */
