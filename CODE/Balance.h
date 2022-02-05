/*
 * Balance.h
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */

#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_

void encoder_Flywheel_get();
float flywheel_balance(float angle , float gyro);
float flywheel_speed(int encoder);
void flywheel_control_Brush();
void flywheel_control_Brushless();



#endif /* CODE_BALANCE_H_ */
