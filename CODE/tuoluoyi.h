/*
 * tuoluoyi.h
 *
 *  Created on: 2021��11��4��
 *      Author: �ν���
 */

#ifndef CODE_TUOLUOYI_H_
#define CODE_TUOLUOYI_H_
#include "headfile.h"

extern void IMU_getValues(void);
extern void IMU_quaterToEulerianAngles(void);
extern  void IMU_AHRSupdate_noMagnetic(float , float , float , float , float , float );
extern float invSqrt(float );

extern float yaw,pitch,roll;






#endif /* CODE_TUOLUOYI_H_ */
