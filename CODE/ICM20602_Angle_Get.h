/*
 * ICM20602_Angle_Get.h
 *
 *  Created on: 2022Äê1ÔÂ29ÈÕ
 *      Author: guoguo
 */

#ifndef CODE_ICM20602_ANGLE_GET_H_
#define CODE_ICM20602_ANGLE_GET_H_

void icm20602_data_get();
void gyroOffsetInit();
void icmGetValues() ;
void icmAHRSupdate();
void IMU_quaterToEulerianAngles();
float Angle_Get();




#endif /* CODE_ICM20602_ANGLE_GET_H_ */
