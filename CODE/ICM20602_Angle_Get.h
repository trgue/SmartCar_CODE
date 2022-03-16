/*
 * ICM20602_Angle_Get.h
 *
 *  Created on: 2022年1月29日
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
//void ComplementaryFiltering2();
float mysqrt(float x);
void ICM_20602(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro) ;//卡尔曼函数

extern float Angle_X_Final;



#endif /* CODE_ICM20602_ANGLE_GET_H_ */
