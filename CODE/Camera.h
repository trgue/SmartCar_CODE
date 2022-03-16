/*
 * Camera.h
 *
 *  Created on: 2022年2月7日
 *      Author: guoguo
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "headfile.h"

uint8 my_adapt_threshold(uint8*image,uint16 col,uint16 row);
void binarization();
void my_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold);
void Side_Search();
void Add_Line(int16 x0, int16 y0, int16 x1, int16 y1);//补线
void CameraWorking996();

extern uint8 T_OSTU;
extern uint8 SelfControl_OSTU;
extern int16 RightDown_Line_Break_Point[1];
extern int16 Right_Line[MT9V03X_H];
extern int16 Left_Line[MT9V03X_H];
extern uint8 Image_Binarization[MT9V03X_H][MT9V03X_W];

#endif /* CODE_CAMERA_H_ */
