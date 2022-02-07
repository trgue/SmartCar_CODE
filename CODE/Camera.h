/*
 * Camera.h
 *
 *  Created on: 2022Äê2ÔÂ7ÈÕ
 *      Author: guoguo
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#include "headfile.h"

uint8 my_adapt_threshold(uint8*image,uint16 col,uint16 row);
void binarization();
void my_sobel (unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold);
void CameraWorking996();



#endif /* CODE_CAMERA_H_ */
