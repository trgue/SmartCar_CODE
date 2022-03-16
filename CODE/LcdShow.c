/*
 * LcdShow.c
 *
 *  Created on: 2022年1月25日
 *      Author: guoguo
 */


//头函数包含
#include "LcdShow.h"
#include "Camera.h"
#include "Balance.h"
#include "tuoluoyi.h"
#include "headfile.h"
#include "ICM20602_Angle_Get.h"

//引用变量



//所有显示函数
void Lcd_Show()
{
    lcd_showint16(0 , 0 , flywheel_duty);
    lcd_showint16(0 , 1 , encoder_flywheel);
//    lcd_showint16(0 , 2 , Right_Line[90]);
//    lcd_showfloat(0, 2 , Angle_X_Final, 5, 5);
    lcd_showfloat(0, 2, pitch, 4, 5);
    lcd_showfloat(0, 3, yaw, 4, 5);


}

