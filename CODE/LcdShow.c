/*
 * LcdShow.c
 *
 *  Created on: 2022年1月25日
 *      Author: guoguo
 */


//头函数包含
#include "LcdShow.h"
#include "Camera.h"
#include "headfile.h"

//引用变量



//所有显示函数
void Lcd_Show()
{
    lcd_showint16(0 , 0 , RightDown_Line_Break_Point[0]);
    lcd_showint16(0 , 1 , RightDown_Line_Break_Point[1]);
    lcd_showint16(0 , 2 , Right_Line[90]);

}

