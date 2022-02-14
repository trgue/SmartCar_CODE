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
    lcd_showuint8(0 , 0 , T_OSTU);
}

