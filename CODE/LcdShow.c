/*
 * LcdShow.c
 *
 *  Created on: 2022��1��25��
 *      Author: guoguo
 */


//ͷ��������
#include "LcdShow.h"
#include "Camera.h"
#include "headfile.h"

//���ñ���



//������ʾ����
void Lcd_Show()
{
    lcd_showint16(0 , 0 , RightDown_Line_Break_Point[0]);
    lcd_showint16(0 , 1 , RightDown_Line_Break_Point[1]);
    lcd_showint16(0 , 2 , Right_Line[90]);

}

