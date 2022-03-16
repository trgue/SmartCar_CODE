/*
 * Camera_Deal.c
 *
 *  Created on: 2022年3月2日
 *      Author: guoguo
 */

#include "Camera_Deal.h"
#include "Camera.h"
#include "headfile.h"

/****拐点集****/
int16 leftx[3] = {0};
int16 lefty[3] = {0};
int16 rightx[3] = {0};
int16 righty[3] = {0};


/*************************************十字路口***********************************/
void CrossRoad_Deal()
{
    /******************************************正入十字***********************************************/
    int16 L_Down_Flag = 0 , R_Down_Flag = 0 , L_Up_Flag = 0, R_Up_Flag = 0;
    int16 i , j;
    if(L_Down_Flag == 0)//左下拐点
    {
        if(Left_Line[2] != MT9V03X_W && Left_Line[3] != MT9V03X_W && Left_Line[4] != MT9V03X_W)
        {
            for(i = 1 ; i < MT9V03X_H - 3 ; i++)
            {
                if (abs(Left_Line[i] - Left_Line[i - 1]) < 5 && abs(Left_Line[i + 1] - Left_Line[i]) < 5 && Left_Line[i + 2] - Left_Line[i + 1] > 0 && i + 1 >= 2 && Left_Line[i + 1] < 180 && i + 1 < 90)
                {
                    lefty[0] = i + 1;
                    leftx[0] = Left_Line[i + 1];
                    L_Down_Flag = 1;
                    break;
                }
                else
                {
                    lefty[0] = 2;
                    leftx[0] = 188;
                }
            }
        }
    }
    if (R_Down_Flag == 0)//右下拐点
     {
         if (Right_Line[2] != 1 && Right_Line[3] != 1 && Right_Line[4] != 1)
         {
             for (i = 1; i < 115; i++)
             {
                 if (abs(Right_Line[i] - Right_Line[i - 1]) < 5 && abs(Right_Line[i + 1] - Right_Line[i]) < 5 && Right_Line[i + 2] - Right_Line[i + 1] < 0 && i + 1 >= 2 && Right_Line[i + 1] > 2 && i + 1 < 90)
                 {
                     righty[0] = i + 1;
                     rightx[0] = Right_Line[i + 1];
                     R_Down_Flag = 1;
                     break;
                 }
                 else
                 {
                     righty[0] = 2;
                     rightx[0] = 1;
                 }
             }
         }

     }
    if (L_Up_Flag == 0)
    {
        for (i = 3; i < 115; i++)
        {
            if (Left_Line[i] - Left_Line[i - 1] < -3 && abs(Left_Line[i + 1] - Left_Line[i]) < 4 && abs(Left_Line[i + 2] - Left_Line[i + 1]) < 4 && Left_Line[i] < 186 && (i > lefty[0]))
            {
                lefty[1] = i;
                leftx[1] = Left_Line[i];
                L_Up_Flag = 1;
                break;
            }
        }

    }
    if (R_Up_Flag == 0)
    {
        for (i = 3; i < 115; i++)
        {
            if (Right_Line[i] - Right_Line[i - 1] > 3 && abs(Right_Line[i + 1] - Right_Line[i]) < 4 && abs(Right_Line[i + 2] - Right_Line[i + 1]) < 4 && Right_Line[i] > 2 && (i > righty[0]))
            {
                righty[1] = i;
                rightx[1] = Right_Line[i];
                R_Up_Flag = 1;
                break;
            }
        }
    }
    int16 a = 0, b, c;
    if (lefty[0] < righty[0]) b = righty[0]; else b = lefty[0];
    if (lefty[1] > righty[1]) c = righty[1]; else c = lefty[1];
    for (i = b + 1; i < c - 1; i++)
    {
        if (Left_Line[i] > 180 && Right_Line[i] < 10)  a++;   //确定两边丢线的行数
    }

    int flagl = 0, flagr = 0;
    if ((a >= 5 && (lefty[0] > 2 && lefty[1] > 2) || (righty[0] > 2 && righty[1] > 2)) && R_Up_Flag && L_Up_Flag && L_Down_Flag && R_Down_Flag)
    {
        if (lefty[0] >= 2 && lefty[1] >= 2)                                    //找左线              入十字前
        {
            flagl = 1;
            Add_Line(leftx[0], lefty[0], leftx[1], lefty[1]);//补线
        }



        if (righty[0] >= 2 && righty[1] >= 2)                                         //找右线
        {
            flagr = 1;
            Add_Line(rightx[0], righty[0], rightx[1], righty[1]);//补线

        }
    }
    /******************************************正入十字***********************************************/


    /******************************************斜入十字***********************************************/
      //斜入十字，看上方过于地下中线的行数
        int flagxie = 0;
        if (flagl == 0 || flagr == 0)
        {
            int cntl = 0, cntr = 0;
            for (i = 3; i < 115; i++)
            {
                if (i < 60)
                {
                    if ((Right_Line[i] == 1 && Left_Line[i] < 187 && Right_Line[2] == 1 && Right_Line[3] == 1 && Right_Line[i + 1] > Left_Line[2] && Right_Line[i + 2] > Left_Line[3])
                        || (Right_Line[2] == 1 && Right_Line[3] == 1 && Right_Line[i] > Left_Line[10] && Right_Line[i] > Left_Line[14] && Right_Line[i - 1] > Left_Line[12]))
                    {
                        if ((Right_Line[2] == 1 && Right_Line[3] == 1 && Right_Line[i] > Left_Line[10] && Right_Line[i] > Left_Line[14] && Right_Line[i - 1] > Left_Line[14] && Left_Line[2] != 188 && Left_Line[3] != 188))
                        {
                            cntl = 18;
                        }
                        cntl++;//左斜-- 看到左边图像
                    }

                    else if ((Left_Line[i] == 188 && Left_Line[2] == 188 && Left_Line[3] == 188 && Right_Line[i] > 1 && Left_Line[i + 1] < Right_Line[2] && Left_Line[i + 2] < Right_Line[3])/*100待调*/
                        || (Left_Line[2] == 1 && Left_Line[3] == 1 && Left_Line[i] > Right_Line[10] && Left_Line[i] > Right_Line[14] && Left_Line[i - 1] > Right_Line[14]))
                    {
                        if (Left_Line[2] == 1 && Left_Line[3] == 1 && Left_Line[i] > Right_Line[10] && Left_Line[i] > Right_Line[14] && Left_Line[i - 1] > Right_Line[14] && Right_Line[2] != 1 && Right_Line[3] != 1)
                        {
                            cntr = 18;
                        }
                        cntr++;//右斜
                       // lcd_showuint16(0,6,cntr);
                    }
                }


                else //（i>60）
                {
                    if ((Right_Line[2] == 1 && Right_Line[3] == 1 && Right_Line[i] > Left_Line[10] && Right_Line[i] > Left_Line[14] && Right_Line[i - 1] > Left_Line[14] && Left_Line[2] != 188 && Left_Line[3] != 188))
                    {
                        cntl = 18;
                        // lcd_showuint16(0,5,cntl);
                        break;
                    }
                    else if (Left_Line[2] == 188 && Left_Line[3] == 188 && Left_Line[i] < Right_Line[5] && Left_Line[i] < Right_Line[7] && Left_Line[i - 1] < Right_Line[7] && Right_Line[2] != 1 && Right_Line[3] != 1)
                    {
                        cntr = 18;
                        //  lcd_showuint16(0,6,cntr);
                        break;
                    }
                }

            }

            if (cntl > 16 && cntr < 16)
            {
                cntl = 255;                             //左斜入十字
                flagxie = 1;
            }
            else if (cntr > 16 && cntl < 16)
            {
                cntr = 255;                             //右斜入十字
                flagxie = 1;
            }

            //lcd_showuint16(0,5,cntl);
            // lcd_showuint16(0,6,cntr);

            if (cntl == 255)
            {
                for (i = 5; i < 120; i++)
                {
                    if ((Right_Line[i] - Right_Line[i - 1] > 15) && (Right_Line[i] - Right_Line[i - 2] > 15))
                    {
                        lefty[1] = i;
                        leftx[1] = Right_Line[i];


                        righty[0] = 2;
                        rightx[0] = 20;//(可调)
                        righty[1] = i - 1;
                        rightx[1] = Right_Line[i - 1];

                        //               L_black[i] = R_black[i];
                        //               R_black[i] = R_black[i - 1];
                        //               LCenter[i] = R_black[i] / 2 + L_black[i] / 2;

                        Add_Line(leftx[0], lefty[0], leftx[1], lefty[1]);//补线
                        Add_Line(rightx[0], righty[0], rightx[1], righty[1]);
                        break;
                    }
                }

            }
            else if (cntr == 255)//右斜，看到右边图像
            {
                for (i = 5; i < 120; i++)
                {
                    if ((Left_Line[i] - Left_Line[i - 1]) < -15 && (Left_Line[i] - Left_Line[i - 2]) < -15) //左边界跳变15可调
                    {
                        righty[1] = i;
                        rightx[1] = Left_Line[i];


                        leftx[0] = 168;//（可调）
                        lefty[0] = 2;
                        leftx[1] = Left_Line[i - 1];
                        lefty[1] = i - 1;

                        //               R_black[i] = L_black[i];
                        //               L_black[i] = L_black[i - 1];
                        //               LCenter[i] = R_black[i] / 2 + L_black[i] / 2;

                        Add_Line(rightx[0], righty[0], rightx[1], righty[1]);//补线
                        Add_Line(leftx[0], lefty[0], leftx[1], lefty[1]);
                        break;
                    }
                }

            }
        }
        /******************************************斜入十字***********************************************/

        /********************************************** 十 字  中 间 ************************************************************/
           if (R_Up_Flag == 1 && L_Up_Flag == 1 && L_Down_Flag == 0 && R_Down_Flag == 0 && flagxie == 0)
           {

               leftx[0] = leftx[1];
               lefty[0] = 2;
               righty[0] = 2;
               rightx[0] = rightx[1];


               Add_Line(rightx[0], righty[0], rightx[1], righty[1]);//补线
               Add_Line(leftx[0], lefty[0], leftx[1], lefty[1]);

           }
           /********************************************** 十 字  中 间 ************************************************************/
           Side_Search();

}


/****************************************环岛*************************************/
void Island_Deal()
{
    uint8 Flag_Island_In=0;
    if(Flag_Island_In == 0)
    {
        //右线斜率
        float K1 = (Right_Line[10] - Right_Line[40]) * 1.0 / (10 - 40);
        if (K1 > 20) K1 = 20;
        if (K1 < -20) K1 = -20;
        float K2 = (Right_Line[40] - Right_Line[70]) * 1.0 / (40 - 70);
        if (K2 > 20) K2 = 20;
        if (K2 < -20) K2 = -20;
        float K3 = (Right_Line[10] - Right_Line[70]) * 1.0 / (10 - 70);
        if (K3 > 20) K3 = 20;
        if (K3 < -20) K3 = -20;

        if (Left_Line[2] != 188 && Left_Line[3] != 188 && Left_Line[4] != 188) //下拐点
        {
            for (int i = 3; i < 120; i++)
            {
                if (abs(Left_Line[i] - Left_Line[i - 1]) < 5 && abs(Left_Line[i + 1] - Left_Line[i]) < 5 && Left_Line[i + 2] - Left_Line[i + 1] > 0 && i + 1 >= 2 && Left_Line[i + 1] < 180 && i + 1 < 90)
                {
                    lefty[0] = i + 1;
                    leftx[0] = Left_Line[i + 1];
                    break;
                }
                else
                {
                    lefty[0] = 2;
                    leftx[0] = 188;
                }
            }
        }
        for (int i = 2; i <= 115; i++)//中拐点
        {
             if (i >= 8 && (Left_Line[i - 5] - Left_Line[i - 1]) > 0 && (Left_Line[i - 4] - Left_Line[i - 1]) > 0 && (Left_Line[i - 3] - Left_Line[i - 1]) > 0
                 && (Left_Line[i - 5] - Left_Line[i - 4]) >= 0 && (Left_Line[i - 4] - Left_Line[i - 3]) >= 0 && (Left_Line[i - 3] - Left_Line[i - 2]) >= 0
                 && (Left_Line[i + 4] - Left_Line[i - 1]) >= 0 && (Left_Line[i + 3] - Left_Line[i - 1]) >= 0 && (Left_Line[i + 2] - Left_Line[i - 1]) >= 0
                 && i > lefty[0])
             {
                 lefty[1] = i - 1;
                 leftx[1] = Left_Line[i - 1];
                 break;
             }
             else
             {
                 lefty[1] = 2;
                 leftx[1] = 188;
             }
        }

        for (int i = 3; i < 116; i++)//上拐点
        {
            if (Left_Line[i] - Left_Line[i - 1] < -3 && abs(Left_Line[i + 1] - Left_Line[i]) < 4 && abs(Left_Line[i + 2] - Left_Line[i + 1]) < 4
                && Left_Line[i] < 186 && (i > lefty[1]))
            {
                lefty[2] = i;
                leftx[2] = Left_Line[i];
                break;
            }
            else
            {
                lefty[2] = 2;
                leftx[2] = 188;
            }
        }

    }
}


//
///***************************************斑马线*****************************************/
//void check_starting_line(int16 start_point, int16 end_point)
//{
//    /**********************************************确 定 斑 马 线  *****************************************************/
//    uint8 times = 0;
//    int16 flag_starting_line = 0;
//    for(int16 y = start_point; y <= end_point; y++)
//    {
//        int16 black_blocks = 0;
//        int16 cursor = 0;    //指向栈顶的游标
//        for(int16 x = 5; x <= 185; x++)
//        {
//            if (Use_Image[y][x] == 0)//黑点
//             {
//                 if (cursor >= 20)
//                 {
//                     break;          //当黑色元素超过栈长度的操作   break;
//                 }
//                 else
//                 {
//                     cursor++;
//                 }
//             }
//        }
//    }
//}
