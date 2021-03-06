/*
 * Camera.c
 *
 *  Created on: 2022年2月7日
 *      Author: guoguo
 */

//头文件包含
#include "Camera.h"
#include "headfile.h"



//变量定义
#define ShowFlag  3 //控制摄像头显示模式:模式0为输出原图像,模式1为输出二值化后图像,模式2为输出边缘图像,模式3为使用上位机
#define DoImageFlag   2 //控制图像处理模式:模式0为sobel边缘提取（阈值动态控制），模式1为sobel边缘提取（阈值手动控制），模式2为一般方法


uint8 Image_Binarization[MT9V03X_H][MT9V03X_W];
uint8 Image_Side[MT9V03X_H][MT9V03X_W];

uint8 T_OSTU = 0;
uint8 SelfControl_OSTU = 0;
int16 Left_Line_Flag[MT9V03X_H] = {0};       //左边线是否扫到标识集
int16 Right_Line_Flag[MT9V03X_H] = {0};      //右边线是否扫到标识集
int16 Middle_Line[MT9V03X_H] = {0};          //中线集
int16 Left_Line[MT9V03X_H] = {0};            //左边线集
int16 Right_Line[MT9V03X_H] = {0};           //右边线集
int16 RightDown_Line_Break_Point[1] = {0};   //右下拐点
int16 Break_Line = 0;                        //中线出赛道行数
int16 All_Get_Times = 0;                     //80行以内左右边界都扫到次数
int16 All_Lose_Times = 0;                    //40行以内左右边界都丢失次数
int16 Left_Break_Line = 0;                   //30行以上左边线断裂行数
int16 Right_Break_Line = 0;                  //30行以上右边线断裂行数
uint8 Straight_Flag = 0;//直线判断标志






//大津法求阈值
uint8 my_adapt_threshold(uint8*image,uint16 col,uint16 row)
{
    #define GrayScale 256
    uint16 width=col;
    uint16 height=row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i,j,pixelSum=width*height/4;
    uint8 threshold=0;
    uint8*data=image;
    for(i=0;i<GrayScale;i++)
    {
        pixelCount[i]=0;
        pixelPro[i]=0;
    }
    uint32 gray_sum=0;
    for(i=0;i<height;i+=2)
    {
        for(j=0;j<width;j+=2)
        {
            pixelCount[(int)data[i*width+j]]++;
            gray_sum+=(int)data[i*width+j];
        }
    }
    for(i=0;i<GrayScale;i++)
    {
        pixelPro[i]=(float)pixelCount[i]/pixelSum;
    }
    float w0,w1,u0tmp,u1tmp,u0,u1,u,deltaTmp,deltaMax=0;
    w0=w1=u0tmp=u1tmp=u0=u1=u=deltaTmp=0;
    for(j=0;j<GrayScale;j++)
    {
        w0+=pixelPro[j];
        u0tmp+=j*pixelPro[j];
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0=u0tmp/w0;
        u1=u1tmp/w1;
        u=u0tmp+u1tmp;
        deltaTmp=w0*pow((u0-u),2)+w1*pow((u1-u),2);
        if(deltaTmp>deltaMax)
        {
            deltaMax=deltaTmp;
            threshold=j;
        }
        if(deltaTmp<deltaMax)
        {
            break;
        }
    }
    return threshold;
}



//通过大津法求出的阈值进行二值化
void binarization()
{
    uint8 i,j;
    T_OSTU = my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
    for(i = 0;i < MT9V03X_H;i++)
    {
      for(j = 0;j < MT9V03X_W;j++)
      {
          if(mt9v03x_image[i][j] <= T_OSTU)
          {
              Image_Binarization[MT9V03X_H - i - 1][MT9V03X_W - i - 1] = BLACK;
          }
          else
          {
              Image_Binarization[MT9V03X_H - i - 1][MT9V03X_W - i - 1] = WHITE;
          }
      }
    }
}











//基于sobel边缘检测算子的一种边缘检测
void my_sobel(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
{
    /* 卷积核大小 */
    short KERNEL_SIZE = 3;
    short xStart = KERNEL_SIZE / 2;
    short xEnd = MT9V03X_W - KERNEL_SIZE / 2;
    short yStart = KERNEL_SIZE / 2;
    short yEnd = MT9V03X_H - KERNEL_SIZE / 2;
    short i, j, k;
    short temp[4];
    for (i = yStart; i < yEnd - 1; i++)
    {
        for (j = xStart; j < xEnd - 1; j++)
        {
            /* 计算不同方向梯度幅值  */
            temp[0] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j + 1]     //{{-1, 0, 1},
            - (short) imageIn[i][j - 1] + (short) imageIn[i][j + 1]        // {-1, 0, 1},
            - (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j + 1];    // {-1, 0, 1}};

            temp[1] = -(short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j - 1]     //{{-1, -1, -1},
            - (short) imageIn[i - 1][j] + (short) imageIn[i + 1][j]       // { 0,  0,  0},
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j + 1];    // { 1,  1,  1}};

            temp[2] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j - 1]       //  0, -1, -1
            - (short) imageIn[i][j + 1] + (short) imageIn[i + 1][j]       //  1,  0, -1
            - (short) imageIn[i - 1][j + 1] + (short) imageIn[i + 1][j - 1];    //  1,  1,  0

            temp[3] = -(short) imageIn[i - 1][j] + (short) imageIn[i][j + 1]       // -1, -1,  0
            - (short) imageIn[i][j - 1] + (short) imageIn[i + 1][j]       // -1,  0,  1
            - (short) imageIn[i - 1][j - 1] + (short) imageIn[i + 1][j + 1];    //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (short) imageIn[i - 1][j - 1] + (short) imageIn[i - 1][j] + (short) imageIn[i - 1][j + 1]
                    + (short) imageIn[i][j - 1] + (short) imageIn[i][j] + (short) imageIn[i][j + 1]
                    + (short) imageIn[i + 1][j - 1] + (short) imageIn[i + 1][j] + (short) imageIn[i + 1][j + 1];

//            if (temp[0] > Threshold)
//            {
//                imageOut[i][j] = BLACK;
//            }
            if (temp[0] > temp[3] / 12.0f)
            {
                imageOut[i][j] = BLACK;
            }
            else
            {
                imageOut[i][j] = WHITE;
            }
        }
    }
}



//一般方法扫边界
void Side_Search()
{
    //变量定义
    int16 i,j;
    /*
    float Middle_Line_Sum1 = 0;             //前40行的中线偏差平方和
    float Middle_Line_Sum2 = 0;             //前40行的中线偏差和
    */
    //清除上一帧图像
    for(i = 0 ; i < MT9V03X_H ; i++)
    {
        for(j = 0 ; j < MT9V03X_W ; j++)
        {
            Image_Side[i][j] = WHITE;
        }
    }


    //开始扫线
    for(i = 0 ; i < MT9V03X_H ; i++)
    {
        //第一行处理
        if(i == 0)
        {
            //往左扫
            for(j = MT9V03X_W / 2 ; j <= MT9V03X_W - 1 ; j = j++)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j - 1] == BLACK)//当前点为黑点且上一个为白点
                {
                    Image_Side[i][j] = BLACK;
                    Left_Line_Flag[i] = 1;
                    Left_Line[i] = j;
                    break;
                }
                else
                {
                    ;
                }
            }
            //往右扫
            for(j = MT9V03X_W / 2 ; j >= 0 ; j = j--)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j + 1] == BLACK)
                {
                    Image_Side[i][j] = BLACK;
                    Right_Line_Flag[i] = 1;
                    Right_Line[i] = j;
                    break;
                }
                else
                {
                    ;
                }
            }
        }
        //非第一行处理
        else
        {
            //往左扫
            for(j = Middle_Line[i - 1] ; j <= MT9V03X_W - 1 ; j = j++)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j - 1] == BLACK)//当前点为黑点且上一个为白点
                {
                    Image_Side[i][j] = BLACK;
                    Left_Line_Flag[i] = 1;
                    Left_Line[i] = j;
                    break;
                }
                else
                {
                    ;
                }
            }
            //往右扫
            for(j = Middle_Line[i - 1] ; j >= 0 ; j = j--)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j + 1] == BLACK)
                {
                    Image_Side[i][j] = BLACK;
                    Right_Line_Flag[i] = 1;
                    Right_Line[i] = j;
                    break;
                }
                else
                {
                    ;
                }
            }

        }

/*
        //初步特征值提取
        if(i <= 80 && Left_Line_Flag[i] == 1 && Right_Line_Flag[i] == 1)
        {
            All_Get_Times++;
        }
        if(i <= 40 && Left_Line_Flag[i] == 0 && Right_Line_Flag[i] == 0)
        {
            All_Lose_Times++;
        }
        if(i >= 30 && Left_Line[i] - Left_Line[i - 1] <= -20)
        {
            Left_Break_Line++;
        }
        if(i >= 30 && Right_Line[i] - Right_Line[i - 1] >= 20)
        {
            Right_Break_Line++;
        }
*/
        //中线值
        Middle_Line[i] = (Left_Line[i] + Right_Line[i]) / 2;
/*
        if( i<= 40)
        {
            Middle_Line_Sum1 += (94 - Middle_Line[i])*(94 - Middle_Line[i]);
            Middle_Line_Sum2 += (94 - Middle_Line[i]);
        }


//        //如果所得到的相邻中线点已经是黑色,则已经扫描出赛道,打断
//        if(Image_Binarization[i][Middle_Line[i]] == BLACK && Image_Binarization[i + 1][Middle_Line[i]] == BLACK )
//        {
//            Break_Line = i;
//            if(i > 30)//30行以内不会断线
//            {
//                break;
//            }
//        }

        Image_Side[i][Middle_Line[i]] = BLACK;//中线涂黑
*/
    }
/*
    //判断直道弯道
    if(sqrt(Middle_Line_Sum1) / 40 > 3.2 || Middle_Line_Sum2 / 40 > 18.5 || Middle_Line_Sum2 / 40 < -18.5)
    {
        Straight_Flag = 1;
    }
    else
    {
        Straight_Flag = 0;
    }
*/
}


/*
//找右下拐点
void Find_Right_Down_Point(int16 start_point , int16 end_point , uint8 road_name)
{
    int16 i = 0;
    if(road_name == 0)
    {
        for(i = start_point ; i >= end_point ; i--)
        {
            if(abs(Right_Line[i - 1] - Right_Line[i]) <= 3 && abs(Right_Line[i - 2] - Right_Line[i - 1]) <= 3 && (Right_Line[i - 3] - Right_Line[i - 2]) > 2
                    && Right_Line_Flag[i] == 1 && Right_Line_Flag[i - 1] == 1 && Right_Line_Flag[i - 2] == 1)
            {
                RightDown_Line_Break_Point[0] = Right_Line[i - 2];
                RightDown_Line_Break_Point[1] = i - 2;
            }
        }
    }
}


*/

void Add_Line(int16 x0, int16 y0, int16 x1, int16 y1)//补线
{
    int16 x, i;
    float K = (x0 - x1) * 1.0 / (y0 - y1);
    if (K > 20) K = 20;
    if (K < -20) K = -20;
    if (y1 > y0)
    {
        for (i = y0 + 1; i < y1; i++)
        {
            x = x0 + (i - y0) * K;
            Image_Binarization[i][x] = 0;
            Image_Binarization[i][x - 1] = 0;
            //        Use_Image[i-1][x-1] = 0;
            //        Use_Image[i-1][x-2] = 0;
        }
    }
    else
    {
        for (i = y1 + 1; i < y0; i++)
        {
            x = x0 + (i - y0) * K;
            Image_Binarization[i][x] = 0;
            Image_Binarization[i][x - 1] = 0;
            //        Use_Image[i-1][x-1] = 0;
            //        Use_Image[i-1][x-2] = 0;
        }
    }

}



void CameraWorking996()
{
    if(mt9v03x_finish_flag)
    {
        binarization();
        switch(DoImageFlag)
        {
          case 0:my_sobel(mt9v03x_image , Image_Side , T_OSTU);break;
          case 1:my_sobel(mt9v03x_image , Image_Side , SelfControl_OSTU);break;//自己调整大津法
          case 2:Side_Search();break;
        }
        switch(ShowFlag)
        {
            case 0:lcd_displayimage032(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);break;
            case 1:lcd_displayimage032(Image_Binarization[0],MT9V03X_W, MT9V03X_H);break;
            case 2:lcd_displayimage032(Image_Side[0],MT9V03X_W, MT9V03X_H);break;
            case 3:seekfree_sendimg_03x(UART_1, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);//使用上位机
        }

//        Find_Right_Down_Point(119 , 60 , 0);
        lcd_displayimage032(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);
        mt9v03x_finish_flag = 0;

    }

}






