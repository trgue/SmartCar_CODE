/*
 * Camera.c
 *
 *  Created on: 2022年2月7日
 *      Author: guoguo
 */

//头文件包含
#include "headfile.h"
#include "Camera.h"



//变量定义
#define ShowFlag  2 //控制摄像头显示模式:模式0为输出原图像,模式1为输出二值化后图像,模式2为输出边缘图像


uint8 Image_Binarization[MT9V03X_H][MT9V03X_W] = {0};
uint8 Image_Soble[MT9V03X_H][MT9V03X_W] = {0};
uint8 T_OSTU = 0;






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
              Image_Binarization[i][j] = BLACK;
          }
          else
          {
              Image_Binarization[i][j] = WHITE;
          }
      }
    }
}


//基于sobel边缘检测算子的一种边缘检测
void my_sobel(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
{
    /** 卷积核大小 */
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

            temp[0] = fabs(temp[0]);
            temp[1] = fabs(temp[1]);
            temp[2] = fabs(temp[2]);
            temp[3] = fabs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            if (temp[0] > Threshold)
            {
                imageOut[i][j] = 1;
            }
            else
            {
                imageOut[i][j] = 0;
            }
        }
    }
}


void CameraWorking996()
{
    if(mt9v03x_finish_flag)
    {
//        uint8 av[2][1];
//        test(av);
        binarization();
        my_sobel(mt9v03x_image , Image_Soble , T_OSTU);
        switch(ShowFlag)
        {
            case 0:lcd_displayimage032(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);break;
            case 1:lcd_displayimage032(Image_Binarization[0],MT9V03X_W, MT9V03X_H);break;
            case 2:lcd_displayimage032(Image_Soble[0],MT9V03X_W, MT9V03X_H);
        }
        mt9v03x_finish_flag = 0;

    }

}


void test(uint8 ab[][1])
{
    int i = 0;
//    for(i = 0 ; i <= 1 ; i++)
//    {
//        ab[i][1] = 0;
//    }
}




