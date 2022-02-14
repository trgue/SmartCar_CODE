/*
 * Camera.c
 *
 *  Created on: 2022��2��7��
 *      Author: guoguo
 */

//ͷ�ļ�����
#include "headfile.h"
#include "Camera.h"



//��������
#define ShowFlag  2 //��������ͷ��ʾģʽ:ģʽ0Ϊ���ԭͼ��,ģʽ1Ϊ�����ֵ����ͼ��,ģʽ2Ϊ�����Եͼ��,ģʽ3Ϊʹ����λ��
#define DoImageFlag   2 //����ͼ����ģʽ:ģʽ0Ϊsobel��Ե��ȡ����ֵ��̬���ƣ���ģʽ1Ϊsobel��Ե��ȡ����ֵ�ֶ����ƣ���ģʽ2Ϊһ�㷽��


uint8 Image_Binarization[MT9V03X_H][MT9V03X_W];
uint8 Image_Side[MT9V03X_H][MT9V03X_W];

uint8 T_OSTU = 0;
uint8 SelfControl_OSTU = 0;






//�������ֵ
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



//ͨ������������ֵ���ж�ֵ��
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











//����sobel��Ե������ӵ�һ�ֱ�Ե���
void my_sobel(unsigned char imageIn[MT9V03X_H][MT9V03X_W], unsigned char imageOut[MT9V03X_H][MT9V03X_W], unsigned char Threshold)
{
    /* ����˴�С */
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
            /* ���㲻ͬ�����ݶȷ�ֵ  */
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

            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
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

//һ�㷽��ɨ�߽�
void Side_Search()
{
    //��������
    int16 i,j;
    int16 Left_Line_Flag[MT9V03X_H] = {0};  //������Ƿ�ɨ����ʶ��
    int16 Right_Line_Flag[MT9V03X_H] = {0}; //�ұ����Ƿ�ɨ����ʶ��
    int16 Middle_Line[MT9V03X_H] = {0};      //���߼�
    int16 Left_Line[MT9V03X_H] = {0};       //����߼�
    int16 Right_Line[MT9V03X_H] = {0};      //�ұ��߼�

    //�����һ֡ͼ��
    for(i = 0 ; i < MT9V03X_H ; i++)
    {
        for(j = 0 ; j < MT9V03X_W ; j++)
        {
            Image_Side[i][j] = WHITE;
        }
    }


    //��ʼɨ��
    for(i = MT9V03X_H - 1 ; i >= 0 ; i--)
    {
        //��һ�д���
        if(i == MT9V03X_H - 1)
        {
            //����ɨ
            for(j = MT9V03X_W / 2 ; j >= 0 ; j = j--)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j + 1] == BLACK)//��ǰ��Ϊ�ڵ�����һ��Ϊ�׵�
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
            //����ɨ
            for(j = MT9V03X_W / 2 ; j <= MT9V03X_W - 1 ; j = j++)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j - 1] == BLACK)
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
            Middle_Line[i] = (Left_Line[i] + Right_Line[i]) / 2;
            Image_Side[i][Middle_Line[i]] = BLACK;//����Ϳ��

        }
        //�ǵ�һ�д���
        else
        {
            //����ɨ
            for(j = Middle_Line[i + 1] ; j >= 0 ; j = j--)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j + 1] == BLACK)//��ǰ��Ϊ�ڵ�����һ��Ϊ�׵�
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
            //����ɨ
            for(j = Middle_Line[i + 1] ; j <= MT9V03X_W - 1 ; j = j++)
            {
                if(Image_Binarization[i][j] == BLACK && Image_Binarization[i][j - 1] == BLACK)
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
            if(i <= 60 && Left_Line_Flag[i] == 1 && Right_Line_Flag[i] == 1)
            {
                ;
            }
            Middle_Line[i] = (Left_Line[i] + Right_Line[i]) / 2;

            //������õ����������ߵ��Ѿ��Ǻ�ɫ,���Ѿ�ɨ�������,���
            if(Image_Binarization[i][Middle_Line[i]] == BLACK && Image_Binarization[i + 1][Middle_Line[i]] == BLACK )
            {

            }

            Image_Side[i][Middle_Line[i]] = BLACK;//����Ϳ��

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
          case 1:my_sobel(mt9v03x_image , Image_Side , SelfControl_OSTU);break;//�Լ��������
          case 2:Side_Search();break;
//        seekfree_sendimg_03x(UART_1, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);//ʹ����λ��
        }
        switch(ShowFlag)
        {
            case 0:lcd_displayimage032(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);break;
            case 1:lcd_displayimage032(Image_Binarization[0],MT9V03X_W, MT9V03X_H);break;
            case 2:lcd_displayimage032(Image_Side[0],MT9V03X_W, MT9V03X_H);break;
            case 3:seekfree_sendimg_03x(UART_1, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);//ʹ����λ��
        }

        mt9v03x_finish_flag = 0;

    }

}






