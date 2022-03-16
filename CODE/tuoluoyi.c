/*
 * tuoluoyi.c
 *
 *  Created on: 2021��11��4��
 *      Author: �ν���
 */
#include "tuoluoyi.h"

//get accel and gyro from iam20609
// ��accelһ�׵�ͨ�˲�(�ο�����)����gyroת�ɻ���ÿ��(2000dps)
#define new_weight           0.35f
#define old_weight           0.65f
#define icm_acc_x mpu_acc_x
#define icm_acc_y mpu_acc_y
#define icm_acc_z mpu_acc_z
#define icm_gyro_x mpu_gyro_x
#define icm_gyro_y mpu_gyro_y
#define icm_gyro_z mpu_gyro_z
#define M_PI 3.14159
float values[6];


//extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
//extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
//extern int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
//extern int16 ,icm_acc_y,icm_acc_z;


//�������ǻ�õ�ԭʼ���ݽ���һ�׵�ͨ�˲�����
void IMU_getValues(void)
{


    static float lastaccel[3]= {0,0,0};
    int i;
    values[0] = ((float)mpu_acc_x) * new_weight + lastaccel[0] * old_weight;
    values[1] = ((float)mpu_acc_y) * new_weight + lastaccel[1] * old_weight;
    values[2] = ((float)mpu_acc_z) * new_weight + lastaccel[2] * old_weight;
    for(i=0; i<3; i++)
    {
        lastaccel[i] = values[i];
    }

    values[3] = ((float)mpu_gyro_x) * M_PI / 180 / 16.4f;
    values[4] = ((float)mpu_gyro_y) * M_PI / 180 / 16.4f;
    values[5] = ((float)mpu_gyro_z) * M_PI / 180 / 16.4f;

    //
}

#define delta_T      0.005f  //5ms����һ��

float I_ex, I_ey, I_ez;  // ������
float Q_info_q0=1,Q_info_q1=0,Q_info_q2=0,Q_info_q3=0;  // ȫ����Ԫ��
float yaw,pitch,roll; //ŷ����
float param_Kp = 50.0;   // ���ٶȼ�(������)���������ʱ�������50
float param_Ki = 0.20;   //�������������ʵĻ������� 0.2


//������Ԫ���������������ݽ�����̬����
 void IMU_AHRSupdate_noMagnetic(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //��ǰ�Ļ�������ϵ�ϵ�������λ����
    float ex, ey, ez;    //��Ԫ������ֵ����ٶȼƲ���ֵ�����
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;

    float q1q1 = q1 * q1;

    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    //�Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ�
     float norm = invSqrt(ax*ax + ay*ay + az*az);
     ax = ax * norm;
     ay = ay * norm;
     az = az * norm;
     vx = 2*(q1q3 - q0q2);
     vy = 2*(q0q1 + q2q3);
     vz = q0q0 - q1q1 - q2q2 + q3q3;
     ex = ay * vz - az * vy;
       ey = az * vx - ax * vz;
       ez = ax * vy - ay * vx;
       //�ò���������PI����������ƫ��
           //ͨ������ param_Kp��param_Ki ����������
           //���Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶȡ�
           I_ex += delta_T * ex;   // integral error scaled by Ki
           I_ey += delta_T * ey;
           I_ez += delta_T * ez;

           gx = gx+ param_Kp*ex + param_Ki*I_ex;
           gy = gy+ param_Kp*ey + param_Ki*I_ey;
           gz = gz+ param_Kp*ez + param_Ki*I_ez;
           //��Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪ�����ǽ��ٶȣ����¶�����֪��������ʹ����һ��������������Ԫ��΢�ַ���
               q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
               q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
               q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
               q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;
           //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
               // ������Ԫ����    ��Ԫ��΢�ַ���  ��Ԫ�������㷨�����ױϿ���
           //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
           //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
           //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
           //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;
               // normalise quaternion
               norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
               Q_info_q0 = q0 * norm;
               Q_info_q1 = q1 * norm;
               Q_info_q2 = q2 * norm;
               Q_info_q3 = q3 * norm;
               IMU_getValues();
               IMU_AHRSupdate_noMagnetic( values[3],  values[4],  values[5],  values[0], values[1],  values[2]);
               pitch = asin(-2*q1*q3 + 2*q0*q2) * 180/M_PI; // pitch
                                 roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180/M_PI; // roll
                                yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180/M_PI; // yaw


                                 if(roll>90 || roll<-90)
                                 {
                                     if(pitch > 0)
                                     {
                                         pitch = 180-pitch;
                                     }
                                     if(pitch < 0)
                                     {
                                        pitch = -(180+pitch);
                                     }
                                 }
                                 if(yaw > 180)
                                 {
                                     yaw -=360;
                                 }
                                 else if(yaw <-180)
                                 {
                                     yaw +=360;
                                 }

}

               void IMU_quaterToEulerianAngles()
               {
                   IMU_getValues();
                   IMU_AHRSupdate_noMagnetic( values[3],  values[4],  values[5],  values[0], values[1],  values[2]);

                   float q0 = Q_info_q0;
                   float q1 = Q_info_q1;
                   float q2 = Q_info_q2;
                   float q3 = Q_info_q3;
                  pitch = asin(-2*q1*q3 + 2*q0*q2) * 180/M_PI; // pitch
                   roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180/M_PI; // roll
                  yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180/M_PI; // yaw


                   if(roll>90 || roll<-90)
                   {
                       if(pitch > 0)
                       {
                           pitch = 180-pitch;
                       }
                       if(pitch < 0)
                       {
                          pitch = -(180+pitch);
                       }
                   }
                   if(yaw > 180)
                   {
                       yaw -=360;
                   }
                   else if(yaw <-180)
                   {
                       yaw +=360;
                   }

               }

               float invSqrt(float x)
               {
float halfx = 0.5f * x;
 float y = x;
             long i = *(long*)&y;
         i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
      y = y * (1.5f - (halfx * y * y));
       return y;
               }





