/*
 * ICM20602_Angle_Get.c
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */


#include "headfile.h"
#include "ICM20602_Angle_Get.h"

//��������
#define PI   3.1415926f
#define delta_T     0.001f  // ��������1ms ��Ƶ��1KHZ
float icm_data_acc_x = 0;
float icm_data_acc_y = 0;
float icm_data_acc_z = 0;
float GyroOffset_Xdata = 0;
float GyroOffset_Ydata = 0;
float GyroOffset_Zdata = 0;
float icm_data_gyro_x = 0;
float icm_data_gyro_y = 0;
float icm_data_gyro_z = 0;
float I_ex, I_ey, I_ez;  // ������
float Q_info[4] = {1.0, 0, 0, 0};  // ��Ԫ����ʼ��
float eulerAngle;              // ŷ����
float icm_data;                  // ICM20602�ɼ���������ֵ
float icm_kp= 0.17;    // ���ٶȼƵ��������ʱ�������
float icm_ki= 0.004;   // �������������ʵĻ�������
float pitch = 0;
float roll = 0;
float yaw = 0;

/*************************************/
//����������
float Q_angle = 0.001;
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dtΪkalman�˲�������ʱ��
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float Angle_X_Final;
float Angle_Y_Final;
float ax;
 float ay ;
 float az;
 float Gyro_x;
float Gyro_y;
float Gyro_z;
float Angle_x_temp;
float Angle_y_temp;

void ICM_20602(void)
{
    icm20602_data_get();   //�õ����ٶȴ���������
       ax = (9.8*icm_acc_x)/8192;
         ay = (9.8*icm_acc_y)/8192;
         az = (9.8*icm_acc_z)/8192;
    Gyro_x = (icm_gyro_x)/16.4;
        Gyro_y = (icm_gyro_y)/16.4;
       Gyro_z = (icm_gyro_z)/16.4;

        //�ü��ٶȼ����������ˮƽ������ϵ֮��ļн�
       Angle_x_temp=(atan(ay/az))*180/3.14;
      Angle_y_temp=(atan(ax/az))*180/3.14;

        Kalman_Filter_X(Angle_x_temp,Gyro_x);  //�������˲�����X���
        Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //�������˲�����Y���
}


void Kalman_Filter_X(float Accel,float Gyro) //����������
{
    Angle_X_Final += (Gyro - Q_bias) * dt; //�������

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

    Pdot[1]= -PP[1][1];
    Pdot[2]= -PP[1][1];
    Pdot[3]= Q_gyro;

    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - Angle_X_Final;  //zk-�������

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_X_Final += K_0 * Angle_err;    //�������
    Q_bias        += K_1 * Angle_err;    //�������
    Gyro_x         = Gyro - Q_bias;  //���ֵ��������ƣ���΢�� = ���ٶ�

}

void Kalman_Filter_Y(float Accel,float Gyro) //����������
{
    Angle_Y_Final += (Gyro - Q_bias) * dt; //�������

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

    Pdot[1]=- PP[1][1];
    Pdot[2]=- PP[1][1];
    Pdot[3]=Q_gyro;

    PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
    PP[0][1] += Pdot[1] * dt;   // =����������Э����
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - Angle_Y_Final;  //zk-�������

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //����������Э����
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_Y_Final   += K_0 * Angle_err;  //�������
    Q_bias  += K_1 * Angle_err;  //�������
    Gyro_y   = Gyro - Q_bias;    //���ֵ��������ƣ���΢�� = ���ٶ�
//    lcd_showfloat(0, 1 , Angle_Y_Final, 5, 5);
}




void icm20602_data_get()
{
    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();
}



/**
 * @brief ��������Ư��ʼ��
 * ͨ���ɼ�һ���������ֵ�������������ƫ��ֵ��
 * ���� �����Ƕ�ȡ������ - ��Ʈֵ������ȥ�����ƫ������
 */
void gyroOffsetInit(void)
{
    GyroOffset_Xdata = 0;
    GyroOffset_Ydata = 0;
    GyroOffset_Zdata = 0;
    for (uint16 i = 0; i < 100; ++i)
    {
        get_icm20602_accdata_spi();
        GyroOffset_Xdata += icm_gyro_x;
        GyroOffset_Ydata += icm_gyro_y;
        GyroOffset_Zdata += icm_gyro_z;
        systick_delay_ms(STM0 , 5);    // ��� 1Khz
    }

    GyroOffset_Xdata /= 100;
    GyroOffset_Ydata /= 100;
    GyroOffset_Zdata /= 100;
}




/**
 * @brief ���ɼ�����ֵת��Ϊʵ������ֵ, ���������ǽ���ȥ��Ư����
 * ���ٶȼƳ�ʼ������ -> ������Χ: ��8g        ��Ӧ������: 4096 LSB/g
 * �����ǳ�ʼ������   -> ������Χ: ��2000 dps  ��Ӧ������: 16.4 LSB/dps   (degree per second)
 * @tips: gyro = (gyro_val / 16.4) ��/s = ((gyro_val / 16.4) * PI / 180) rad/s
 */
void icmGetValues(void)
{
    float alpha = 0.3;

    //һ�׵�ͨ�˲�����λg
    icm_data_acc_x = (((float) icm_acc_x) * alpha) / 4096 + icm_data_acc_x * (1 - alpha);
    icm_data_acc_y = (((float) icm_acc_y) * alpha) / 4096 + icm_data_acc_y * (1 - alpha);
    icm_data_acc_z = (((float) icm_acc_z) * alpha) / 4096 + icm_data_acc_z * (1 - alpha);

    //! �����ǽ��ٶȱ���ת��Ϊ�����ƽ��ٶ�: deg/s -> rad/s
    icm_data_gyro_x = ((float) icm_gyro_x - GyroOffset_Xdata) * PI / 180 / 16.4f;
    icm_data_gyro_y = ((float) icm_gyro_y - GyroOffset_Ydata) * PI / 180 / 16.4f;
    icm_data_gyro_z = ((float) icm_gyro_z - GyroOffset_Zdata) * PI / 180 / 16.4f;
//    lcd_showfloat(0, 1, icm_data_acc_x, 5, 5);
//    lcd_showfloat(0, 2, icm_gyro_x, 5, 5);
//    lcd_showfloat(0, 3, GyroOffset_Xdata, 5, 5);
}



/**
 * @brief �û����˲��㷨������������̬(�����ü��ٶȼ����������ǵĻ������)
 * ���ٶȼƶ���֮��������Ƚ����У��������ݼ��������̬���ţ������Ƕ������������У��������ݿ��ţ�������ʹ�û����������(�ڲ������㷨�Ŵ�̬���)��
 * ���ʹ����̬�����˲����������������ǣ��������ż��ٶȼơ�
 * @tips: n - ��������ϵ�� b - ��������ϵ
 */
void icmAHRSupdate()
{
    float halfT = 0.5 * delta_T;    // ��������һ��
    float vx, vy, vz;               // ��ǰ��̬��������������������ϵķ���
    float ex, ey, ez;               // ��ǰ���ټƲ�õ��������ٶ��������ϵķ������õ�ǰ��̬��������������������ϵķ��������

    float q0 = Q_info[0];  //��Ԫ��
    float q1 = Q_info[1];
    float q2 = Q_info[2];
    float q3 = Q_info[3];

    float q0q0 = q0 * q0;  //����ˣ������������
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;


    // ������ֹ״̬Ϊ-g ����������
    if(icm_data_acc_x * icm_data_acc_y * icm_data_acc_z == 0) // �Ӽƴ�����������״̬ʱ(��ʱg = 0)��������̬���㣬��Ϊ�������ĸ���������
        return;

    // �Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ� (a^b -> ��������ϵ�µļ��ٶ�)
    float norm = mysqrt(icm_data_acc_x * icm_data_acc_x + icm_data_acc_y * icm_data_acc_y + icm_data_acc_z * icm_data_acc_z);
    icm_data_acc_x = icm_data_acc_x / norm;
    icm_data_acc_y = icm_data_acc_y / norm;
    icm_data_acc_z = icm_data_acc_z / norm;


    // ��������ϵ���������������ϵķ���
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // g^b �� a^b ��������ˣ��õ������ǵ�У����������e��ϵ��
    ex = icm_data_acc_y * vz - icm_data_acc_z * vy;
    ey = icm_data_acc_z * vx - icm_data_acc_x * vz;
    ez = icm_data_acc_x * vy - icm_data_acc_y * vx;


    // ����ۼ�
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    // ʹ��PI�������������������(������Ư�����)
    icm_data_gyro_x = icm_data_gyro_x + icm_kp* ex + icm_ki* I_ex;
    icm_data_gyro_y = icm_data_gyro_y + icm_kp* ey + icm_ki* I_ey;
    icm_data_gyro_z = icm_data_gyro_z + icm_kp* ez + icm_ki* I_ez;


    // һ����������������Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪbϵ�����ǽ��ٶȡ�
    q0 = q0 + (-q1 * icm_data_gyro_x - q2 * icm_data_gyro_y - q3 * icm_data_gyro_z) *halfT;
    q1 = q1 + (q0 * icm_data_gyro_x + q2 * icm_data_gyro_z - q3 * icm_data_gyro_y) *halfT ;
    q2 = q2 + (q0 * icm_data_gyro_y - q1 * icm_data_gyro_z + q3 * icm_data_gyro_x) *halfT;
    q3 = q3 + (q0 * icm_data_gyro_z + q1 * icm_data_gyro_y - q2 * icm_data_gyro_x) *halfT;

    // ��λ����Ԫ���ڿռ���תʱ�������죬������ת�Ƕȣ������㷨�������Դ�����������任
    norm = mysqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    Q_info[0] = q0 / norm;
    Q_info[1] = q1 / norm;
    Q_info[2] = q2 / norm;
    Q_info[3] = q3 / norm;  // ��ȫ�ֱ�����¼��һ�μ������Ԫ��ֵ
        lcd_showfloat(0, 1, icm_data_gyro_x, 5, 5);
        lcd_showfloat(0, 2, q1 / norm, 5, 5);
        lcd_showfloat(0, 3, Q_info[1], 5, 5);
        lcd_showfloat(0, 4, norm, 5, 5);
        lcd_showfloat(0, 5, icm_data_gyro_z, 5, 5);
}



void IMU_quaterToEulerianAngles()
{
    float q0 = Q_info[0];
    float q1 = Q_info[1];
    float q2 = Q_info[2];
    float q3 = Q_info[3];
    pitch = asin(2 * q0 * q2 - 2 * q1 * q3) * 180 / PI;
    roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI;
    yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;
}


//void ComplementaryFiltering2()
//{
//    float angle_Filtering = 0;
//    float K2 = 0.2;
//
////    if (icm_acc_x >= 0)
////        icm_acc_x = -1;
//    float acc_x_1 = - icm_acc_x * 9.8 / 4096.0;             //ת����λ
//    float acc_z_1 = icm_acc_z * 9.8 / 4096.0;               //ת����λ
//    float gyro_y_1 = icm_gyro_y / 16.4;                     //ת����λ
//
//    float  anglespeed = - (gyro_y - 0.25);                 //���ƫ��
//    float angle_acc = atan((acc_z) / acc_x) * 57.3;       //���ٶȵõ��ĽǶȣ��ɼ����ƫ��
//
//    float x1 = (angle_acc - angle_Filtering) * (1 - K2) * (1 - K2);
//    float y1 = y1 + x1 * 0.001;
//    float x2 = y1 + 2 * (1 - K2) * (angle_acc - angle_Filtering) + anglespeed;
//    angle_Filtering = angle_Filtering + x2 * 0.001;
//    lcd_showfloat(0, 5, angle_Filtering, 5, 5);
//
//}

float mysqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*) & y;
    i = 0x5f3759df - (i>>1);
    y = *(float*) & i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float Angle_Get()
{
    icm20602_data_get();
//    gyroOffsetInit();
    icmGetValues();
    icmAHRSupdate();
    IMU_quaterToEulerianAngles();
//    lcd_showfloat(0, 0, pitch, 5, 10);
//    lcd_showfloat(0, 1, roll, 5, 10);
//    lcd_showfloat(0, 3, yaw, 5, 10);
    return roll;
}






