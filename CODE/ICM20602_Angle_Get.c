/*
 * ICM20602_Angle_Get.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */


#include "headfile.h"
#include "ICM20602_Angle_Get.h"

//变量定义
#define PI   3.1415926f
#define delta_T     0.001f  // 采样周期1ms 即频率1KHZ
float icm_data_acc_x = 0;
float icm_data_acc_y = 0;
float icm_data_acc_z = 0;
float GyroOffset_Xdata = 0;
float GyroOffset_Ydata = 0;
float GyroOffset_Zdata = 0;
float icm_data_gyro_x = 0;
float icm_data_gyro_y = 0;
float icm_data_gyro_z = 0;
float I_ex, I_ey, I_ez;  // 误差积分
float Q_info[4] = {1.0, 0, 0, 0};  // 四元数初始化
float eulerAngle;              // 欧拉角
float icm_data;                  // ICM20602采集的六轴数值
float icm_kp= 0.17;    // 加速度计的收敛速率比例增益
float icm_ki= 0.004;   // 陀螺仪收敛速率的积分增益
float pitch = 0;
float roll = 0;
float yaw = 0;

/*************************************/
//卡尔曼参数
float Q_angle = 0.001;
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dt为kalman滤波器采样时间
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
    icm20602_data_get();   //得到加速度传感器数据
       ax = (9.8*icm_acc_x)/8192;
         ay = (9.8*icm_acc_y)/8192;
         az = (9.8*icm_acc_z)/8192;
    Gyro_x = (icm_gyro_x)/16.4;
        Gyro_y = (icm_gyro_y)/16.4;
       Gyro_z = (icm_gyro_z)/16.4;

        //用加速度计算三个轴和水平面坐标系之间的夹角
       Angle_x_temp=(atan(ay/az))*180/3.14;
      Angle_y_temp=(atan(ax/az))*180/3.14;

        Kalman_Filter_X(Angle_x_temp,Gyro_x);  //卡尔曼滤波计算X倾角
        Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //卡尔曼滤波计算Y倾角
}


void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数
{
    Angle_X_Final += (Gyro - Q_bias) * dt; //先验估计

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]= -PP[1][1];
    Pdot[2]= -PP[1][1];
    Pdot[3]= Q_gyro;

    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - Angle_X_Final;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_X_Final += K_0 * Angle_err;    //后验估计
    Q_bias        += K_1 * Angle_err;    //后验估计
    Gyro_x         = Gyro - Q_bias;  //输出值（后验估计）的微分 = 角速度

}

void Kalman_Filter_Y(float Accel,float Gyro) //卡尔曼函数
{
    Angle_Y_Final += (Gyro - Q_bias) * dt; //先验估计

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

    Pdot[1]=- PP[1][1];
    Pdot[2]=- PP[1][1];
    Pdot[3]=Q_gyro;

    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - Angle_Y_Final;  //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    PP[0][0] -= K_0 * t_0;       //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    Angle_Y_Final   += K_0 * Angle_err;  //后验估计
    Q_bias  += K_1 * Angle_err;  //后验估计
    Gyro_y   = Gyro - Q_bias;    //输出值（后验估计）的微分 = 角速度
//    lcd_showfloat(0, 1 , Angle_Y_Final, 5, 5);
}




void icm20602_data_get()
{
    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();
}



/**
 * @brief 陀螺仪零漂初始化
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后续 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
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
        systick_delay_ms(STM0 , 5);    // 最大 1Khz
    }

    GyroOffset_Xdata /= 100;
    GyroOffset_Ydata /= 100;
    GyroOffset_Zdata /= 100;
}




/**
 * @brief 将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
 * 加速度计初始化配置 -> 测量范围: ±8g        对应灵敏度: 4096 LSB/g
 * 陀螺仪初始化配置   -> 测量范围: ±2000 dps  对应灵敏度: 16.4 LSB/dps   (degree per second)
 * @tips: gyro = (gyro_val / 16.4) °/s = ((gyro_val / 16.4) * PI / 180) rad/s
 */
void icmGetValues(void)
{
    float alpha = 0.3;

    //一阶低通滤波，单位g
    icm_data_acc_x = (((float) icm_acc_x) * alpha) / 4096 + icm_data_acc_x * (1 - alpha);
    icm_data_acc_y = (((float) icm_acc_y) * alpha) / 4096 + icm_data_acc_y * (1 - alpha);
    icm_data_acc_z = (((float) icm_acc_z) * alpha) / 4096 + icm_data_acc_z * (1 - alpha);

    //! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    icm_data_gyro_x = ((float) icm_gyro_x - GyroOffset_Xdata) * PI / 180 / 16.4f;
    icm_data_gyro_y = ((float) icm_gyro_y - GyroOffset_Ydata) * PI / 180 / 16.4f;
    icm_data_gyro_z = ((float) icm_gyro_z - GyroOffset_Zdata) * PI / 180 / 16.4f;
//    lcd_showfloat(0, 1, icm_data_acc_x, 5, 5);
//    lcd_showfloat(0, 2, icm_gyro_x, 5, 5);
//    lcd_showfloat(0, 3, GyroOffset_Xdata, 5, 5);
}



/**
 * @brief 用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
 * 加速度计对振动之类的噪声比较敏感，长期数据计算出的姿态可信；陀螺仪对振动噪声不敏感，短期数据可信，但长期使用积分误差严重(内部积分算法放大静态误差)。
 * 因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
 * @tips: n - 导航坐标系； b - 载体坐标系
 */
void icmAHRSupdate()
{
    float halfT = 0.5 * delta_T;    // 采样周期一半
    float vx, vy, vz;               // 当前姿态计算得来的重力在三轴上的分量
    float ex, ey, ez;               // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差

    float q0 = Q_info[0];  //四元数
    float q1 = Q_info[1];
    float q2 = Q_info[2];
    float q3 = Q_info[3];

    float q0q0 = q0 * q0;  //先相乘，方便后续计算
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;


    // 正常静止状态为-g 反作用力。
    if(icm_data_acc_x * icm_data_acc_y * icm_data_acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;

    // 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
    float norm = mysqrt(icm_data_acc_x * icm_data_acc_x + icm_data_acc_y * icm_data_acc_y + icm_data_acc_z * icm_data_acc_z);
    icm_data_acc_x = icm_data_acc_x / norm;
    icm_data_acc_y = icm_data_acc_y / norm;
    icm_data_acc_z = icm_data_acc_z / norm;


    // 载体坐标系下重力在三个轴上的分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
    ex = icm_data_acc_y * vz - icm_data_acc_z * vy;
    ey = icm_data_acc_z * vx - icm_data_acc_x * vz;
    ez = icm_data_acc_x * vy - icm_data_acc_y * vx;


    // 误差累加
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    // 使用PI控制器消除向量积误差(陀螺仪漂移误差)
    icm_data_gyro_x = icm_data_gyro_x + icm_kp* ex + icm_ki* I_ex;
    icm_data_gyro_y = icm_data_gyro_y + icm_kp* ey + icm_ki* I_ey;
    icm_data_gyro_z = icm_data_gyro_z + icm_kp* ez + icm_ki* I_ez;


    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
    q0 = q0 + (-q1 * icm_data_gyro_x - q2 * icm_data_gyro_y - q3 * icm_data_gyro_z) *halfT;
    q1 = q1 + (q0 * icm_data_gyro_x + q2 * icm_data_gyro_z - q3 * icm_data_gyro_y) *halfT ;
    q2 = q2 + (q0 * icm_data_gyro_y - q1 * icm_data_gyro_z + q3 * icm_data_gyro_x) *halfT;
    q3 = q3 + (q0 * icm_data_gyro_z + q1 * icm_data_gyro_y - q2 * icm_data_gyro_x) *halfT;

    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = mysqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    Q_info[0] = q0 / norm;
    Q_info[1] = q1 / norm;
    Q_info[2] = q2 / norm;
    Q_info[3] = q3 / norm;  // 用全局变量记录上一次计算的四元数值
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
//    float acc_x_1 = - icm_acc_x * 9.8 / 4096.0;             //转化单位
//    float acc_z_1 = icm_acc_z * 9.8 / 4096.0;               //转化单位
//    float gyro_y_1 = icm_gyro_y / 16.4;                     //转化单位
//
//    float  anglespeed = - (gyro_y - 0.25);                 //零点偏差
//    float angle_acc = atan((acc_z) / acc_x) * 57.3;       //加速度得到的角度，可加零点偏差
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






