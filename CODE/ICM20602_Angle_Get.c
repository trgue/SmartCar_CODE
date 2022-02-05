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
int16 Q_info[4] = {1, 0, 0, 0};  // 四元数初始化
float eulerAngle;              // 欧拉角
float icm_data;                  // ICM20602采集的六轴数值
float icm_kp= 0.17;    // 加速度计的收敛速率比例增益
float icm_ki= 0.004;   // 陀螺仪收敛速率的积分增益
float pitch = 0;
float roll = 0;
float yaw = 0;





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
    float norm = sqrt(icm_data_acc_x * icm_data_acc_x + icm_data_acc_y * icm_data_acc_y + icm_data_acc_z * icm_data_acc_z);
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
    q0 = q0 + (-q1 * icm_data_gyro_x - q2 * icm_data_gyro_y - q3 * icm_data_gyro_z) * halfT;
    q1 = q1 + (q0 * icm_data_gyro_x + q2 * icm_data_gyro_z - q3 * icm_data_gyro_y) * halfT;
    q2 = q2 + (q0 * icm_data_gyro_y - q1 * icm_data_gyro_z + q3 * icm_data_gyro_x) * halfT;
    q3 = q3 + (q0 * icm_data_gyro_z + q1 * icm_data_gyro_y - q2 * icm_data_gyro_x) * halfT;

    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info[0] = q0 / norm;
    Q_info[1] = q1 / norm;
    Q_info[2] = q2 / norm;
    Q_info[3] = q3 / norm;  // 用全局变量记录上一次计算的四元数值
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


float Angle_Get()
{
    icm20602_data_get();
    gyroOffsetInit();
    icmGetValues();
    icmAHRSupdate();
    IMU_quaterToEulerianAngles();
    return roll;
}






