/*
 * Balance.c
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */


#include "headfile.h"
#include "Balance.h"
#include "tuoluoyi.h"

/*
#include "BrushlessMotor_ADC.h"
#include "BrushlessMotor_HALL.h"
#include "BrushlessMotor.h"
#include "Filter.h"
*/

//��������
#define MOTOR_DEADNUM 0
#define MOTOR3_A   ATOM0_CH4_P02_4  //����3�����תPWM����
#define MOTOR3_B   ATOM0_CH5_P02_5  //����3�����תPWM����
#define SD_limit   150//����޷�
#define SD_middle  770//�����ֵ
int16 encoder_flywheel = 0;//�����ֱ�����
int16 limit_speed = 10000;//����������
uint8 Flag = 1;//�Ƿ�ǰ��
//float flywheel_balance_P = 1800;//ֱ��
//float flywheel_balance_I = 0.7;
//float flywheel_balance_D =35;
float flywheel_balance_P = 2500;
float flywheel_balance_I = 1;
float flywheel_balance_D =55;
float flywheel_speed_P = 10;
float flywheel_speed_I = 0;
float SD_B_P = 0;
float SD_B_I = 0;
float SD_B_D = 0;
float error_my = 0;//������PID�ǶȲ�
//float flywheel_speed_D = 0;
int16 flywheel_duty = 0;//�������ٶ�PWM
float Angle = 0;//�Ƕ�
float Angle_Zero = 0;//�Ƕ����
float Integration = 0;//���PID����
int16 SD_duty = 770;//���PWM
int16 SD_error = 0;//�������ֵ





//������ֵ��ȡ
void encoder_Flywheel_get()
{
    encoder_flywheel = gpt12_get(GPT12_T4);
    gpt12_clear(GPT12_T4);
}






//������PID����ƽ��
float flywheel_balance(float angle , float gyro)
{
    float PWM = 0;

    static float errors;
    error_my = angle - Angle_Zero;
    errors += error_my;
    if(errors > 30)
    {
        errors = 30;
    }
    if(errors < -30)
    {
        errors = -30;
    }
    PWM = error_my * flywheel_balance_P + errors * flywheel_balance_I + gyro * flywheel_balance_D;
    return PWM;
}


//�������ٶȱջ�
float flywheel_speed(int16 encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = encoder;                                  //�ٶ��˲�
    Encoder *= 0.7;                                           //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                             //һ�׵�ͨ�˲���
    Encoder_Integral += Encoder;                              //���ֳ�λ��
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;    //�����޷�
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;    //�����޷�
    Velocity = Encoder * flywheel_speed_P + Encoder_Integral * flywheel_speed_I/100;
                                                              //��ȡ������ֵ
    if(Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;//ֹͣʱ��������
    return Velocity;
}


//�����ֿ���(��ˢ���)
void flywheel_control_Brush()
{
    int16 PWM , PWM_error;
    Flag = 1;
    encoder_Flywheel_get();
    Angle = roll;
    Angle_Zero = 0;
    Angle_Zero = Angle_Zero + (SD_duty - 770)/100.0;//ƫ
    PWM = flywheel_balance(Angle , icm_gyro_x);
    PWM_error = flywheel_speed(encoder_flywheel);
    flywheel_duty = (PWM - PWM_error);
    //�޷�
    if(flywheel_duty < -limit_speed)
    {
        flywheel_duty = -limit_speed;
    }
    else if(flywheel_duty > limit_speed)
    {
        flywheel_duty = limit_speed;
    }
    //����У��
    else if(flywheel_duty > 0)
    {
        flywheel_duty += MOTOR_DEADNUM;
    }
    else if(flywheel_duty < 0)
    {
        flywheel_duty -= MOTOR_DEADNUM;
    }
    if((flywheel_duty < 1100)&&(flywheel_duty > -1100))
    {
        flywheel_duty = 0;
    }
    if(Angle < -25 || Angle > 25)
    {
        flywheel_duty = 0;
        Integration = 0;
        Flag = 0;
    }
    if(0<=flywheel_duty) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/GTM_ATOM0_PWM_DUTY_MAX*100)
    {
        pwm_duty(MOTOR3_A, flywheel_duty);
        pwm_duty(MOTOR3_B, 0);
    }
    else                //���1   ��ת
    {
        pwm_duty(MOTOR3_A, 0);
        pwm_duty(MOTOR3_B, -flywheel_duty);
    }



}

int16 SBB_Get_BalancePID(float Angle,float Gyro)
{
    float  Bias;
    int16 SBB_BalancePID;
    Bias = Angle - Angle_Zero;     // ���ƽ��ĽǶ���ֵ�ʹ�ʱ����ǵ�ƫ��
    Integration += Bias;           // ����
    if(Integration<-380)      Integration=-380; //�޷�
    else if(Integration>380)  Integration= 380; //�޷�
    //===����ƽ����ƵĶ��PWM  PID���� kp��Pϵ�� kiʽIϵ�� kd��Dϵ��
    SBB_BalancePID = SD_B_P * Bias + SD_B_I*Integration + SD_B_D*Gyro;
    return SBB_BalancePID;
}

void SD_Balance()
{
    SD_error = SBB_Get_BalancePID(roll , icm_gyro_x);
    SD_duty = 770 + SD_error;
    if(SD_duty >= SD_middle + SD_limit)
    {
        SD_duty = SD_middle + SD_limit;
    }
    if(SD_duty <= SD_middle - SD_limit)
    {
        SD_duty = SD_middle - SD_limit;
    }
    pwm_duty(ATOM0_CH1_P33_9 , SD_duty);
}

void Balance_Control()
{
    flywheel_control_Brush();
//    SD_Balance();
}


/*LLLL

/*
//�����ֿ���(��ˢ���)
//ʹ����ˢʱ�轫PWM�жϿ���
void flywheel_control_Brushless()
{
    //�ǶȻ�ȡ
    Angle = Angle_Get();
    motor_set_dir();
    //ͨ���ٶ�������������ǰ�ٶ�
    motor_speed_out();
    motor_control.set_speed = flywheel_balance(Angle , icm_acc_x);
    //����PI�ջ�����
    if(motor_control.dir == FORWARD)
        duty = (int16)closed_loop_pi_calc((float)(motor_control.set_speed - speed_filter.data_average));
    else
        duty = (int16)closed_loop_pi_calc((float)(motor_control.set_speed + speed_filter.data_average));
}

*/









